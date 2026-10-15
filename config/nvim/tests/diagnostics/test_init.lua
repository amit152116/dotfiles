local T = dofile "tests/diagnostics/helpers.lua"
local diag = require "diagnostics"

local starts, stops = 0, 0
local fake_guard
package.preload["diagnostics.collectors.fake"] = function()
  return {
    start = function(ctx)
      starts = starts + 1
      fake_guard = ctx.guard
    end,
    stop = function() stops = stops + 1 end,
  }
end
package.preload["diagnostics.collectors.boom"] = function()
  return {
    start = function() error "start exploded" end,
    stop = function() stops = stops + 1 end,
  }
end

T.test("start_collector is idempotent", function()
  T.ok(diag.start_collector "fake")
  T.ok(diag.start_collector "fake")
  T.eq(starts, 1)
  T.ok(diag.is_running "fake")
  diag.stop_collector "fake"
  diag.stop_collector "fake"
  T.eq(diag.is_running "fake", false)
  T.eq(stops, 1)
end)

T.test("collector failing at start is cleaned up and reported", function()
  local calls = T.capture_notify()
  stops = 0
  T.eq(diag.start_collector "boom", false)
  T.eq(diag.is_running "boom", false)
  T.eq(stops, 1)
  local own = T.own(calls)
  T.eq(#own, 1)
  T.ok(own[1].msg:find("boom", 1, true), "names the collector")
  T.eq(own[1].level, vim.log.levels.ERROR)
end)

T.test("runtime error through guard disables collector once", function()
  local calls = T.capture_notify()
  stops = 0
  diag.start_collector "fake"
  local bad = fake_guard(function() error "tick exploded" end)
  bad()
  bad()
  vim.wait(100, function() return not diag.is_running "fake" end)
  T.eq(diag.is_running "fake", false)
  T.eq(stops, 1)
  local own = T.own(calls)
  T.eq(#own, 1)
  T.ok(own[1].msg:find("tick exploded", 1, true), "includes the error")
end)

T.test("unknown collector reports error", function()
  local calls = T.capture_notify()
  T.eq(diag.start_collector "nope", false)
  T.eq(#T.own(calls), 1)
end)

local OFF = {
  startup = false,
  stall = false,
  bigfile = false,
  lsp = false,
  notify = false,
  profile = false,
  memory = false,
  input = false,
}
local log_file = T.tmpdir() .. "/d.jsonl"

T.test("setup is idempotent", function()
  diag.setup { log_file = log_file, collectors = OFF }
  diag.setup { log_file = log_file .. ".other", collectors = OFF }
  T.eq(
    #vim.api.nvim_get_autocmds { group = "Diagnostics", event = "VimLeavePre" },
    1
  )
  T.eq(require("diagnostics.writer").status().path, log_file)
  T.eq(vim.g.diagnostics_setup_done, true)
end)

T.test("VimLeavePre drains the writer", function()
  require("diagnostics.writer").emit("x", 1)
  vim.api.nvim_exec_autocmds("VimLeavePre", { group = "Diagnostics" })
  T.eq(#T.read_events(log_file), 1)
end)

T.done()
