local T = dofile "tests/diagnostics/helpers.lua"
local diag = require "diagnostics"

local calls = T.capture_notify()
local log_file = T.tmpdir() .. "/d.jsonl"
diag.setup {
  log_file = log_file,
  flush_ms = 60000, -- keep events in the queue so T.queued sees them
  collectors = {
    startup = false,
    stall = false,
    bigfile = false,
    lsp = false,
    notify = false,
    profile = false,
    memory = false,
    input = false,
    autoprofile = false,
  },
}
local capture = require "diagnostics.capture"

local function hot()
  local x = 0
  for i = 1, 2e7 do
    x = x + i % 3
  end
  return x
end

T.test("jit capture pauses the stall profiler and writes a file", function()
  T.ok(diag.start_collector "profile")
  T.ok(capture.start { backend = "jit" })
  T.ok(capture.running(), "not running")
  T.eq(diag.is_running "profile", false)
  hot()
  T.ok(capture.stop())
  T.eq(capture.running(), false)
  T.ok(diag.is_running "profile", "profile collector not resumed")
  local ev = T.queued("profile_capture")[1]
  T.eq(ev.data.backend, "jit")
  T.eq(ev.data.reason, "manual")
  T.eq(vim.fs.dirname(ev.data.file), vim.fs.dirname(log_file))
  local f = assert(io.open(ev.data.file))
  T.ok(f:read("*a"):find("test_capture.lua", 1, true), "profile lacks hot fn")
  f:close()
end)

T.test("profile collector that was off stays off", function()
  diag.stop_collector "profile"
  T.ok(capture.start { backend = "jit" })
  T.ok(capture.stop())
  T.eq(diag.is_running "profile", false)
end)

T.test("a second start is refused, stop when idle is refused", function()
  T.ok(capture.start { backend = "jit" })
  local ok, err = capture.start { backend = "jit" }
  T.eq(ok, false)
  T.ok(err:find("already", 1, true), err)
  T.ok(capture.stop())
  ok, err = capture.stop()
  T.eq(ok, false)
  T.ok(err:find("not running", 1, true), err)
end)

T.test("timed capture stops itself", function()
  T.ok(capture.start { backend = "jit", duration_s = 0.2 })
  T.ok(
    vim.wait(1500, function() return not capture.running() end, 20),
    "still running"
  )
  T.eq(#T.queued "profile_capture", 4)
end)

T.test("snacks backend without snacks is refused", function()
  local ok, err = capture.start { backend = "snacks" }
  T.eq(ok, false)
  T.ok(err:find("snacks", 1, true), err)
  T.eq(capture.running(), false)
end)

T.test("snacks backend drives snacks.profiler", function()
  local log = {}
  local running = false
  package.loaded["snacks.profiler"] = {
    start = function()
      running = true
      log[#log + 1] = "start"
    end,
    stop = function()
      running = false
      log[#log + 1] = "stop"
    end,
    running = function() return running end,
  }
  T.ok(capture.start { backend = "snacks" })
  T.ok(capture.stop())
  T.eq(log, { "start", "stop" })
  local ev = T.queued "profile_capture"
  T.eq(ev[#ev].data.backend, "snacks")
  T.eq(ev[#ev].data.file, nil)
  package.loaded["snacks.profiler"] = nil
end)

T.test(":DiagProfile starts, reports and stops", function()
  vim.cmd "DiagProfile start jit"
  T.ok(capture.running())
  vim.cmd "DiagProfile"
  local own = T.own(calls)
  T.ok(own[#own].msg:find("running", 1, true), own[#own].msg)
  vim.cmd "DiagProfile stop"
  T.eq(capture.running(), false)
end)

T.test(":DiagProfile rejects bad arguments", function()
  local before = #T.own(calls)
  vim.cmd "DiagProfile start perf"
  vim.cmd "DiagProfile start jit soon"
  vim.cmd "DiagProfile restart"
  local own = T.own(calls)
  T.eq(#own, before + 3)
  for i = before + 1, #own do
    T.eq(own[i].level, vim.log.levels.ERROR)
  end
  T.eq(capture.running(), false)
end)

T.test("capture still running at exit is stopped and logged", function()
  T.ok(capture.start { backend = "jit" })
  vim.api.nvim_exec_autocmds("VimLeavePre", { group = "Diagnostics" })
  T.eq(capture.running(), false)
  local captures = vim.tbl_filter(
    function(e) return e.kind == "profile_capture" end,
    T.read_events(log_file)
  )
  T.eq(#captures, 7)
end)

T.done()
