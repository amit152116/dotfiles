local T = dofile "tests/diagnostics/helpers.lua"
local diag = require "diagnostics"

T.capture_notify()
diag.setup {
  log_file = T.tmpdir() .. "/d.jsonl",
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
  },
  thresholds = { heartbeat_ms = 50, stall_ms = 100 },
}

T.test("DiagToggle starts the collector", function()
  vim.cmd "DiagToggle stall"
  T.ok(diag.is_running "stall")
end)

T.test("idle loop produces no stall", function()
  vim.wait(500)
  T.eq(#T.queued "stall", 0)
end)

T.test("blocked loop produces one stall with context", function()
  require("diagnostics.writer").emit("marker", 1)
  T.busy(400)
  T.ok(
    vim.wait(500, function() return #T.queued "stall" > 0 end, 10),
    "no stall recorded"
  )
  local ev = T.queued("stall")[1]
  T.ok(ev.ms >= 250, "lateness " .. ev.ms)
  T.ok(ev.data.elapsed_ms >= ev.ms, "elapsed covers lateness")
  T.eq(ev.data.mode, "n")
  T.eq(ev.data.recent[#ev.data.recent].kind, "marker")
  T.eq(#T.queued "stall", 1)
end)

T.test("DiagToggle stops it and stop is idempotent", function()
  vim.cmd "DiagToggle stall"
  T.eq(diag.is_running "stall", false)
  require("diagnostics.collectors.stall").stop()
end)

T.done()
