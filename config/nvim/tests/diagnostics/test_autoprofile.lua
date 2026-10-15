local T = dofile "tests/diagnostics/helpers.lua"
local diag = require "diagnostics"

local calls = T.capture_notify()
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
    autoprofile = false,
  },
  thresholds = {
    profile_auto_stalls = 2,
    profile_auto_window_s = 60,
    profile_auto_duration_s = 0.2,
  },
}
local capture = require "diagnostics.capture"

local function stall()
  vim.api.nvim_exec_autocmds(
    "User",
    { pattern = "DiagnosticsStall", modeline = false, data = { ms = 400 } }
  )
end

T.test(
  "autoprofile is off by default",
  function()
    T.eq(require("diagnostics.config").defaults.collectors.autoprofile, false)
  end
)

T.test("one stall does not arm a capture", function()
  T.ok(diag.start_collector "autoprofile")
  stall()
  T.eq(capture.running(), false)
end)

T.test("repeated stalls start a timed auto capture", function()
  stall()
  T.ok(capture.running(), "not armed")
  local own = T.own(calls)
  T.ok(own[#own].msg:find("auto profile", 1, true), own[#own].msg)
  T.ok(
    vim.wait(1500, function() return not capture.running() end, 20),
    "did not stop"
  )
  local ev = T.queued("profile_capture")[1]
  T.eq(ev.data.reason, "auto")
end)

T.test("no second auto capture inside the cooldown window", function()
  stall()
  stall()
  stall()
  T.eq(capture.running(), false)
  T.eq(#T.queued "profile_capture", 1)
end)

T.test("a manual capture in progress is left alone", function()
  diag.stop_collector "autoprofile"
  T.ok(diag.start_collector "autoprofile") -- fresh window and cooldown
  T.ok(capture.start { backend = "jit" })
  stall()
  stall()
  T.ok(capture.stop())
  T.eq(T.queued("profile_capture")[2].data.reason, "manual")
end)

T.test("stall collector announces stalls for autoprofile", function()
  diag.stop_collector "autoprofile"
  local seen = 0
  vim.api.nvim_create_autocmd("User", {
    pattern = "DiagnosticsStall",
    callback = function() seen = seen + 1 end,
  })
  require("diagnostics.config").options.thresholds.heartbeat_ms = 50
  require("diagnostics.config").options.thresholds.stall_ms = 100
  T.ok(diag.start_collector "stall")
  vim.wait(150)
  T.busy(400)
  T.ok(vim.wait(500, function() return seen > 0 end, 10), "no User event")
end)

T.done()
