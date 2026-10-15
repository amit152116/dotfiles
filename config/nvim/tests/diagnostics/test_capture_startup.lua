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

T.test("jit capture keeps full module paths", function()
  T.ok(capture.start { backend = "jit" })
  hot()
  T.ok(capture.stop())
  local ev = T.queued("profile_capture")[1]
  local f = assert(io.open(ev.data.file))
  local text = f:read "*a"
  f:close()
  T.ok(
    text:find("tests/diagnostics/test_capture_startup.lua", 1, true),
    "profile lacks full path:\n" .. text
  )
end)

T.test("startup_from_env without the env var does nothing", function()
  vim.env.DIAG_PROFILE_STARTUP = nil
  T.eq(capture.startup_from_env(), false)
  T.eq(capture.running(), false)
end)

T.test("startup_from_env captures until VimEnter when headless", function()
  vim.env.DIAG_PROFILE_STARTUP = "1"
  for k in pairs(calls) do
    calls[k] = nil
  end
  T.eq(capture.startup_from_env(), true)
  T.ok(capture.running(), "not running")
  T.eq(capture.status().reason, "startup")
  T.ok(capture.startup_profiled, "startup_profiled not set")

  -- a collector restart mid-startup must not notify
  T.eq(diag.start_collector "profile", false)
  T.eq(#calls, 0, "notified during startup capture")

  vim.api.nvim_exec_autocmds("VimEnter", {})
  T.ok(
    vim.wait(1000, function() return not capture.running() end),
    "not stopped"
  )
  T.ok(diag.is_running "profile", "profile collector not resumed")
  local evs = T.queued "profile_capture"
  T.eq(evs[#evs].data.reason, "startup")
  T.eq(#calls, 1, "expected one notify with the file")
  T.ok(calls[1].msg:find(evs[#evs].data.file, 1, true), calls[1].msg)
  vim.env.DIAG_PROFILE_STARTUP = nil
end)

T.done()
