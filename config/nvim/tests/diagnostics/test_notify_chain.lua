local T = dofile "tests/diagnostics/helpers.lua"
local diag = require "diagnostics"

local delivered = {}
local function real(msg) delivered[#delivered + 1] = msg end
-- snacks-style one-shot shim: swaps in the real notifier on first call
vim.notify = function(...)
  vim.notify = real
  return real(...)
end

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
}
T.ok(diag.start_collector "notify", "notify collector failed to start")

T.test("keeps recording after a one-shot shim replaces vim.notify", function()
  vim.api.nvim_exec_autocmds("User", { pattern = "LazyVimStarted" })
  vim.notify("info first", vim.log.levels.INFO)
  vim.notify("warn later", vim.log.levels.WARN)
  vim.notify("error later", vim.log.levels.ERROR)
  T.eq(#T.queued "notify", 2)
  T.eq(delivered, { "info first", "warn later", "error later" })
end)

T.test("delegating plugin wrapper installed above us does not loop", function()
  local hops = 0
  local prev = vim.notify
  vim.notify = function(...)
    hops = hops + 1
    if hops > 5 then error "notify loop" end
    return prev(...)
  end
  vim.api.nvim_exec_autocmds("User", { pattern = "LazyVimStarted" })
  vim.notify("hello", vim.log.levels.ERROR)
  T.eq(hops, 1)
  T.eq(#T.queued "notify", 3)
  T.eq(delivered[#delivered], "hello")
end)

T.test("stopped wrapper left inside a chain stops recording", function()
  diag.stop_collector "notify"
  vim.notify("after stop", vim.log.levels.ERROR)
  T.eq(#T.queued "notify", 3)
  T.eq(delivered[#delivered], "after stop")
end)

T.done()
