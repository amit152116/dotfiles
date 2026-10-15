local T = dofile "tests/diagnostics/helpers.lua"
local diag = require "diagnostics"

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
  thresholds = { input_latency_ms = 50 },
}
T.ok(diag.start_collector "input", "input collector failed to start")
local input = require "diagnostics.collectors.input"

-- a mapping whose handler is slow, like a heavy plugin keymap
vim.keymap.set("n", "Q", function() T.busy(80) end)

---@param keys string
local function press(keys)
  vim.api.nvim_feedkeys(keys, "xt", false)
  vim.api.nvim_exec_autocmds("SafeState", { modeline = false })
end

T.test("slow key handling emits input_latency without the key", function()
  press "Q"
  local evs = T.queued "input_latency"
  T.eq(#evs, 1)
  T.ok(evs[1].ms >= 80, "ms " .. evs[1].ms)
  T.eq(evs[1].data.mode, "n")
  T.eq(evs[1].data.key, nil)
end)

T.test("fast keys are only counted", function()
  press "l"
  press "h"
  T.eq(#T.queued "input_latency", 1)
end)

T.test("summary reports per mode and filetype percentiles", function()
  input.flush_summary()
  local s = T.queued("input_summary")[1]
  T.ok(s, "no summary")
  T.eq(s.data.mode, "n")
  T.eq(s.data.count, 3)
  T.ok(s.data.max >= 80, "max")
  T.ok(s.data.p50 < 50, "p50 " .. s.data.p50)
  input.flush_summary()
  T.eq(#T.queued "input_summary", 1)
end)

T.test("keys typed in terminal mode are not timed", function()
  -- SafeState never fires in terminal mode; a later mode change must not book the wait
  input._on_key("a", "a", "t")
  T.busy(80)
  vim.api.nvim_exec_autocmds("SafeState", { modeline = false })
  T.eq(#T.queued "input_latency", 1)
end)

T.test("a key injected by a plugin cancels the pending measurement", function()
  -- e.g. operator-pending "d", then a plugin feedkeys(<Esc>) without "t"
  input._on_key("d", "d", "n")
  T.busy(600) -- the human/idle wait before the plugin injects
  input._on_key("\27", "", "no")
  vim.api.nvim_exec_autocmds("SafeState", { modeline = false })
  T.eq(#T.queued "input_latency", 1)
end)

T.test("rhs keys of a string mapping keep the measurement", function()
  -- nvim reports rhs keys as untyped, exactly like plugin feedkeys
  vim.keymap.set("n", "X", "ll")
  -- room to move, or the first "l" fails and aborts the rest of the mapping
  vim.api.nvim_buf_set_lines(0, 0, -1, false, { "abcdefgh" })
  vim.api.nvim_win_set_cursor(0, { 1, 0 })
  input.flush_summary()
  local before = #T.queued "input_summary"
  press "X"
  input.flush_summary()
  T.eq(#T.queued "input_summary", before + 1)
end)

T.test("mode is the one active when the key was typed", function()
  vim.wait(1100) -- spike events are rate-limited to one per second
  input._on_key("\27", "\27", "i")
  T.busy(80)
  vim.api.nvim_exec_autocmds("SafeState", { modeline = false })
  local evs = T.queued "input_latency"
  T.eq(#evs, 2)
  T.eq(evs[2].data.mode, "i")
end)

T.test("SafeState without a key does nothing", function()
  input.flush_summary()
  local before = #T.queued "input_summary"
  vim.api.nvim_exec_autocmds("SafeState", { modeline = false })
  input.flush_summary()
  T.eq(#T.queued "input_summary", before)
end)

T.done()
