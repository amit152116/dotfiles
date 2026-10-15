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
  },
}
T.ok(diag.start_collector "notify", "notify collector failed to start")

T.test("ERROR and WARN recorded, INFO and no-level not", function()
  vim.notify("bad thing", vim.log.levels.ERROR)
  vim.notify("careful", vim.log.levels.WARN)
  vim.notify("fyi", vim.log.levels.INFO)
  vim.notify "no level"
  local evs = T.queued "notify"
  T.eq(#evs, 2)
  T.eq(evs[1].data.level, "ERROR")
  T.eq(evs[1].data.msg, "bad thing")
  T.eq(evs[2].data.level, "WARN")
  T.ok(evs[1].data.trace:find("test_notify", 1, true), "trace has caller")
  T.eq(#calls, 4)
end)

T.test("non-string and huge messages are bounded", function()
  vim.notify({ code = 42 }, vim.log.levels.WARN)
  vim.notify(string.rep("y", 10000), vim.log.levels.ERROR)
  local evs = T.queued "notify"
  T.ok(evs[3].data.msg:find("code = 42", 1, true), "table inspected")
  T.eq(#evs[4].data.msg, 500)
  T.ok(#vim.split(evs[4].data.trace, "\n") <= 20, "trace bounded")
end)

T.test("own messages are not recorded", function()
  vim.notify("[diagnostics] write failed", vim.log.levels.ERROR)
  T.eq(#T.queued "notify", 4)
end)

T.test("re-wraps after another plugin replaces vim.notify", function()
  local replaced = {}
  vim.notify = function(msg) replaced[#replaced + 1] = msg end
  vim.api.nvim_exec_autocmds("User", { pattern = "LazyVimStarted" })
  vim.notify("after replace", vim.log.levels.ERROR)
  T.eq(#T.queued "notify", 5)
  T.eq(replaced[#replaced], "after replace")
end)

T.test("stop restores the underlying notify", function()
  diag.stop_collector "notify"
  vim.notify("x", vim.log.levels.ERROR)
  T.eq(#T.queued "notify", 5)
end)

T.done()
