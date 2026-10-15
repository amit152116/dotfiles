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
  },
}

T.test("lazy absent: collector starts and stays silent", function()
  T.ok(diag.start_collector "startup")
  T.eq(#T.queued(), 0)
  diag.stop_collector "startup"
end)

local plugins = {
  slow = {
    name = "slow",
    _ = { loaded = { time = 12e6, event = "BufReadPost" } },
  },
  fast = { name = "fast", _ = { loaded = { time = 1e6, start = "start" } } },
  idle = { name = "idle", _ = {} },
}
local stats =
  { startuptime = 0, count = 3, loaded = 2, times = { LazyStart = 1.5 } }
package.loaded["lazy"] = { stats = function() return stats end }
package.loaded["lazy.core.config"] = { plugins = plugins }

T.test("headless start records unavailable startup and slow plugins", function()
  T.ok(diag.start_collector "startup")
  local s = T.queued("startup")[1]
  T.eq(s.ms, nil)
  T.eq(s.data.available, false)
  T.eq(s.data.count, 3)
  local loads = T.queued "plugin_load"
  T.eq(#loads, 1)
  T.eq(loads[1].data.name, "slow")
  T.eq(loads[1].data.reason, "event:BufReadPost")
  T.eq(loads[1].ms, 12)
end)

T.test(
  "scheduled LazyLoad for a plugin seen at startup is not duplicated",
  function()
    -- lazy fires LazyLoad via vim.schedule, so it can arrive after the startup pass
    vim.api.nvim_exec_autocmds("User", { pattern = "LazyLoad", data = "slow" })
    T.eq(#T.queued "plugin_load", 1)
  end
)

T.test("LazyLoad after startup records late plugin", function()
  plugins.late = { name = "late", _ = { loaded = { time = 8e6, cmd = "Foo" } } }
  vim.api.nvim_exec_autocmds("User", { pattern = "LazyLoad", data = "late" })
  vim.api.nvim_exec_autocmds(
    "User",
    { pattern = "LazyLoad", data = "unknown-plugin" }
  )
  local loads = T.queued "plugin_load"
  T.eq(#loads, 2)
  T.eq(loads[2].data.reason, "cmd:Foo")
end)

T.test("changed lazy shape warns once and stops per-plugin times", function()
  local calls = T.capture_notify()
  plugins.bad = { name = "bad", _ = { loaded = { time = "soon" } } }
  vim.api.nvim_exec_autocmds("User", { pattern = "LazyLoad", data = "bad" })
  plugins.late2 =
    { name = "late2", _ = { loaded = { time = 9e6, cmd = "Bar" } } }
  vim.api.nvim_exec_autocmds("User", { pattern = "LazyLoad", data = "late2" })
  local own = T.own(calls)
  T.eq(#own, 1)
  T.eq(own[1].level, vim.log.levels.WARN)
  T.eq(#T.queued "plugin_load", 2)
  diag.stop_collector "startup"
end)

T.test("UI session waits for LazyVimStarted", function()
  local startup = require "diagnostics.collectors.startup"
  startup.headless = function() return false end
  startup._reset()
  plugins.bad = nil
  T.ok(diag.start_collector "startup")
  T.eq(#T.queued "startup", 1)
  stats.startuptime = 95.5
  vim.api.nvim_exec_autocmds("User", { pattern = "LazyVimStarted" })
  local s = T.queued("startup")[2]
  T.eq(s.ms, 95.5)
  T.eq(s.data.available, true)
end)

T.done()
