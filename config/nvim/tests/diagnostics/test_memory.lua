local T = dofile "tests/diagnostics/helpers.lua"
local diag = require "diagnostics"

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
  },
  thresholds = { memory_growth_mb = 2 },
}
T.ok(diag.start_collector "memory", "memory collector failed to start")
local memory = require "diagnostics.collectors.memory"

T.test("no event without growth", function()
  memory.sample()
  T.eq(#T.queued "memory", 0)
end)

local hoard = {}
T.test("rss growth past a step emits once per step", function()
  for i = 1, 200000 do
    hoard[i] = string.rep("x", 20) .. i
  end
  memory.sample()
  memory.sample()
  local evs = T.queued "memory"
  T.eq(#evs, 1)
  T.ok(evs[1].data.growth_mb >= 2, "growth " .. evs[1].data.growth_mb)
  T.ok(evs[1].data.heap_mb > 0 and evs[1].data.rss_mb > 0, "sizes")
end)

T.test("a Lua heap spike alone is not growth", function()
  -- heap count includes uncollected garbage, so it overshoots between GC cycles
  diag.stop_collector "memory"
  local real = memory._snapshot
  local fake = { rss_mb = 300, heap_mb = 50 }
  memory._snapshot = function() return vim.deepcopy(fake) end
  diag.start_collector "memory"
  local before = #T.queued "memory"
  fake.heap_mb = 500
  memory.sample()
  T.eq(#T.queued "memory", before)
  fake.rss_mb = 303
  memory.sample()
  T.eq(#T.queued "memory", before + 1)
  memory._snapshot = real
end)

T.test("session summary is written on exit", function()
  vim.api.nvim_exec_autocmds("VimLeavePre", { group = "Diagnostics" })
  local summary
  for _, ev in ipairs(T.read_events(log_file)) do
    if ev.kind == "session_summary" then summary = ev.data end
  end
  T.ok(summary, "no session_summary")
  for _, key in ipairs {
    "duration_s",
    "utime_s",
    "stime_s",
    "maxrss_mb",
    "majflt",
    "nivcsw",
    "rss_mb",
    "heap_mb",
  } do
    T.ok(type(summary[key]) == "number", key)
  end
  T.ok(type(summary.redraws) == "number", "nvim__stats redraw")
end)

T.done()
