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
  thresholds = { heartbeat_ms = 50, stall_ms = 100, profile_interval_ms = 10 },
}

-- stands in for a plugin callback that blocks the loop
local function frozen_plugin_callback()
  local start = vim.uv.hrtime()
  while (vim.uv.hrtime() - start) / 1e6 < 400 do
  end
end

T.test("profile collector starts and stops", function()
  T.ok(diag.start_collector "profile")
  T.ok(diag.is_running "profile")
end)

T.test("stall carries the stacks that blocked the loop", function()
  T.ok(diag.start_collector "stall")
  vim.wait(150)
  frozen_plugin_callback()
  T.ok(
    vim.wait(500, function() return #T.queued "stall" > 0 end, 10),
    "no stall recorded"
  )
  local stacks = T.queued("stall")[1].data.stacks
  T.ok(stacks and #stacks > 0, "no stacks attached")
  T.ok(stacks[1].stack:find("test_profile.lua", 1, true), stacks[1].stack)
  T.ok(stacks[1].count >= 10, "count " .. stacks[1].count)
  T.ok(type(stacks[1].vm) == "string", "vm state")
end)

T.test("samples queued outside Lua are flagged as deferred", function()
  local profile = require "diagnostics.collectors.profile"
  local t0 = vim.uv.hrtime()
  -- a callback carrying many samples means they piled up while no Lua ran
  profile._record("C", 25, "next.lua:1;", t0 + 1)
  profile._record("I", 1, "hot.lua:9;", t0 + 2)
  local stacks = profile.top(t0, t0 + 10, 3)
  local by = {}
  for _, s in ipairs(stacks) do
    by[s.stack] = s
  end
  T.eq(by["next.lua:1;"].deferred, true)
  T.eq(by["hot.lua:9;"].deferred, nil)
end)

T.test(
  "samples landing in the diagnostics plugin itself are ignored",
  function()
    local profile = require "diagnostics.collectors.profile"
    local t0 = vim.uv.hrtime() + 1e12 -- a window no real sample falls in
    profile._record(
      "I",
      1,
      "/c/nvim/lua/diagnostics/init.lua:44;/x/y.lua:1;",
      t0
    )
    profile._record("I", 1, "/x/plugin.lua:7;", t0 + 1)
    local stacks = profile.top(t0, t0 + 10, 3)
    T.eq(#stacks, 1)
    T.eq(stacks[1].stack, "/x/plugin.lua:7;")
  end
)

T.test("stall records that profiling was on", function()
  local stall = T.queued("stall")[1]
  T.eq(stall.data.profiled, true)
end)

T.test("stall without profile has no stacks", function()
  diag.stop_collector "profile"
  T.eq(require("diagnostics.collectors.profile").top(0, math.huge, 3), {})
  frozen_plugin_callback()
  T.ok(
    vim.wait(500, function() return #T.queued "stall" > 1 end, 10),
    "no second stall"
  )
  T.eq(T.queued("stall")[2].data.stacks, nil)
end)

T.done()
