local config = require "diagnostics.config"

local M = {}

local RING_SIZE = 256
local STACK_DEPTH = 5

local ring, head = {}, 0
local running = false

--- Store one sample; split out so tests can inject samples.
---@param vm string
---@param samples integer
---@param stack string
---@param t number hrtime
function M._record(vm, samples, stack, t)
  -- our own heartbeat is often the next Lua to run after a native block
  local first = stack:match "^[^;]*"
  if first:find("/lua/diagnostics/", 1, true) then return end
  head = head % RING_SIZE + 1
  ring[head] = { t = t, stack = stack, vm = vm, samples = samples }
end

--- Most frequent stacks sampled in [from_ns, to_ns] (hrtime), most first.
---@param from_ns number
---@param to_ns number
---@param n integer
---@return {stack: string, count: integer, vm: string, deferred: boolean|nil}[]
function M.top(from_ns, to_ns, n)
  local by = {}
  for _, s in pairs(ring) do
    if s.t >= from_ns and s.t <= to_ns then
      local entry = by[s.stack] or { stack = s.stack, count = 0, vm = s.vm }
      by[s.stack] = entry
      entry.count = entry.count + 1
      -- samples piled up while no Lua ran (C, vimscript, I/O): this stack is
      -- whatever Lua ran next, so it may not be the code that blocked
      if s.samples > 1 then entry.deferred = true end
    end
  end
  local out = vim.tbl_values(by)
  table.sort(out, function(a, b) return a.count > b.count end)
  return vim.list_slice(out, 1, n)
end

---@return boolean
function M.running() return running end

---@param ctx {guard: fun(fn: function): function}
function M.start(ctx)
  if running then return end
  local profile = require "jit.profile"
  local interval = config.get_value "thresholds.profile_interval_ms"
  -- callback runs on its own coroutine, so it can read a stack that is hogging the loop
  -- WARN(diagnostics): only one jit.profile can run; another profiler (perfanno, jit.p) replaces this one
  profile.start(
    "li" .. interval,
    ctx.guard(function(thread, samples, vmstate)
      M._record(
        vmstate,
        samples,
        -- innermost frame first: the hot line leads
        profile.dumpstack(thread, "pl;", STACK_DEPTH),
        vim.uv.hrtime()
      )
    end)
  )
  running = true
end

function M.stop()
  if not running then return end
  require("jit.profile").stop()
  running = false
  ring, head = {}, 0
end

return M
