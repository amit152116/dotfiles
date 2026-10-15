local config = require "diagnostics.config"
local writer = require "diagnostics.writer"
local uv = vim.uv

local M = {}

local MB = 1024 * 1024

local timer, baseline, started_at
local level = 0

---@param n number|nil
---@return number|nil
local function round1(n) return n and math.floor(n * 10 + 0.5) / 10 end

---@param tv {sec: integer, usec: integer}
---@return number
local function seconds(tv) return tv.sec + tv.usec / 1e6 end

---@return {rss_mb: number, heap_mb: number, stats: table|nil}
function M._snapshot()
  -- nvim__stats is experimental; its keys may change
  local ok, stats = pcall(vim.api.nvim__stats)
  return {
    rss_mb = (uv.resident_set_memory() or 0) / MB,
    heap_mb = collectgarbage "count" / 1024,
    stats = ok and stats or nil,
  }
end

--- Emit once each time RSS grows another step past the baseline.
function M.sample()
  if not baseline then return end
  local s = M._snapshot()
  local step = config.get_value "thresholds.memory_growth_mb"
  -- rss only: the lua heap count includes uncollected garbage and overshoots
  -- between gc cycles; it is still reported alongside for context
  local growth = s.rss_mb - baseline.rss_mb
  if growth < level + step then return end
  level = math.floor(growth / step) * step
  writer.emit("memory", nil, {
    rss_mb = round1(s.rss_mb),
    heap_mb = round1(s.heap_mb),
    growth_mb = round1(growth),
    lua_refcount = s.stats and s.stats.lua_refcount,
    uptime_s = math.floor((uv.hrtime() - started_at) / 1e9),
  })
end

--- One record per session: CPU, peak memory and internal counters.
function M.on_exit()
  if not started_at then return end
  local ru = uv.getrusage()
  local s = M._snapshot()
  local stats = s.stats or {}
  writer.emit("session_summary", nil, {
    duration_s = math.floor((uv.hrtime() - started_at) / 1e9),
    utime_s = ru and round1(seconds(ru.utime)),
    stime_s = ru and round1(seconds(ru.stime)),
    maxrss_mb = ru and round1(ru.maxrss / 1024), -- linux reports KB
    majflt = ru and ru.majflt,
    nivcsw = ru and ru.nivcsw,
    oublock = ru and ru.oublock,
    rss_mb = round1(s.rss_mb),
    heap_mb = round1(s.heap_mb),
    redraws = stats.redraw,
    fsyncs = stats.fsync,
    ts_query_parses = stats.ts_query_parse_count,
    lua_refcount = stats.lua_refcount,
  })
end

---@param ctx {guard: fun(fn: function): function}
function M.start(ctx)
  if timer then return end
  started_at = started_at or uv.hrtime()
  baseline, level = M._snapshot(), 0
  local interval = config.get_value "thresholds.memory_interval_ms"
  local sample = ctx.guard(M.sample)
  timer = uv.new_timer()
  -- nvim__stats is an api call, so sample on the main loop
  timer:start(interval, interval, function() vim.schedule(sample) end)
end

function M.stop()
  if not timer then return end
  timer:stop()
  timer:close()
  timer, baseline = nil, nil
end

return M
