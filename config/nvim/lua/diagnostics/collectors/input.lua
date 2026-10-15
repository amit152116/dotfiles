local config = require "diagnostics.config"
local writer = require "diagnostics.writer"
local uv = vim.uv

local M = {}

local GROUP = "Diagnostics_input"
local SUMMARY_MS = 60000
local MAX_SAMPLES = 1000
local MIN_EVENT_GAP_NS = 1e9
-- longer freezes than this are logged by the stall collector anyway
local INJECTED_GAP_MS = 500

local ns, timer
local last_key_t, last_key_mode
local latency_ms
local last_event_t = 0
local buckets = {}

---@param sorted number[]
---@param q number
---@return number
local function quantile(sorted, q)
  return sorted[math.max(1, math.ceil(#sorted * q))]
end

--- Emit one summary per mode+filetype seen since the last flush.
function M.flush_summary()
  for key, samples in pairs(buckets) do
    table.sort(samples)
    local mode, ft = key:match "^(.-)|(.*)$"
    writer.emit("input_summary", nil, {
      mode = mode,
      ft = ft,
      count = #samples,
      p50 = quantile(samples, 0.5),
      p95 = quantile(samples, 0.95),
      max = samples[#samples],
    })
  end
  buckets = {}
end

--- Start timing on a typed key; split out so tests can pass the mode.
---@param key string
---@param typed string
---@param mode string mode when the key arrived
function M._on_key(key, typed, mode)
  if typed == nil or typed == "" then
    -- untyped: mapping rhs keys (same burst) or a plugin's feedkeys. A late one
    -- likely ended an idle wait, e.g. operator-pending "d", so drop the timing
    -- NOTE(diagnostics): an injected key within INJECTED_GAP_MS still counts
    if
      key ~= ""
      and last_key_t
      and (uv.hrtime() - last_key_t) / 1e6 > INJECTED_GAP_MS
    then
      last_key_t, last_key_mode = nil, nil
    end
    return
  end
  -- terminal mode sends keys to the job and never reaches SafeState
  if mode:sub(1, 1) == "t" then
    last_key_t, last_key_mode = nil, nil
    return
  end
  -- last key, not first: a mapping prefix waits on the human, not on nvim
  last_key_t, last_key_mode = uv.hrtime(), mode
end

-- NOTE(diagnostics): measures key -> SafeState (input fully processed); redraw time is not included
local function on_safe_state()
  if not last_key_t then return end
  local ms = (uv.hrtime() - last_key_t) / 1e6
  local mode = last_key_mode
  last_key_t, last_key_mode = nil, nil
  local ft = vim.bo.filetype
  local key = mode .. "|" .. ft
  local samples = buckets[key] or {}
  buckets[key] = samples
  if #samples < MAX_SAMPLES then
    samples[#samples + 1] = math.floor(ms * 10 + 0.5) / 10
  end
  local now = uv.hrtime()
  if ms > latency_ms and now - last_event_t > MIN_EVENT_GAP_NS then
    last_event_t = now
    -- never log which key: typed text can be a password
    writer.emit("input_latency", ms, { mode = mode, ft = ft })
  end
end

---@param ctx {guard: fun(fn: function): function}
function M.start(ctx)
  if ns then return end
  ns = vim.api.nvim_create_namespace "diagnostics_input"
  -- cached: SafeState fires constantly and get_value allocates
  latency_ms = config.get_value "thresholds.input_latency_ms"
  vim.on_key(
    function(key, typed) M._on_key(key, typed, vim.api.nvim_get_mode().mode) end,
    ns
  )
  vim.api.nvim_create_autocmd("SafeState", {
    group = vim.api.nvim_create_augroup(GROUP, { clear = true }),
    callback = ctx.guard(on_safe_state),
  })
  timer = uv.new_timer()
  timer:start(
    SUMMARY_MS,
    SUMMARY_MS,
    function() vim.schedule(ctx.guard(M.flush_summary)) end
  )
end

function M.on_exit() M.flush_summary() end

function M.stop()
  if not ns then return end
  vim.on_key(nil, ns)
  pcall(vim.api.nvim_del_augroup_by_name, GROUP)
  if timer then
    timer:stop()
    timer:close()
    timer = nil
  end
  ns, last_key_t, buckets = nil, nil, {}
end

return M
