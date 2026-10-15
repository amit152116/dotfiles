local config = require "diagnostics.config"
local uv = vim.uv

local M = {}

local GROUP = "Diagnostics_autoprofile"
-- NOTE(diagnostics): fed by the stall collector; with collectors.stall = false it never fires

local stalls = {}
local cooldown_until = 0

---@param now number hrtime
local function on_stall(now)
  local window_ns = config.get_value "thresholds.profile_auto_window_s" * 1e9
  local kept = {}
  for _, t in ipairs(stalls) do
    if now - t <= window_ns then kept[#kept + 1] = t end
  end
  kept[#kept + 1] = now
  stalls = kept

  local capture = require "diagnostics.capture"
  if #stalls < config.get_value "thresholds.profile_auto_stalls" then return end
  if now < cooldown_until or capture.running() then return end

  local duration = config.get_value "thresholds.profile_auto_duration_s"
  local ok, err = capture.start {
    backend = "jit",
    duration_s = duration,
    reason = "auto",
  }
  if not ok then
    vim.notify(
      "[diagnostics] auto profile failed: " .. err,
      vim.log.levels.WARN
    )
    return
  end
  -- one capture per window: repeated stalls in a bad session shouldn't keep profiling
  cooldown_until = now + window_ns
  stalls = {}
  vim.notify(
    string.format(
      "[diagnostics] auto profile capture started (%g s) after repeated stalls",
      duration
    ),
    vim.log.levels.INFO
  )
end

---@param ctx {guard: fun(fn: function): function}
function M.start(ctx)
  stalls, cooldown_until = {}, 0
  vim.api.nvim_create_autocmd("User", {
    group = vim.api.nvim_create_augroup(GROUP, { clear = true }),
    pattern = "DiagnosticsStall",
    callback = ctx.guard(function() on_stall(uv.hrtime()) end),
  })
end

function M.stop()
  pcall(vim.api.nvim_del_augroup_by_name, GROUP)
  stalls = {}
end

return M
