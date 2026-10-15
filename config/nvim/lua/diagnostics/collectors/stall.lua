local config = require "diagnostics.config"
local writer = require "diagnostics.writer"
local uv = vim.uv

local M = {}

local STACKS_PER_STALL = 3

local timer

---@param ctx {guard: fun(fn: function): function}
function M.start(ctx)
  if timer then return end
  local interval = config.get_value "thresholds.heartbeat_ms"
  local stall_ms = config.get_value "thresholds.stall_ms"
  local last = uv.hrtime()
  local deadline = last + interval * 1e6

  -- TODO(diagnostics): freeze ended by killing the pane is never logged; add external watchdog process
  timer = uv.new_timer()
  timer:start(
    interval,
    interval,
    ctx.guard(function()
      -- fast-event context: timing only, editor state is read in vim.schedule
      local now = uv.hrtime()
      local lateness_ms = (now - deadline) / 1e6
      local elapsed_ms = (now - last) / 1e6
      local window_start = last
      last, deadline = now, now + interval * 1e6
      if lateness_ms <= stall_ms then return end
      local recent = writer.recent()
      local profile = package.loaded["diagnostics.collectors.profile"]
      local stacks = profile
        and profile.running()
        and profile.top(window_start, now, STACKS_PER_STALL)
      if stacks and #stacks == 0 then stacks = nil end
      vim.schedule(ctx.guard(function()
        local buf = vim.api.nvim_get_current_buf()
        writer.emit("stall", lateness_ms, {
          elapsed_ms = math.floor(elapsed_ms),
          buf = vim.api.nvim_buf_get_name(buf),
          ft = vim.bo[buf].filetype,
          mode = vim.api.nvim_get_mode().mode,
          recent = recent,
          stacks = stacks or nil,
          -- lets the report tell "not CPU-bound" apart from "profiler off"
          profiled = profile and profile.running() or nil,
        })
        -- autoprofile (and user config) can react without polling the log
        vim.api.nvim_exec_autocmds("User", {
          pattern = "DiagnosticsStall",
          modeline = false,
          data = { ms = lateness_ms },
        })
      end))
    end)
  )
end

function M.stop()
  if not timer then return end
  timer:stop()
  timer:close()
  timer = nil
end

return M
