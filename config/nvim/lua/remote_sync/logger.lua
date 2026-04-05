---
--- Optional file logger for remote_sync
--- Records sync events to a persistent log file for post-hoc debugging.
--- Disabled by default; enable with: require("remote_sync").setup({ log_sync_ops = true })
---

local M = {}

-- 0644 octal = 420 decimal: owner read/write, group and others read-only
local FILE_MODE = 420
local MAX_LOG_SIZE = 10 * 1024 * 1024 -- 10MB before rotation

-- Numeric rank for each level (higher = more severe)
local LEVEL_RANK = { DEBUG = 1, INFO = 2, WARN = 3, ERROR = 4 }

--- Get the log file path
---@return string
function M.get_log_path() return vim.fn.stdpath "cache" .. "/remote_sync.log" end

--- Write a log entry asynchronously.
--- No-ops if log_sync_ops is not enabled in config.
---@param level string "DEBUG"|"INFO"|"WARN"|"ERROR"
---@param message string Log message
---@param context table|nil Optional structured context (vim.inspect'd)
function M.log(level, message, context)
  -- Lazy-load config to avoid circular require at module load time
  local cfg = require "remote_sync.config"
  if not cfg.get_value "log_sync_ops" then return end

  -- Drop entries below the configured minimum level
  local min_level = cfg.get_value "log_level" or "INFO"
  if (LEVEL_RANK[level] or 0) < (LEVEL_RANK[min_level] or 0) then return end

  local log_file = M.get_log_path()
  local timestamp = os.date "%Y-%m-%d %H:%M:%S"
  local ctx_str = context and (" " .. vim.inspect(context)) or ""
  local entry =
    string.format("[%s] [%s] %s%s\n", timestamp, level, message, ctx_str)

  -- Open for append and write entry (called after optional rotation)
  local function write_entry()
    vim.loop.fs_open(log_file, "a", FILE_MODE, function(err, fd)
      if not err and fd then
        vim.loop.fs_write(fd, entry, -1, function()
          vim.loop.fs_close(fd, function() end)
        end)
      end
    end)
  end

  -- Check size synchronously (stat is microseconds; doing it async would
  -- require extra nesting with no practical benefit for this use case)
  local stat = vim.loop.fs_stat(log_file)
  if stat and stat.size > MAX_LOG_SIZE then
    -- Rotate asynchronously, then write to the fresh file
    vim.loop.fs_rename(
      log_file,
      log_file .. ".old",
      function() write_entry() end
    )
  else
    write_entry()
  end
end

function M.debug(msg, ctx) M.log("DEBUG", msg, ctx) end
function M.info(msg, ctx) M.log("INFO", msg, ctx) end
function M.warn(msg, ctx) M.log("WARN", msg, ctx) end
function M.error(msg, ctx) M.log("ERROR", msg, ctx) end

return M
