---
--- Utility functions for remote_sync
--- Shell escaping, path validation, and common helpers
---

local M = {}
local Snacks = require "snacks"

--- Shell escape a string for safe command execution
--- Uses Lua pattern to wrap in single quotes and escape existing quotes
---@param str string The string to escape
---@return string|nil escaped The escaped string or nil on failure
function M.shell_escape(str)
  if not str then return nil end
  -- Check for null bytes (security)
  if str:find "%z" then return nil end
  -- Replace single quotes with '\'' (end quote, escaped quote, start quote)
  local escaped = str:gsub("'", "'\\''")
  return "'" .. escaped .. "'"
end

--- Validate a remote path for security
--- Blocks shell metacharacters that could enable command injection
--- and path traversal sequences that could escape the base directory
---@param path string The path to validate (just the path portion, not host:path)
---@return boolean valid Whether the path is valid
---@return string|nil error Error message if invalid
function M.validate_path(path)
  if not path or path == "" then return false, "Path is empty" end

  -- Check for null bytes
  if path:find "%z" then return false, "Path contains null bytes" end

  -- Block path traversal
  if path:match "%.%." then
    return false, "Path contains directory traversal (..)"
  end

  -- Block shell expansion characters at start
  if path:match "^~" or path:match "^%$" then
    return false, "Path contains shell expansion characters"
  end

  -- Block dangerous shell metacharacters
  local dangerous_patterns = {
    { pattern = ";", name = "semicolon" },
    { pattern = "|", name = "pipe" },
    { pattern = "`", name = "backtick" },
    { pattern = "%$%(", name = "command substitution" },
    { pattern = "&&", name = "AND operator" },
    { pattern = "||", name = "OR operator" },
    { pattern = "\n", name = "newline" },
    { pattern = "\r", name = "carriage return" },
  }

  for _, check in ipairs(dangerous_patterns) do
    if path:find(check.pattern, 1, check.pattern ~= "%$%(" and true) then
      return false, "Path contains dangerous character: " .. check.name
    end
  end

  return true, nil
end

--- Extract host and file from remote path (host:path format)
---@param remote_path string The full remote path (e.g., "user@host:/path/to/file")
---@return string|nil host The host portion
---@return string|nil file The file path portion
function M.parse_remote_path(remote_path)
  local host = remote_path:match "^([^:]+):"
  local file = remote_path:match ":(.+)$"
  return host, file
end

--- Construct remote path from components
---@param host string Remote host
---@param path string Remote file path
---@return string remote_path The combined host:path
function M.make_remote_path(host, path) return host .. ":" .. path end

--- Check if a path is within a base directory
--- Prevents path traversal attacks
---@param path string The path to check
---@param base_path string The base directory
---@return boolean is_within Whether path is within base_path
function M.is_path_within(path, base_path)
  -- Normalize: ensure base_path ends with /
  if not base_path:match "/$" then base_path = base_path .. "/" end
  return path:sub(1, #base_path) == base_path
end

--- Get relative path from base
---@param full_path string The full path
---@param base_path string The base path to remove
---@return string|nil relative The relative path or nil if not within base
function M.get_relative_path(full_path, base_path)
  -- Normalize: ensure base_path ends with /
  if not base_path:match "/$" then base_path = base_path .. "/" end

  if full_path:sub(1, #base_path) == base_path then
    return full_path:sub(#base_path + 1)
  end
  return nil
end

--- Notify user with consistent formatting
---@param msg string The message
---@param level number vim.log.levels.*
---@param title string|nil Optional title prefix
function M.notify(msg, level, title)
  local prefix = title and (title .. ": ") or "Remote Sync: "
  vim.notify(prefix .. msg, level)
  -- Snacks.notifier.
end

--- Notify info
---@param msg string
function M.debug(msg) M.notify(msg, vim.log.levels.DEBUG) end

--- Notify info
---@param msg string
function M.info(msg) M.notify(msg, vim.log.levels.INFO) end

--- Notify warning
---@param msg string
function M.warn(msg) M.notify(msg, vim.log.levels.WARN) end

--- Notify error
---@param msg string
function M.error(msg) M.notify(msg, vim.log.levels.ERROR) end

--- Get filename from path
---@param path string
---@return string filename
function M.basename(path) return vim.fn.fnamemodify(path, ":t") end

--- Get directory from path
---@param path string
---@return string directory
function M.dirname(path) return vim.fn.fnamemodify(path, ":h") end

--- Check if file exists
---@param path string
---@return boolean
function M.file_exists(path) return vim.fn.filereadable(path) == 1 end

--- Check if directory exists
---@param path string
---@return boolean
function M.dir_exists(path) return vim.fn.isdirectory(path) == 1 end

return M
