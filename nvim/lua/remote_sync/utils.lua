---
--- Utility functions for remote_sync
--- Shell escaping, path validation, and common helpers
---

local M = {}

-- Chunk size used by the large-file comparison path.
local LARGE_FILE_CHUNK_SIZE = 1024 * 1024 -- 1 MB

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
---@return string host The host portion
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

--- Fast, full-memory comparison for files at or below the size threshold.
--- Reads the local file as a single raw binary string and compares it with
--- the remote content string.  Handles the trailing-newline edge-case that
--- arises because the SSH capture layer strips the final newline from stdout.
---@param local_path string
---@param remote_content string Remote file content as returned by get_remote_file
---@return boolean
function M.compare_files_small(local_path, remote_content)
  local local_content = ""
  local f = io.open(local_path, "rb")
  if f then
    local_content = f:read "*a"
    f:close()
  end

  -- Fast path: exact match.
  if local_content == remote_content then return true end

  -- SSH capture strips the trailing newline from the remote content.
  -- Treat a single trailing-newline difference as identical.
  local c_len, l_len = #remote_content, #local_content
  if c_len == l_len + 1 and remote_content:byte(-1) == 10 then
    return remote_content:sub(1, -2) == local_content
  elseif l_len == c_len + 1 and local_content:byte(-1) == 10 then
    return local_content:sub(1, -2) == remote_content
  end

  return false
end

--- Memory-efficient comparison for files above the size threshold.
--- Reads the local file in ~1 MB chunks and compares each chunk against the
--- corresponding segment of the (already-fetched) remote content string.
--- Peak memory usage: remote_content + one chunk buffer (~1 MB).
---
--- The same trailing-newline normalisation as compare_files_small is applied:
--- remote_content has its trailing newline stripped upfront, and the local
--- file's final chunk has its trailing newline stripped when local_size is
--- exactly one byte larger than the normalised remote length.
---@param local_path string
---@param remote_content string Remote file content as returned by get_remote_file
---@param local_size number Actual local file size in bytes (from vim.loop.fs_stat)
---@return boolean
function M.compare_files_large(local_path, remote_content, local_size)
  -- Normalise remote: strip trailing newline stripped by SSH capture.
  local norm_remote = remote_content
  if #remote_content > 0 and remote_content:byte(-1) == 10 then
    norm_remote = remote_content:sub(1, -2)
  end
  local remote_len = #norm_remote

  -- If local has exactly one more byte than normalised remote, that extra byte
  -- must be a trailing newline for the files to be equal.
  local strip_local_trailing = (local_size == remote_len + 1)

  -- Any other size mismatch means the files are definitely different.
  -- (Defensive: the pre-check in show_diff should have caught this already.)
  if local_size ~= remote_len and not strip_local_trailing then return false end

  local f = io.open(local_path, "rb")
  if not f then return false end

  local offset = 1 -- 1-based index into norm_remote
  local bytes_read = 0

  while true do
    local chunk = f:read(LARGE_FILE_CHUNK_SIZE)
    if not chunk then break end

    bytes_read = bytes_read + #chunk

    -- On the final chunk, strip the trailing newline when expected.
    local effective_chunk = chunk
    if strip_local_trailing and bytes_read == local_size then
      if chunk:byte(-1) == 10 then
        effective_chunk = chunk:sub(1, -2)
      else
        -- Expected a trailing newline but found a different byte; they differ.
        f:close()
        return false
      end
    end

    local eff_len = #effective_chunk
    if eff_len == 0 then break end -- last chunk was a bare newline, already consumed

    local remote_seg = norm_remote:sub(offset, offset + eff_len - 1)
    if effective_chunk ~= remote_seg then
      f:close()
      return false
    end

    offset = offset + eff_len
  end

  f:close()

  -- All local bytes matched; verify we also consumed all of norm_remote.
  return (offset - 1) == remote_len
end

function M.cleanup_diff_mode(local_win)
  if vim.api.nvim_win_is_valid(local_win) then
    vim.api.nvim_set_current_win(local_win)
    vim.cmd "diffoff"
  end
end

function M.prompt_receive(callback)
  vim.ui.select(
    { "Yes, receive remote version", "No, keep local version" },
    { prompt = "Receive file from remote?" },
    function(choice)
      local confirmed = choice and choice:match "^Yes"
      if not confirmed then M.info "Receive cancelled" end
      if callback then callback(confirmed) end
    end
  )
end
return M
