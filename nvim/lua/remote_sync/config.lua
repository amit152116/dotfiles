---
--- Global configuration for remote_sync
--- User-configurable defaults that apply across all projects
---

local M = {}

---@class RemoteSyncConfig
---@field sync_on_save boolean|nil Auto-sync file to remote on save
---@field show_diff_on_receive boolean|nil Show diff preview before receiving
---@field auto_create_dirs boolean|nil Create remote directories if missing
---@field project_config_path string|nil Path to project config file (relative to project root)
---@field ignorefile_paths string[]|nil Default gitignore-style files to use for exclusions
---@field remote_includes string[]|nil Files to include even if matched by ignore patterns
---@field ssh_control_persist string|nil SSH ControlMaster persist time
---@field connect_timeout number|nil SSH connection timeout in seconds
---@field rsync_io_timeout number|nil rsync I/O timeout in seconds (kills rsync if no data flows)
---@field rsync_timeout number|nil Overall rsync timeout in seconds (hard ceiling for the entire operation)
---@field backup_dir string|nil Directory to store local backup files created during receive (default: stdpath("data")/remote_sync_backups)
---@field log_sync_ops boolean|nil Write sync events to a log file (stdpath("data")/remote_sync.log)
---@field log_level "DEBUG"|"INFO"|"WARN"|"ERROR" Minimum level to write: "DEBUG"|"INFO"|"WARN"|"ERROR" (default "INFO")
---@field diff_size_threshold number|nil File size in bytes above which chunked (1 MB) comparison is used instead of full in-memory compare (default: 5 MB)
---@field on_sync_start fun(direction: string, file: string)|nil Callback when sync starts
---@field on_sync_complete fun(direction: string, file: string, success: boolean)|nil Callback when sync completes

---@type RemoteSyncConfig
local defaults = {
  -- Sync behavior
  sync_on_save = false,
  show_diff_on_receive = true,
  auto_create_dirs = true,

  -- Project config
  project_config_path = ".nvim/remote_sync.toml",

  -- Ignore/include patterns (gitignore-style files)
  -- Files matching patterns in these ignore files are excluded from sync
  ignorefile_paths = { ".gitignore" },
  -- Files to sync even if they match ignore patterns (useful for build artifacts)
  remote_includes = {},

  -- SSH settings
  ssh_control_persist = "10m",
  connect_timeout = 5,

  -- Timeout settings
  rsync_io_timeout = 30, -- Kill rsync if no data flows for 30s (handles mid-transfer hangs)
  rsync_timeout = 300, -- Hard ceiling: kill rsync after 5 minutes regardless

  -- Diff comparison settings
  -- Files at or below this size use a fast full-string comparison.
  -- Files above this threshold are compared in ~1 MB chunks to avoid
  -- loading the entire file into memory simultaneously.
  diff_size_threshold = 5 * 1024 * 1024, -- 5 MB

  -- Backup settings
  -- Local directory where backup copies of overwritten files are stored during receive.
  -- A subdirectory structure mirroring the original paths is created inside this dir.
  backup_dir = vim.fn.stdpath "data" .. "/remote_sync_backups",

  -- Logging (disabled by default; set to true to write sync events to stdpath("data")/remote_sync.log)
  log_sync_ops = false,
  -- Minimum log level written to file. Entries below this level are silently dropped.
  -- Set to "DEBUG" to capture everything; "ERROR" for only failures.
  log_level = "INFO",

  -- Callbacks (optional)
  on_start = nil,
  on_sync_complete = nil,
}

---@type RemoteSyncConfig
local config = vim.deepcopy(defaults)

--- Apply user configuration
---@param user_config RemoteSyncConfig|nil
function M.setup(user_config)
  if user_config and type(user_config) == "table" then
    config = vim.tbl_deep_extend("force", defaults, user_config)
  end
end

--- Get current configuration
---@return RemoteSyncConfig
function M.get() return config end

--- Get a specific config value
---@param key string
---@return any
function M.get_value(key) return config[key] end

--- Set a specific config value at runtime
---@param key string
---@param value any
function M.set_value(key, value)
  if defaults[key] ~= nil then config[key] = value end
end

--- Get SSH control socket path
---@return string
function M.get_ssh_control_path()
  return vim.fn.stdpath "cache" .. "/remote-sync-ssh-%r@%h-%p"
end

--- Get hostname history file path
---@return string
function M.get_hostname_history_path()
  return vim.fn.stdpath "data" .. "/remote_sync_hosts.txt"
end

--- Reset config to defaults
function M.reset() config = vim.deepcopy(defaults) end

return M
