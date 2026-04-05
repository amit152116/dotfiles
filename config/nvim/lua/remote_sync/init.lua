---
--- remote_sync.nvim
--- Secure file synchronization between remote machines via rsync/SSH
---
--- Features:
--- - Per-project TOML configuration (no code execution)
--- - Multi-address support with automatic fallback
--- - SSH ControlMaster for connection reuse
--- - Diff preview before receiving
--- - .gitignore filtering (perfect for ROS2/C++ projects)
--- - Security hardened (path validation, shell escaping)
---
--- Usage:
---   require("remote_sync").setup({
---     sync_on_save = false,
---     show_diff_on_receive = true,
---   })
---
--- Commands:
---   :RemoteSyncConfigure  - Configure sync for current project
---   :RemoteSyncSend       - Send current file to remote
---   :RemoteSyncReceive    - Receive current file from remote
---   :RemoteSyncDiff       - Show diff between local and remote
---   :RemoteSyncStatus     - Show sync status
---

local config = require "remote_sync.config"
local commands = require "remote_sync.commands"
local sync = require "remote_sync.sync"
local project = require "remote_sync.project"

local M = {}

--- Setup remote_sync with user configuration
---@param user_config RemoteSyncConfig|nil
function M.setup(user_config)
  -- Apply user config
  config.setup(user_config)

  -- Setup commands and autocmds
  commands.setup()
end

--- Send current file to remote machine
---@param callback fun(success: boolean)|nil
function M.send(callback) sync.send_file(callback) end

--- Receive current file from remote machine
---@param callback fun(success: boolean)|nil
function M.receive(callback) sync.receive_file(callback) end

--- Show diff between local and remote file
---@param callback fun(confirmed: boolean)|nil
function M.diff(callback) sync.show_diff(callback) end

--- Sync entire directory
---@param direction "send"|"receive"
---@param callback fun(success: boolean)|nil
function M.sync_dir(direction, callback)
  sync.sync_directory(direction, callback)
end

--- Cancel current sync operation
function M.cancel() sync.cancel() end

--- Get current sync status
---@return string status Human-readable status
function M.status()
  local state = sync.get_state()

  if state == sync.State.SYNCING_UP then
    return "Syncing up"
  elseif state == sync.State.SYNCING_DOWN then
    return "Syncing down"
  else
    return "Idle"
  end
end

--- Check if sync is in progress
---@return boolean
function M.is_busy() return sync.is_busy() end

--- Get current project config
---@return RemoteSyncProjectConfig|nil
function M.get_project_config() return project.load() end

--- Check if current project is configured
---@return boolean
function M.is_configured() return project.config_exists() end

return M
