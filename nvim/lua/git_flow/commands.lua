---
--- git-flow.nvim - commands.lua
--- Registers all user-facing Ex commands and autocmds for backup + worktree.
--- Called once by init.lua during setup.
---

local M = {}

--- Register all git-flow user commands and autocmds.
--- Called by git_flow.init.setup() after config is merged.
function M.register()
  local config = require "git_flow.config"
  local backup = require "git_flow.backup"
  local worktree = require "git_flow.worktree"
  local common = require "git_flow.common"

  -- Ensure backup directory exists
  common.ensure_dir(config.options.backup.backup_dir)

  -- -------------------------------------------------------------------------
  -- Backup commands
  -- -------------------------------------------------------------------------

  -- Manually trigger a backup right now
  vim.api.nvim_create_user_command(
    "GitBackupNow",
    function() backup.create_backup() end,
    { desc = "git-flow: create a backup patch now" }
  )

  -- Browse and optionally apply backup patches for the current repo
  vim.api.nvim_create_user_command(
    "GitBackupShow",
    function() backup.show_backups() end,
    { desc = "git-flow: browse backup patches" }
  )

  -- Start the periodic auto-backup timer
  vim.api.nvim_create_user_command(
    "GitBackupStart",
    function() backup.start_auto_backup() end,
    { desc = "git-flow: start auto-backup timer" }
  )

  -- Stop the periodic auto-backup timer
  vim.api.nvim_create_user_command(
    "GitBackupStop",
    function() backup.stop_auto_backup() end,
    { desc = "git-flow: stop auto-backup timer" }
  )

  -- -------------------------------------------------------------------------
  -- Worktree commands
  -- -------------------------------------------------------------------------

  if config.options.worktree.enabled then
    -- Open the switch/create worktree picker
    vim.api.nvim_create_user_command(
      "GitWorktree",
      function() worktree.switch_worktree() end,
      { desc = "git-flow: open worktree switch/create picker" }
    )
  end

  -- -------------------------------------------------------------------------
  -- Auto-backup autocmds
  -- -------------------------------------------------------------------------

  if config.options.backup.enabled and config.options.backup.auto_backup then
    -- Start timer when Neovim is fully loaded
    vim.api.nvim_create_autocmd("VimEnter", {
      group = vim.api.nvim_create_augroup(
        "GitFlowAutoBackup",
        { clear = true }
      ),
      callback = function()
        -- Only activate inside a git repo
        if common.get_git_root() then backup.start_auto_backup() end
      end,
      once = true,
      desc = "git-flow: start auto-backup timer on VimEnter",
    })

    -- Persist tree-SHA manifest on exit so deduplication survives restarts
    vim.api.nvim_create_autocmd("VimLeavePre", {
      group = vim.api.nvim_create_augroup(
        "GitFlowManifestSave",
        { clear = true }
      ),
      callback = function() backup.save_manifest() end,
      desc = "git-flow: persist backup manifest on exit",
    })
  end
end

return M
