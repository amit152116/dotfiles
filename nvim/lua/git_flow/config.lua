---
--- git-flow.nvim - config.lua
--- Centralised configuration with defaults
---

local M = {}

---@class GitFlowBackupConfig
---@field enabled boolean|nil Auto-create backups on a timer
---@field auto_backup boolean|nil Start the timer on VimEnter when inside a git repo
---@field backup_interval number|nil Milliseconds between auto-backups (default 1 hour)
---@field max_backups number|nil Max patch files to keep per repo before rotating
---@field backup_dir string|nil Directory where patch files are stored

---@class GitFlowWorktreeConfig
---@field enabled boolean Register worktree picker in gitsigns on_attach

---@class GitFlowConfig
---@field backup GitFlowBackupConfig|nil
---@field worktree GitFlowWorktreeConfig|nil

M.defaults = {
  backup = {
    enabled = true,
    auto_backup = true,
    backup_interval = 60 * 60 * 1000, -- 1 hour
    max_backups = 20,
    backup_dir = vim.fn.stdpath "data" .. "/git-backups",
  },
  worktree = {
    enabled = true,
  },
}

---@type GitFlowConfig
M.options = vim.deepcopy(M.defaults)

---@param user_config GitFlowConfig|nil
function M.setup(user_config)
  M.options = vim.tbl_deep_extend("force", M.defaults, user_config or {})
end

return M
