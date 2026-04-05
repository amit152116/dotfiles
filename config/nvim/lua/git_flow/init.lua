---
--- git-flow.nvim - init.lua
--- Public API entry point. Exposes setup() and re-exports module APIs.
---

local M = {}

--- Configure and initialise git-flow.nvim.
---@param user_config GitFlowConfig|nil Optional overrides merged onto defaults
function M.setup(user_config)
  -- 1. Merge user config into defaults
  require("git_flow.config").setup(user_config)

  -- 2. Register all user commands and autocmds
  require("git_flow.commands").register()

  -- Mark as initialised so the auto-load entry point skips a second setup
  vim.g.git_flow_setup_done = true
end

-- Re-export module APIs for direct use (e.g. from gitsigns on_attach)
M.backup = require "git_flow.backup"
M.worktree = require "git_flow.worktree"
M.common = require "git_flow.common"
M.config = require "git_flow.config"

return M
