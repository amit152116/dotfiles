-- Auto-load entry point for git-flow.nvim
-- Loaded by Neovim from the plugin/ directory on startup.

-- Prevent loading twice
if vim.g.loaded_git_flow then return end
vim.g.loaded_git_flow = true

-- Defer loading until VimEnter to not slow down startup.
-- The lazy.nvim spec (lua/plugins/git_flow.lua) calls setup() with user
-- options before this fires, setting vim.g.git_flow_setup_done = true,
-- so this block is only reached when lazy.nvim is NOT managing the plugin.
vim.api.nvim_create_autocmd("VimEnter", {
  callback = function()
    if not vim.g.git_flow_setup_done then require("git_flow").setup() end
  end,
  once = true,
})
