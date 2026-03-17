-- Entry point for remote_sync.nvim
-- This file is auto-loaded by Neovim from the plugin/ directory

-- Prevent loading twice
if vim.g.loaded_remote_sync then return end
vim.g.loaded_remote_sync = true

-- Defer loading until VimEnter to not slow down startup
vim.api.nvim_create_autocmd("VimEnter", {
  callback = function()
    -- Only load if not already set up via lazy.nvim or similar
    if not vim.g.remote_sync_setup_done then
      -- Auto-setup with defaults (user can override with setup())
      require("remote_sync").setup()
    end
  end,
  once = true,
})
