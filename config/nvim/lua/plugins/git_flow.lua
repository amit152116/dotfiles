-- git-flow.nvim plugin spec for lazy.nvim
-- Consolidates git backup and worktree management into a single local plugin.

---@type LazySpec
return {
  {
    dir = vim.fn.stdpath "config",
    name = "git_flow",
    lazy = false,
    dependencies = {
      "nvim-lua/plenary.nvim",
      "folke/snacks.nvim",
    },
    config = function()
      require("git_flow").setup {
        backup = {
          max_backups = 20,
        },
      }
    end,
  },
}
