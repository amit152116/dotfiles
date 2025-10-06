-- You can also add or configure plugins by creating files in this `plugins/` folder
-- PLEASE REMOVE THE EXAMPLES YOU HAVE NO INTEREST IN BEFORE ENABLING THIS FILE
-- Here are some examples:

---@type LazySpec
return {

  -- == Examples of Adding Plugins ==
  { "andweeb/presence.nvim" },

  { "max397574/better-escape.nvim" },

  {
    "krady21/compiler-explorer.nvim",
  },

  {
    "ray-x/lsp_signature.nvim",
    event = "BufRead",
    enabled = false,
    config = function() require("lsp_signature").setup() end,
  },

  {
    "yutkat/confirm-quit.nvim",
    event = "CmdlineEnter",
    opts = {
      overwrite_q_command = true,
      quit_message = "Do you want to quit?",
    },
  },

  {
    "which-key.nvim",
    opts = {
      preset = "helix",
    },
  },

  { "nvim-treesitter/playground" },
  {
    "nvim-treesitter/nvim-treesitter",
    opts = {
      ensure_installed = {
        "lua",
        "vim",
        "latex",
        -- add more arguments for adding more treesitter parsers
      },
    },
  },

  {
    "MeanderingProgrammer/render-markdown.nvim",
    dependencies = {
      "nvim-treesitter/nvim-treesitter",
      "echasnovski/mini.icons",
    }, -- if you use the mini.nvim suite
    ---@module 'render-markdown'
    ---@type render.md.UserConfig
    config = function()
      require("render-markdown").setup {
        latex = { enabled = false },
        completions = { blink = { enabled = true } },
      }
    end,
  },
}
