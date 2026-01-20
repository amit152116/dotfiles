-- You can also add or configure plugins by creating files in this `plugins/` folder
-- PLEASE REMOVE THE EXAMPLES YOU HAVE NO INTEREST IN BEFORE ENABLING THIS FILE
-- Here are some examples:

---@type LazySpec
return {

  -- == Examples of Adding Plugins ==
  { "andweeb/presence.nvim" },
  { "tpope/vim-fugitive" },

  { "max397574/better-escape.nvim" },

  { "wakatime/vim-wakatime", event = "User AstroFile" },

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
      sort = { "group", "alphanum", "mod", "order", "local" },
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
        "bash",
        "cpp",
        -- add more arguments for adding more treesitter parsers
      },

      incremental_selection = {
        enable = true,
        keymaps = {
          init_selection = false, -- set to `false` to disable one of the mappings
          node_incremental = false,
          scope_incremental = false,
          node_decremental = false,
        },
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
  {
    "krady21/compiler-explorer.nvim",
    enabled = false,
    config = function()
      require("compiler-explorer").setup {
        line_match = {
          highlight = true, -- highlight the matching line(s) in the other buffer.
          jump = true, -- move the cursor in the other buffer to the first matching line.
        },
        compiler_flags = "", -- Default flags passed to the compiler.
        languages = { -- Language specific default compiler/flags
          cpp = {
            compiler = "cg114",
            compiler_flags = "-O2 -Wall",
          },
        },
      }
    end,
  },
  {
    "nvim-treesitter/nvim-treesitter-context",
    dependencies = {
      "nvim-treesitter/nvim-treesitter",
    },
    config = function()
      require("treesitter-context").setup {
        enable = true, -- Enable this plugin (Can be enabled/disabled later via commands)
        multiwindow = true, -- Enable multiwindow support.
        max_lines = 0, -- How many lines the window should span. Values <= 0 mean no limit.
        min_window_height = 0, -- Minimum editor window height to enable context. Values <= 0 mean no limit.
        line_numbers = true,
        multiline_threshold = 20, -- Maximum number of lines to show for a single context
        trim_scope = "outer", -- Which context lines to discard if `max_lines` is exceeded. Choices: 'inner', 'outer'
        mode = "cursor", -- Line used to calculate context. Choices: 'cursor', 'topline'
        -- Separator between context and content. Should be a single character string, like '-'.
        -- When separator is set, the context will only show up when there are at least 2 lines above cursorline.
        separator = "─",
        zindex = 1, -- The Z-index of the context window
        on_attach = nil, -- (fun(buf: integer): boolean) return false to disable attaching
      }
    end,
  },
  {
    "bloznelis/before.nvim",
    event = { "InsertEnter", "TextChanged" },
    dependencies = {
      {
        "AstroNvim/astrocore",
        opts = {
          mappings = {
            n = {
              ["]E"] = {
                function() require("before").jump_to_next_edit() end,
                desc = "Next edit",
              },
              ["[E"] = {
                function() require("before").jump_to_last_edit() end,
                desc = "Previous edit",
              },
            },
          },
        },
      },
    },
    opts = {},
  },
}
