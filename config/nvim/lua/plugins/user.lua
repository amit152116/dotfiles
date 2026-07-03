---@type LazySpec
return {
  { "andweeb/presence.nvim", event = "VeryLazy" },

  { "tpope/vim-fugitive" },

  { "max397574/better-escape.nvim" },

  { "wakatime/vim-wakatime", event = "User AstroFile" },

  {
    "ThePrimeagen/refactoring.nvim",
    dependencies = { "lewis6991/async.nvim", "nvim-treesitter/nvim-treesitter" },
    opts = {},
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
      sort = { "group", "alphanum", "mod", "order", "local" },
      show_help = false, -- footer renders as separate float with wrong width (wk bug)
    },
  },

  {
    "folke/twilight.nvim",
    enabled = true,
    keys = {
      {
        "ux",
        function() require("twilight.view").toggle() end,
        mode = "n",
        desc = "Toggle twilight",
      },
    },
    opts = {
      dimming = {
        alpha = 0.25,
        color = { "Normal", "#ffffff" }, -- falls back to this if no fg color is found on the highlight group
        term_bg = "#000000", -- used to compute dimmed text color when guibg=NONE
        inactive = false, -- true dims every other window too, except ones showing the same buffer
      },
      context = 20,
      treesitter = true,
      -- expands the dimmed region to the top-most ancestor node of these types
      expand = {
        "function",
        "method",
      },
      exclude = {},
    },
    specs = {
      {
        "AstroNvim/astrocore",
        opts = {
          mappings = {
            n = {
              ["ux"] = {
                function() require("twilight.view").toggle() end,
                desc = "Toggle twilight",
              },
            },
          },
        },
      },
    },
  },
  {
    "MeanderingProgrammer/render-markdown.nvim",
    ft = { "markdown" },
    dependencies = {
      "nvim-treesitter/nvim-treesitter",
      "nvim-mini/mini.icons",
    },
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
          highlight = true,
          jump = true,
        },
        compiler_flags = "",
        languages = {
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
        enable = true, -- can also be toggled later via :TSContextToggle
        multiwindow = true,
        max_lines = 0, -- <= 0 means no limit
        min_window_height = 0, -- <= 0 means no limit
        line_numbers = true,
        multiline_threshold = 20,
        trim_scope = "outer", -- "inner" or "outer" -- which lines to drop once max_lines is exceeded
        mode = "cursor", -- "cursor" or "topline" -- which line context is computed from
        separator = "─", -- only shows once there are >= 2 lines above cursorline
        zindex = 1,
        on_attach = nil, -- fun(buf): boolean -- return false to skip attaching
      }
    end,
  },
  {
    "folke/noice.nvim",
    event = "VeryLazy",
    opts = {},
    -- lazy-loaded deps need an explicit module="..." entry or noice won't trigger their load
    dependencies = {
      "MunifTanjim/nui.nvim",
    },

    config = function()
      require("noice").setup {
        lsp = {
          -- routes markdown rendering through Treesitter instead of each plugin's own renderer
          override = {
            ["vim.lsp.util.convert_input_to_markdown_lines"] = true,
            ["vim.lsp.util.stylize_markdown"] = true,
            ["cmp.entry.get_documentation"] = true, -- requires hrsh7th/nvim-cmp
          },
          -- lspsaga owns hover/signature UI (K -> Lspsaga hover_doc); both
          -- patch the same vim.lsp.buf functions, leave it to lspsaga
          hover = { enabled = false },
          signature = { enabled = false },
        },
        presets = {
          bottom_search = false, -- classic bottom cmdline for search instead of noice's popup
          command_palette = true, -- cmdline + popupmenu share one position
          long_message_to_split = true,
          inc_rename = false, -- input dialog for inc-rename.nvim
          lsp_doc_border = true,
        },
      }
    end,
  },
  {
    "bloznelis/before.nvim",
    event = { "InsertEnter", "TextChanged" },
    specs = {
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
