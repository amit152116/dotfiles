local active_ai = "copilot"
return {
  {
    "saghen/blink.cmp",
    ---@module 'blink.cmp'
    ---@type blink.cmp.Config

    optional = true,
    dependencies = {
      -- Copilot Source for Blink
      {
        "fang2hou/blink-copilot",
      },
      {
        "zbirenbaum/copilot.lua",
      },
      -- Ripgrep source for Blink
      {
        "mikavilpas/blink-ripgrep.nvim",
        version = "*", -- use the latest stable version
      },
      -- Supermaven source for Blink
      {
        "huijiro/blink-cmp-supermaven",
      },
      {
        "supermaven-inc/supermaven-nvim",
        opts = {
          disable_inline_completion = true, -- disables inline completion for use with cmp
          disable_keymaps = true, -- disables built in keymaps for more manual control
        },
      },
    },
    specs = {
      {
        "AstroNvim/astrocore",
        ---@type AstroCoreOpts
        opts = {
          commands = {
            UseCopilot = {
              function()
                local blink = require "blink.cmp"
                active_ai = "copilot"
                blink.reload()
                vim.notify "Switched to Copilot 🚀"
              end,
            },
            UseSupermaven = {
              function()
                local blink = require "blink.cmp"
                active_ai = "supermaven"
                blink.reload()
                vim.notify "Switched to Supermaven 🚀"
              end,
            },
          },
        },
      },
    },
    ---@module 'blink.cmp'
    ---@type blink.cmp.Config
    opts = {
      sources = {
        default = { "copilot", "ripgrep", "supermaven" },
        providers = {
          ripgrep = {
            module = "blink-ripgrep",
            name = "Ripgrep",
            score_offset = -5,
            async = true,
            ---@module "blink-ripgrep"
            ---@type blink-ripgrep.Options
            opts = {
              prefix_min_len = 5,
              backend = {
                use = "gitgrep-or-ripgrep",
              },
            },
          },
          supermaven = {
            name = "supermaven",
            module = "blink-cmp-supermaven",
            score_offset = 0,
            async = true,
            enabled = function()
              if active_ai == "supermaven" then return true end
              return false
            end,
          },
          copilot = {
            name = "copilot",
            module = "blink-copilot",
            score_offset = 0,
            enabled = function()
              if active_ai == "copilot" then return true end
              return false
            end,
            async = true,
            opts = {
              -- Local options override global ones
              max_completions = 3, -- Override global max_completions
              max_attempts = 2,
            },
          },
        },
      },
      keymap = {
        -- ["<Tab>"] = {
        --   "snippet_forward",
        --   function()
        --     if vim.g.ai_accept then return vim.g.ai_accept() end
        --   end,
        --   "fallback",
        -- },

        -- Or use Alt+Space instead
        -- ["<A-Space>"] = { "show", "show_documentation", "hide_documentation" },
        -- Or use Ctrl+N for a more vim-like approach
        -- ["<C-n>"] = { "show", "show_documentation", "hide_documentation" },
      },
    },
  },
}
