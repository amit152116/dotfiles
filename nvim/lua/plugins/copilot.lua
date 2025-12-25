---@type LazySpec
return {

  {
    "supermaven-inc/supermaven-nvim",
    cmd = { "SupermavenUseFree", "SupermavenUsePro" },
    event = "InsertEnter",
    opts = {
      disable_inline_completion = true,
      disable_keymaps = true,
      -- keymaps = {
      --   accept_suggestion = "<Tab>",
      -- },
    },
    specs = {
      {
        "AstroNvim/astrocore",
        opts = {
          options = {
            g = {
              -- set the ai_accept function
              ai_accept = function()
                local suggestion = require "supermaven-nvim.completion_preview"
                if suggestion.has_suggestion() then
                  vim.schedule(function() suggestion.on_accept_suggestion() end)
                  return true
                end
              end,
            },
          },
        },
      },
    },
  },
  {
    "zbirenbaum/copilot.lua",
    cmd = "Copilot",
    build = ":Copilot auth",
    event = "InsertEnter",

    opts = {
      suggestion = {
        enabled = false,
        keymap = {
          accept = false, -- handled by completion engine
        },
      },
      panel = {
        enabled = false,
      },
      filetypes = {
        markdown = false,
        help = false,
      },
    },
    specs = {
      {
        "AstroNvim/astrocore",
        opts = {
          options = {
            g = {
              -- set the ai_accept function
              ai_accept = function()
                if require("copilot.suggestion").is_visible() then
                  require("copilot.suggestion").accept()
                  return true
                end
              end,
            },
          },
          mappings = {
            n = {
              ["<Leader>as"] = {
                "<cmd>Copilot status<CR>",
                desc = "Copilot Status",
              },
            },
          },
        },
      },
    },
  },
}
