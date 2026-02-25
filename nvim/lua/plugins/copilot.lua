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
        ---@type AstroCoreOpts
        opts = {
          autocmds = {
            -- Add a atuocmd when AstroLargeBuf is triggered it should  stop the supermaven for that
            ai_large_buf = {
              {
                event = "User",
                pattern = "AstroLargeBuf",
                callback = "<cmd>SupermavenStop<CR>",
              },
            },
          },
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
    config = function()
      require("copilot").setup {
        copilot_node_command = "/usr/bin/node",
      }
    end,
    dependencies = {
      "copilotlsp-nvim/copilot-lsp", -- (optional) for NES functionality
      init = function() vim.g.copilot_nes_debounce = 200 end,
    },
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
      nes = {
        enabled = true,
        keymap = {
          accept_and_goto = "<Tab>",
          accept = false,
          dismiss = "<Esc>",
        },
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
        },
      },
    },
  },
}
