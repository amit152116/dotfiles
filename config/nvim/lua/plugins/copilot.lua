---@type LazySpec
return {

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
        enabled = false,
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
