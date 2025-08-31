return {
  {
    "folke/trouble.nvim",
    enabled = false,
    cmd = "Trouble",
    specs = {
      {
        "folke/snacks.nvim",
        opts = function(_, opts)
          return vim.tbl_deep_extend("force", opts or {}, {
            picker = {
              actions = require("trouble.sources.snacks").actions,
              win = {
                input = {
                  keys = {
                    ["<c-t>"] = {
                      "trouble_open",
                      mode = { "n", "i" },
                    },
                  },
                },
              },
            },
          })
        end,
      },
      {
        "AstroNvim/astrocore",
        opts = {
          autocmds = {
            trouble = {
              {
                event = "FileType",
                pattern = "qf",
                callback = function()
                  vim.schedule(function()
                    vim.cmd "cclose" -- close quickfix window immediately
                  end)
                end,
              },
              {
                event = "QuickFixCmdPost",
                callback = function()
                  local trouble = require "trouble"
                  if trouble.is_open() and trouble.get_mode() == "quickfix" then
                    trouble.refresh()
                  else
                    trouble.open "quickfix"
                  end
                end,
              },
            },
          },
          mappings = {
            n = {
              ["<Leader>xq"] = {
                "<cmd>Trouble qflist toggle<CR>",
                desc = "Quickfix List",
              },
              ["<Leader>xl"] = {
                "<Cmd>Trouble loclist toggle<CR>",
                desc = "Location List",
              },

              ["<Leader>xx"] = {
                "<cmd>Trouble<cr>",
                desc = "Toggle Trouble",
              },
              ["<Leader>ls"] = {
                "<cmd>Trouble symbols toggle<cr>",
                desc = "Symbols Outline",
              },
            },
          },
        },
      },
    },
  },
}
