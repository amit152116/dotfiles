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
              ["<Leader>xt"] = {
                "<cmd>Trouble todo toggle<cr>",
                desc = "Todo (Trouble)",
              },
              ["<Leader>xT"] = {
                "<cmd>Trouble todo toggle filter={tag={TODO,FIX,FIXME}}<cr>",
                desc = "Todo/Fix/Fixme (Trouble)",
              },
              -- smart nav: uses trouble if open, else falls back to qf
              ["[q"] = {
                function()
                  local trouble = require "trouble"
                  if trouble.is_open() then
                    trouble.prev { skip_groups = true, jump = true }
                  else
                    pcall(vim.cmd.cprev)
                  end
                end,
                desc = "Previous trouble/quickfix item",
              },
              ["]q"] = {
                function()
                  local trouble = require "trouble"
                  if trouble.is_open() then
                    trouble.next { skip_groups = true, jump = true }
                  else
                    pcall(vim.cmd.cnext)
                  end
                end,
                desc = "Next trouble/quickfix item",
              },
            },
          },
        },
      },
    },
  },
}
