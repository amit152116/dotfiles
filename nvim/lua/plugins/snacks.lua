local Snacks = require "snacks"
local helper = require "utils.helper"
return {
  "folke/snacks.nvim",
  lazy = false,
  opts = {
    picker = {
      win = {
        input = {
          keys = {
            ["<Esc>"] = { "close", mode = { "n", "i" } },
            -- ["<c-o"] = { "confirm", mode = { "n", "i" } },
          },
        },
      },
      layout = {},
    },
    dashboard = {
      preset = {
        header = table.concat({
          [[                                                                       ]],
          [[                                                                     ]],
          [[       ████ ██████           █████      ██                     ]],
          [[      ███████████             █████                             ]],
          [[      █████████ ███████████████████ ███   ███████████   ]],
          [[     █████████  ███    █████████████ █████ ██████████████   ]],
          [[    █████████ ██████████ █████████ █████ █████ ████ █████   ]],
          [[  ███████████ ███    ███ █████████ █████ █████ ████ █████  ]],
          [[ ██████  █████████████████████ ████ █████ █████ ████ ██████ ]],
          [[                                                                       ]],
        }, "\n"),
      },
    },
    lazygit = {},
    scratch = {
      ft = "markdown",
    },
  },
  specs = {
    {
      "AstroNvim/astrocore",
      ---@type AstroCoreOpts
      opts = {
        mappings = {
          n = {

            ["<Leader>."] = {
              function() Snacks.scratch() end,
              desc = "Toggle Scratch Buffer",
            },

            ["<Leader>s"] = {
              function() Snacks.scratch.select() end,
              desc = "Select Scratch Buffer",
            },

            ["<Leader>ud"] = {
              function() Snacks.notifier.hide() end,
              desc = "Dismiss notifications",
            },

            ["<Leader>fi"] = {
              function() Snacks.picker.icons() end,
              desc = "Find icons",
            },

            ["<Leader>fj"] = {
              function() Snacks.picker.jumps() end,
              desc = "Find jumps",
            },

            ["<Leader>fT"] = {
              function() Snacks.picker.colorschemes() end,
              desc = "Find themes",
            },

            ["<Leader>ft"] = {
              function()
                if not package.loaded["todo-comments"] then -- make sure to load todo-comments
                  require("lazy").load { plugins = { "todo-comments.nvim" } }
                end
                Snacks.picker.todo_comments()
              end,
              desc = "Todo Comments",
            },

            ["<Leader>fz"] = {
              function() Snacks.picker.zoxide() end,
              desc = "Find zoxide",
            },

            ["<Leader>gl"] = {
              function() Snacks.picker.git_log() end,
              desc = "Find themes",
            },
            -- Find all Neovim plugins files
            ["<Leader>pf"] = {
              function()
                Snacks.picker.files {
                  cwd = vim.fs.joinpath(vim.fn.stdpath "data", "lazy"),
                  matcher = {
                    frecency = true,
                  },
                }
              end,
              desc = "Plugins files",
            },

            -- Find words in Neovim plugins files
            ["<Leader>pw"] = {
              function()
                Snacks.picker.grep {
                  cwd = vim.fs.joinpath(vim.fn.stdpath "data", "lazy"),
                  matcher = {
                    frecency = true,
                  },
                }
              end,
              desc = "Plugins Live Grep",
            },

            ["<Leader>f."] = {
              function()
                Snacks.picker.pickers {
                  -- layout = { preset = "vscode" },
                  focus = "input",
                  confirm = function(picker, item)
                    if not item then return end
                    picker:close()
                    Snacks.picker.pick {
                      source = item.text,
                      focus = "input",
                    }
                  end,
                }
              end,
              desc = "Find pickers",
            },
          },
          x = {

            ["<Leader>fw"] = {
              function()
                local selected_text = helper.get_selected_text()
                require("utils.multigrep").live_multigrep {
                  search = selected_text,
                  regex = false,
                  additional_args = { "--fixed-strings" },
                }
                -- Snacks.picker.grep_word {}
              end,
              desc = "Find words",
            },

            ["<Leader>fW"] = {
              function()
                local selected_text = helper.get_selected_text()
                Snacks.picker.grep_word {
                  search = selected_text,
                  hidden = true,
                  ignored = true,
                  regex = false,
                  args = {
                    "--fixed-strings",
                  },
                }
              end,
              desc = "Find words in all files",
            },
          },
        },
      },
    },
  },
}
