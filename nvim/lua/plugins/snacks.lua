local Snacks = require "snacks"
local helper = require "utils.helper"
local myPicker = require "myPlugins"
return {
  "folke/snacks.nvim",
  lazy = false,
  opts = {
    picker = {
      win = {
        input = {
          keys = {
            ["<a-a>"] = {
              "sidekick_send",
              mode = { "n", "i" },
            },
            ["<Esc>"] = { "close", mode = { "n", "i" } },
            -- ["<c-o"] = { "confirm", mode = { "n", "i" } },
            ["<a-s>"] = { "flash", mode = { "n", "i" } },
            ["s"] = { "flash" },
          },
        },
        list = {
          ["<Esc>"] = { "close", mode = { "n", "i" } },
          ["<a-s>"] = { "flash", mode = { "n", "i" } },
          ["s"] = "flash",
          ["<c-u>"] = "preview_scroll_up",
          ["<c-d>"] = "preview_scroll_down",
        },
      },
      actions = {
        sidekick_send = function(...)
          return require("sidekick.cli.picker.snacks").send(...)
        end,
        flash = function(picker)
          require("flash").jump {
            pattern = "^",
            label = { after = { 0, 0 } },
            search = {
              mode = "search",
              exclude = {
                function(win)
                  return vim.bo[vim.api.nvim_win_get_buf(win)].filetype
                    ~= "snacks_picker_list"
                end,
              },
            },
            action = function(match)
              local idx = picker.list:row2idx(match.pos[1])
              picker.list:_move(idx, true, true)
            end,
          }
        end,
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
    scratch = {},
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

            -- ["<Leader>fp"] = {
            --   function()
            --     Snacks.picker.projects {
            --       dev = {
            --         "~/",
            --         "~/Documents/",
            --         "~/Downloads/",
            --         "~/myDisk/Personal Projects/",
            --         "~/.config/",
            --         "~/myDisk/GithubRepositories/",
            --         "~/go/src/github.com/amit152116/",
            --         "~/.local/share/",
            --       },
            --       patterns = {
            --         ".git",
            --         "package.json",
            --         "go.mod",
            --         "README.md",
            --         "README",
            --         "Makefile",
            --       },
            --     }
            --   end,
            --   desc = "Find projects",
            -- },
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

            ["<Leader>fw"] = {
              function() myPicker.grep {} end,
              desc = "Find words",
            },

            ["<Leader>fW"] = {
              function()
                myPicker.grep {
                  hidden = true,
                  ignored = true,
                }
              end,
              desc = "Find words in all files",
            },
            ["<Leader>fz"] = {
              function() Snacks.picker.zoxide() end,
              desc = "Find zoxide",
            },

            ["<Leader>f."] = {
              function() Snacks.picker.resume() end,
              desc = "Resume previous search",
            },

            ["<Leader>f<CR>"] = {
              function()
                Snacks.picker.pickers {
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

            ["<Leader>gb"] = {
              function() Snacks.picker.git_branches { focus = "list" } end,
              desc = "Git branches",
            },

            -- Attach some git mapping when buffer is attached via gitsigns on_attach
            ["<Leader>gc"] = false,
            ["<Leader>gC"] = false,
            ["<Leader>go"] = false,
            ["<Leader>gg"] = false,
            ["<Leader>gl"] = {
              function() Snacks.picker.git_log { focus = "list" } end,
              desc = "Git Logs",
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
                myPicker.grep {
                  cwd = vim.fs.joinpath(vim.fn.stdpath "data", "lazy"),
                  matcher = {
                    frecency = true,
                  },
                }
              end,
              desc = "Plugins Live Grep",
            },
          },
          x = {

            ["<Leader>fw"] = {
              function()
                local selected_text = helper.get_selected_text()
                myPicker.grep {
                  search = selected_text,
                  regex = false,
                }
              end,
              desc = "Find words",
            },

            ["<Leader>fW"] = {
              function()
                local selected_text = helper.get_selected_text()
                myPicker.grep {
                  search = selected_text,
                  hidden = true,
                  ignored = true,
                  regex = false,
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
