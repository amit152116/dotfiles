local helper = require "utils.helper"
local myPicker = require "myPlugins"
local Snacks = require "snacks"
_G.Snacks = require "snacks"

return {
  "folke/snacks.nvim",
  lazy = false,
  ---@type snacks.Config
  opts = {
    animate = {},

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
    indent = {
      enabled = true,
      indent = { char = "▏" },
      scope = { char = "▏" },
      chunk = {
        enabled = true,
        char = {
          corner_top = "╭",
          corner_bottom = "╰",
          horizontal = "─",
          vertical = "│",
          arrow = ">",
        },
      },
      animate = {
        enabled = true,
        style = "out",
      },
    },
    -- explorer = {
    --   trash = true,
    --   replace_netrw = true,
    -- },

    picker = {
      ui_select = true,
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
      win = {
        -- input window
        input = {
          keys = {
            -- to close the picker on ESC instead of going to normal mode,
            -- add the following keymap to your config
            ["<Esc>"] = { "close", mode = { "n", "i" } },
            ["<a-a>"] = {
              "sidekick_send",
              mode = { "n", "i" },
            },
            -- ["<c-o"] = { "confirm", mode = { "n", "i" } },
            ["<a-s>"] = { "flash", mode = { "n", "i" } },
            ["s"] = { "flash" },
            ["/"] = "toggle_focus",
            ["<C-Down>"] = { "history_forward", mode = { "i", "n" } },
            ["<C-Up>"] = { "history_back", mode = { "i", "n" } },
            ["<C-c>"] = { "cancel", mode = "i" },
            ["<C-w>"] = {
              "<c-s-w>",
              mode = { "i" },
              expr = true,
              desc = "delete word",
            },
            ["<CR>"] = { "confirm", mode = { "n", "i" } },
            ["<Down>"] = { "list_down", mode = { "i", "n" } },
            ["<S-CR>"] = { { "pick_win", "jump" }, mode = { "n", "i" } },
            ["<S-Tab>"] = { "select_and_prev", mode = { "i", "n" } },
            ["<Tab>"] = { "select_and_next", mode = { "i", "n" } },
            ["<Up>"] = { "list_up", mode = { "i", "n" } },
            ["<a-d>"] = { "inspect", mode = { "n", "i" } },
            ["<a-f>"] = { "toggle_follow", mode = { "i", "n" } },
            ["<a-h>"] = { "toggle_hidden", mode = { "i", "n" } },
            ["<a-i>"] = { "toggle_ignored", mode = { "i", "n" } },
            ["<a-r>"] = { "toggle_regex", mode = { "i", "n" } },
            ["<a-m>"] = { "toggle_maximize", mode = { "i", "n" } },
            ["<a-p>"] = { "toggle_preview", mode = { "i", "n" } },
            ["<a-w>"] = { "cycle_win", mode = { "i", "n" } },
            ["<c-a>"] = { "select_all", mode = { "n", "i" } },
            ["<c-u>"] = { "preview_scroll_up", mode = { "i", "n" } },
            ["<c-d>"] = { "preview_scroll_down", mode = { "i", "n" } },
            ["<c-b>"] = { "list_scroll_up", mode = { "i", "n" } },
            ["<c-f>"] = { "list_scroll_down", mode = { "i", "n" } },
            ["<c-g>"] = { "toggle_live", mode = { "i", "n" } },
            ["<c-j>"] = { "list_down", mode = { "i", "n" } },
            ["<c-k>"] = { "list_up", mode = { "i", "n" } },
            ["<c-n>"] = { "list_down", mode = { "i", "n" } },
            ["<c-p>"] = { "list_up", mode = { "i", "n" } },
            ["<c-q>"] = { "qflist", mode = { "i", "n" } },
            ["<c-s>"] = { "edit_split", mode = { "i", "n" } },
            ["<c-t>"] = { "tab", mode = { "n", "i" } },
            ["<c-v>"] = { "edit_vsplit", mode = { "i", "n" } },
            ["<c-r>#"] = { "insert_alt", mode = "i" },
            ["<c-r>%"] = { "insert_filename", mode = "i" },
            ["<c-r><c-a>"] = { "insert_cWORD", mode = "i" },
            ["<c-r><c-f>"] = { "insert_file", mode = "i" },
            ["<c-r><c-l>"] = { "insert_line", mode = "i" },
            ["<c-r><c-p>"] = { "insert_file_full", mode = "i" },
            ["<c-r><c-w>"] = { "insert_cword", mode = "i" },
            ["<c-w>H"] = "layout_left",
            ["<c-w>J"] = "layout_bottom",
            ["<c-w>K"] = "layout_top",
            ["<c-w>L"] = "layout_right",
            ["?"] = "toggle_help_input",
            ["G"] = "list_bottom",
            ["gg"] = "list_top",
            ["j"] = "list_down",
            ["k"] = "list_up",
            ["q"] = "cancel",
          },
          b = {
            minipairs_disable = true,
          },
        },
        -- result list window
        list = {
          keys = {

            ["<Esc>"] = { "close", mode = { "n", "i" } },
            ["<a-s>"] = { "flash", mode = { "n", "i" } },
            ["s"] = "flash",
            ["/"] = "toggle_focus",
            ["<2-LeftMouse>"] = "confirm",
            ["<CR>"] = "confirm",
            ["<Down>"] = "list_down",
            ["<S-CR>"] = { { "pick_win", "jump" } },
            ["<S-Tab>"] = { "select_and_prev", mode = { "n", "x" } },
            ["<Tab>"] = { "select_and_next", mode = { "n", "x" } },
            ["<Up>"] = "list_up",
            ["<a-d>"] = "inspect",
            ["<a-f>"] = "toggle_follow",
            ["<a-h>"] = "toggle_hidden",
            ["<a-i>"] = "toggle_ignored",
            ["<a-m>"] = "toggle_maximize",
            ["<a-p>"] = "toggle_preview",
            ["<a-w>"] = "cycle_win",
            ["<c-a>"] = "select_all",
            ["<c-u>"] = { "preview_scroll_up", mode = { "i", "n" } },
            ["<c-d>"] = { "preview_scroll_down", mode = { "i", "n" } },
            ["<c-b>"] = { "list_scroll_up", mode = { "i", "n" } },
            ["<c-f>"] = { "list_scroll_down", mode = { "i", "n" } },
            ["<c-j>"] = "list_down",
            ["<c-k>"] = "list_up",
            ["<c-n>"] = "list_down",
            ["<c-p>"] = "list_up",
            ["<c-q>"] = "qflist",
            ["<c-g>"] = "print_path",
            ["<c-s>"] = "edit_split",
            ["<c-t>"] = "tab",
            ["<c-v>"] = "edit_vsplit",
            ["<c-w>H"] = "layout_left",
            ["<c-w>J"] = "layout_bottom",
            ["<c-w>K"] = "layout_top",
            ["<c-w>L"] = "layout_right",
            ["?"] = "toggle_help_list",
            ["G"] = "list_bottom",
            ["gg"] = "list_top",
            ["i"] = "focus_input",
            ["j"] = "list_down",
            ["k"] = "list_up",
            ["q"] = "cancel",
            ["zb"] = "list_scroll_bottom",
            ["zt"] = "list_scroll_top",
            ["zz"] = "list_scroll_center",
          },
          wo = {
            conceallevel = 2,
            concealcursor = "nvc",
          },
        },
        -- preview window
        preview = {
          keys = {
            ["<Esc>"] = "cancel",
            ["q"] = "cancel",
            ["i"] = "focus_input",
            ["<a-w>"] = "cycle_win",
          },
        },
      },
    },
  },
  specs = {
    {
      "AstroNvim/astrocore",
      ---@type AstroCoreOpts
      opts = {
        mappings = {
          n = {

            -- ["<leader>e"] = {
            --   function() Snacks.picker.explorer() end,
            --   desc = "Toggle Explorer",
            -- },

            ["<Leader>."] = {
              function() Snacks.scratch() end,
              desc = "Toggle Scratch Buffer",
            },

            ["<Leader>ud"] = {
              function() Snacks.notifier.hide() end,
              desc = "Dismiss notifications",
            },

            ["<Leader>;"] = {
              function()
                Snacks.picker.command_history {
                  focus = "list",
                }
              end,
              desc = "Last command",
            },

            ["<Leader>fA"] = {
              function() Snacks.picker.autocmds() end,
              desc = "Find autocmds",
            },

            ["<leader>fe"] = {
              function() Snacks.picker.explorer() end,
              desc = "Find explorer",
            },

            ["<leader>ff"] = {
              function()
                Snacks.picker.files {
                  cmd = "fd",
                }
              end,
              desc = "Find files",
            },

            ["<leader>fh"] = {
              function()
                Snacks.picker.help {
                  layout = "left",
                }
              end,
              desc = "Find help",
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
                require("snacks").picker.todo_comments {}
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

            -- All <Leader>g* mappings are buffer-local via gitsigns on_attach
            ["<Leader>gb"] = false,
            ["<Leader>gc"] = false,
            ["<Leader>gC"] = false,
            ["<Leader>go"] = false,
            ["<Leader>gg"] = false,
            ["<Leader>gl"] = false,
            ["<Leader>gt"] = false,
            ["<Leader>gT"] = false,
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
