---@type LazySpec
return {
  "AstroNvim/astrocore",
  ---@type AstroCoreOpts
  opts = {
    -- check resolved root with `:AstroRootInfo`
    rooter = {
      autochdir = false,
    },
    features = {
      large_buf = { size = 2 * 1024 * 1024, lines = 10000 }, -- set global limits for large files for disabling features like treesitter
      autopairs = true, -- enable autopairs at start
      cmp = true, -- enable completion at start
      diagnostics = { virtual_text = true, virtual_lines = false }, -- diagnostic settings on startup
      highlighturl = true, -- highlight URLs at start
      notifications = true, -- enable notifications at start
    },
    diagnostics = {
      virtual_text = true,
      underline = true,
      update_in_insert = false, -- skip diagnostic redraw while still typing
      severity_sort = true, -- highest-severity sign wins when a line has several
    },
    -- passed straight to `vim.filetype.add`
    filetypes = {
      extension = {
        foo = "fooscript",
        urdf = "xml",
        zsh = "bash",
      },
      filename = {
        [".foorc"] = "fooscript",
      },
      pattern = {
        [".*/etc/foo/.*"] = "fooscript",
      },
    },
    options = {
      opt = { -- vim.opt.<key>
        exrc = true, -- auto-source .nvim.lua at project root
        spell = false,
        wrap = true,
        scrolloff = 10,
        showmode = true,
        sidescrolloff = 8,
        colorcolumn = "80",
        cursorline = true,
        tabstop = 4,
        smartindent = true,
        showmatch = false,
        smoothscroll = true,
        splitkeep = "screen",
        foldlevel = 99,
        foldtext = "",
        inccommand = "nosplit",
        jumpoptions = "view",
      },
      -- mapleader/maplocalleader must be set in lua/lazy_setup.lua, before lazy.setup runs
      g = {},
    },
    autocmds = {
      urdf = {
        desc = "Set urdf filetype for .urdf files",
        event = { "BufRead", "BufNewFile" },
        pattern = "*.urdf",
        callback = function()
          -- Set the filetype for snippets
          vim.bo.filetype = "urdf"
          -- Use XML syntax for highlighting
          vim.bo.syntax = "xml"
          -- Optional: enable XML indentation
          vim.bo.shiftwidth = 4
          vim.bo.tabstop = 4
          vim.bo.expandtab = true
        end,
      },
    },
    commands = {
      BufferPath = {
        function() print(vim.api.nvim_buf_get_name(0)) end,
        desc = "Show current buffer path",
      },
    },
    -- keycodes follow vimdocs casing -- `<Leader>` must stay capitalized
    mappings = {
      n = {
        ["n"] = {
          "'Nn'[v:searchforward].'zzzv'",
          expr = true,
          silent = true,
          desc = "Next result (always forward, center+unfold)",
        },
        ["N"] = {
          "'Nn'[v:searchforward == 0].'zzzv'",
          expr = true,
          silent = true,
          desc = "Prev result (always backward, center+unfold)",
        },

        ["<M-q>"] = {
          "<Cmd>confirm qall<CR>",
          desc = "Exit AstroNvim",
          silent = true,
        },

        ["<Leader>qt"] = {
          "<cmd>tabclose<cr>",
          desc = "Quit Tab",
        },
        ["<Leader>q"] = {
          function() require("confirm-quit").confirm_quit() end,
          desc = "Quit Window",
        },

        ["<Leader>o"] = {
          -- HACK(gitsigns): suppress BufLeave so gitsigns' preview_hunk_inline
          -- (autocloses on BufLeave) survives switching windows; regressed after
          -- v5->v6/nvim 0.11 migration where <C-w>w reliably fires BufLeave now.
          function()
            local ei = vim.o.eventignore
            vim.o.eventignore = "BufLeave"
            vim.cmd "wincmd w"
            vim.o.eventignore = ei
          end,
          desc = "Switch Window",
        },

        ["<Leader>D"] = {
          desc = "Delete file",
          function()
            local file = vim.fn.expand "%:p"
            if file == "" then
              print "No file to delete"
              return
            end
            local choice =
              vim.fn.confirm("Delete file?\n" .. file, "&Yes\n&No", 2)
            if choice == 1 then
              file = vim.fn.expand "%"
              vim.cmd "silent! bdelete" -- buffer must close before the backing file is removed
              os.remove(file)
              print("Deleted file: " .. file)
            else
              print "File deletion cancelled"
            end
          end,
        },

        ["<Leader>w"] = {
          function()
            if vim.fn.expand "%" == "" then
              -- never-saved buffer has no name yet, so :write can't infer a path
              require("myPlugins.save_new_file").save_file()
            else
              vim.cmd "write"
            end
          end,
          desc = "Save File",
        },
        ["<Leader>m"] = {
          function()
            local path = vim.fn.expand "%:p"
            vim.fn.jobstart({
              "tmux",
              "neww",
              "-dt",
              "67",
              "glow -p " .. vim.fn.shellescape(path),
            }, {
              detach = true,
              on_exit = function()
                vim.fn.jobstart(
                  { "tmux", "select-window", "-t", "67" },
                  { detach = true }
                )
              end,
            })
          end,
          desc = "Open file in Glow",
          noremap = true,
        },

        ["<Leader>bd"] = {
          function()
            require("astroui.status.heirline").buffer_picker(
              function(bufnr) require("astrocore.buffer").close(bufnr) end
            )
          end,
          desc = "Close buffer from tabline",
        },

        ["<Leader>bj"] = {
          function() require("myPlugins").buffer_cycle "prev" end,
          desc = "Cycle to Previous Buffer",
        },
        ["<Leader>bk"] = {
          function() require("myPlugins").buffer_cycle "next" end,
          desc = "Cycle to Next Buffer",
        },

        ["<Leader>bp"] = {
          "<cmd>b#<cr>",
          desc = "Jump to Previous Buffer",
        },
        ["<Leader>s"] = { desc = "󰒮 Remote Sync" },
        ["<Leader>ss"] = {
          function() require("remote_sync").send() end,
          desc = "Send file to remote",
        },
        ["<Leader>sr"] = {
          function() require("remote_sync").receive() end,
          desc = "Receive file from remote",
        },
        ["<Leader>sd"] = {
          function() require("remote_sync").diff() end,
          desc = "Diff with remote",
        },
        ["<Leader>sD"] = {
          function() require("remote_sync").sync_dir "send" end,
          desc = "Sync directory to remote",
        },
        ["<Leader>sc"] = {
          "<cmd>RemoteSyncConfigure<cr>",
          desc = "Configure remote",
        },
        ["<Leader>se"] = {
          function()
            local project = require "remote_sync.project"
            if project.config_exists() then
              vim.cmd("edit " .. vim.fn.fnameescape(project.get_config_path()))
            else
              vim.notify(
                "No config file. Run :RemoteSyncConfigure first",
                vim.log.levels.WARN
              )
            end
          end,
          desc = "Edit remote config",
        },
        ["<Leader>si"] = {
          "<cmd>RemoteSyncStatus<cr>",
          desc = "Show sync status",
        },
        ["<Leader>sR"] = {
          function() require("remote_sync").sync_dir "receive" end,
          desc = "Sync directory from remote",
        },
        ["<Leader>sx"] = {
          "<cmd>RemoteSyncCancel<cr>",
          desc = "Cancel sync",
        },
        ["<Leader>st"] = {
          "<cmd>RemoteSyncTestConnection<cr>",
          desc = "Test SSH connection",
        },
        ["<Leader>so"] = {
          "<cmd>RemoteSyncOnSave<cr>",
          desc = "Toggle sync on save",
        },
        ["<Leader>sl"] = {
          function()
            local log_path = require("remote_sync.logger").get_log_path()
            if vim.fn.filereadable(log_path) == 1 then
              vim.cmd("edit " .. vim.fn.fnameescape(log_path))
            else
              vim.notify(
                "No log file found. Enable log_sync_ops in setup().",
                vim.log.levels.WARN
              )
            end
          end,
          desc = "Open sync log",
        },
        ["<Leader>sC"] = {
          "<cmd>RemoteSyncConfig reload<cr>",
          desc = "Reload project config",
        },

        ["<Leader>pR"] = {
          "<cmd>AstroRoot<CR>",
          desc = "Show project root",
          silent = true,
        },
        ["<Leader>pr"] = {
          "<cmd>AstroReload<CR>",
          desc = "Astro Reload",
          silent = true,
        },

        -- move line down/up in normal mode (visual has J/K)
        ["<A-j>"] = { "<cmd>m .+1<cr>==", desc = "Move line down" },
        ["<A-k>"] = { "<cmd>m .-2<cr>==", desc = "Move line up" },

        -- buffer management
        ["<Leader>bi"] = {
          function()
            local visible = {}
            for _, win in ipairs(vim.api.nvim_list_wins()) do
              visible[vim.api.nvim_win_get_buf(win)] = true
            end
            for _, buf in ipairs(vim.api.nvim_list_bufs()) do
              if vim.bo[buf].buflisted and not visible[buf] then
                require("snacks").bufdelete(buf)
              end
            end
          end,
          desc = "Delete invisible buffers",
        },

        ["mm"] = { "%", desc = "Jump to matching bracket" },

        -- which-key window hydra — hold <c-w> loop without modifier
        ["<C-w><space>"] = {
          function() require("which-key").show { keys = "<c-w>", loop = true } end,
          desc = "Window hydra mode",
        },

        -- inspect treesitter tree at cursor
        ["<Leader>uI"] = {
          function()
            vim.treesitter.inspect_tree()
            vim.api.nvim_input "I"
          end,
          desc = "Inspect treesitter tree",
        },
      },
      i = {
        -- ==gi: reindent line, re-enter insert at last cursor pos
        ["<A-j>"] = { "<esc><cmd>m .+1<cr>==gi", desc = "Move line down" },
        ["<A-k>"] = { "<esc><cmd>m .-2<cr>==gi", desc = "Move line up" },
        -- <c-g>u inserts undo checkpoint so u only undoes back to last punctuation
        [","] = { ",<c-g>u", desc = "Undo checkpoint" },
        ["."] = { ".<c-g>u", desc = "Undo checkpoint" },
        [";"] = { ";<c-g>u", desc = "Undo checkpoint" },
        ["("] = { "(<c-g>u", desc = "Undo checkpoint" },
        [")"] = { ")<c-g>u", desc = "Undo checkpoint" },
        ["<CR>"] = { "<CR><c-g>u", desc = "Newline + undo checkpoint" },
      },
      x = {
        -- "_d = delete to black hole (no yank), P = paste; preserves yank register
        ["<Leader>p"] = { '"_dP', desc = "Paste over selection (keep yank)" },
      },
      t = {
        ["jj"] = {
          "<C-\\><C-n>",
          noremap = true,
          silent = true,
          desc = "Normal Mode Terminal",
        },
        ["jk"] = {
          "<C-\\><C-n>",
          noremap = true,
          silent = true,
          desc = "Normal Mode Terminal",
        },
      },
      v = {
        ["J"] = {
          ":m '>+1<CR>gv=gv",
          desc = "Move Current Line Down",
          noremap = true,
          silent = true,
        },
        ["K"] = {
          ":m '<-2<CR>gv=gv",
          desc = "Move Current Line Up",
          noremap = true,
          silent = true,
        },

        ["/"] = {
          function()
            local selected_text = require("utils.helper").get_selected_text()
            -- \V switches the register to literal (non-regex) search
            local escaped_text = vim.fn.escape(selected_text, "/\\")
            vim.fn.setreg("/", "\\V" .. escaped_text)
            vim.cmd "normal! n"
          end,
          desc = "Search selected text",
        },
      },
    },
    treesitter = {
      ensure_installed = {
        "lua",
        "vim",
        "bash",
        "cpp",
        "go",
      },
      auto_install = true, -- fetch parser automatically on opening an unparsed filetype
      textobjects = {
        select = {
          select_textobject = {
            ["af"] = { query = "@function.outer", desc = "around function" },
            ["if"] = { query = "@function.inner", desc = "inside function" },
            ["ac"] = { query = "@class.outer", desc = "around class" },
            ["ic"] = { query = "@class.inner", desc = "inside class" },
            ["aa"] = { query = "@parameter.outer", desc = "around parameter" },
            ["ia"] = { query = "@parameter.inner", desc = "inside parameter" },
            ["ao"] = {
              query = "@conditional.outer",
              desc = "around conditional",
            },
            ["io"] = {
              query = "@conditional.inner",
              desc = "inside conditional",
            },
            ["al"] = { query = "@loop.outer", desc = "around loop" },
            ["il"] = { query = "@loop.inner", desc = "inside loop" },
            ["am"] = { query = "@call.outer", desc = "around call" },
            ["im"] = { query = "@call.inner", desc = "inside call" },
          },
        },
        move = {
          goto_next_start = {
            ["]f"] = { query = "@function.outer", desc = "Next function start" },
            ["]a"] = { query = "@parameter.inner", desc = "Next parameter" },
            ["]o"] = {
              query = "@conditional.outer",
              desc = "Next conditional start",
            },
            ["]l"] = { query = "@loop.outer", desc = "Next loop start" },
          },
          goto_next_end = {
            ["]F"] = { query = "@function.outer", desc = "Next function end" },
            ["]O"] = {
              query = "@conditional.outer",
              desc = "Next conditional end",
            },
            ["]L"] = { query = "@loop.outer", desc = "Next loop end" },
          },
          goto_previous_start = {
            ["[f"] = {
              query = "@function.outer",
              desc = "Previous function start",
            },
            ["[a"] = { query = "@parameter.inner", desc = "Previous parameter" },
            ["[o"] = {
              query = "@conditional.outer",
              desc = "Previous conditional start",
            },
            ["[l"] = { query = "@loop.outer", desc = "Previous loop start" },
          },
          goto_previous_end = {
            ["[F"] = {
              query = "@function.outer",
              desc = "Previous function end",
            },
            ["[O"] = {
              query = "@conditional.outer",
              desc = "Previous conditional end",
            },
            ["[L"] = { query = "@loop.outer", desc = "Previous loop end" },
          },
        },
        swap = {
          swap_next = {
            [">F"] = { query = "@function.outer", desc = "Swap next function" },
            [">a"] = {
              query = "@parameter.inner",
              desc = "Swap next parameter",
            },
          },
          swap_previous = {
            ["<F"] = {
              query = "@function.outer",
              desc = "Swap previous function",
            },
            ["<a"] = {
              query = "@parameter.inner",
              desc = "Swap previous parameter",
            },
          },
        },
      },
    },
  },
}
