-- AstroCore provides a central place to modify vim options, autocommands, and more!
-- Configuration documentation can be found with `:h astrocore`

local ros2_temminal = require "myPlugins.ros2-terminal"
local ros2_nvim = require "myPlugins.ros2-nvim"
local utils = require "myPlugins.utils"

---@type LazySpec
return {
  "AstroNvim/astrocore",
  ---@type AstroCoreOpts
  opts = {
    -- Configure core features of AstroNvim
    features = {
      large_buf = { size = 1024 * 1024, lines = 10000 }, -- set global limits for large files for disabling features like treesitter
      autopairs = true, -- enable autopairs at start
      cmp = true, -- enable completion at start
      diagnostics_mode = 3, -- diagnostic mode on start (0 = off, 1 = no signs/virtual text, 2 = no virtual text, 3 = on)
      highlighturl = true, -- highlight URLs at start
      notifications = true, -- enable notifications at start
    },
    -- Diagnostics configuration (for vim.diagnostics.config({...})) when diagnostics are on
    diagnostics = {
      virtual_text = true,
      underline = true,
    },
    -- vim options can be configured here
    options = {
      opt = { -- vim.opt.<key>
        relativenumber = true, -- sets vim.opt.relativenumber
        number = true, -- sets vim.opt.number
        spell = false, -- sets vim.opt.spell
        signcolumn = "yes", -- sets vim.opt.signcolumn to yes
        wrap = false, -- sets vim.opt.wrap
        scrolloff = 10,
        columns = 80,
        undodir = os.getenv "HOME" .. "/.vim/undodir",
        undofile = true,
        ignorecase = true,
        smartcase = true,
      },
      g = { -- vim.g.<key>
        -- configure global vim variables (vim.g)
        -- NOTE: `mapleader` and `maplocalleader` must be set in the AstroNvim opts or before `lazy.setup`
        -- This can be found in the `lua/lazy_setup.lua` file
      },
    },
    -- Configure plugins
    autocmds = {

      ["urdf"] = {
        {
          desc = "Set urdf filetype for .urdf files",
          event = { "BufRead", "BufNewFile" },
          pattern = "*.urdf",
          callback = function()
            vim.bo.filetype = "urdf"
            -- Enable XML language features
            vim.b.xml_syntax_folding = 1
          end,
        },
      },
    },
    -- NOTE: keycodes follow the casing in the vimdocs. For example, `<Leader>` must be capitalized
    mappings = {
      -- first key is the mode
      n = {
        -- second key is the lefthand side of the map

        -- mappings seen under group name "Buffer"
        ["<C-Q>"] = false,
        ["<C-R>"] = false,
        ["<Leader>C"] = false,
        ["<Leader>q"] = {
          function()
            -- Check if more than one window is open
            if vim.fn.winnr "$" > 1 then
              vim.cmd "confirm q"
            else
              vim.notify(
                "This is the last window, not quitting!",
                vim.log.levels.WARN
              )
            end
          end,
          desc = "Quit Window",
        },
        ["<C-f>"] = {
          "<cmd>silent !tmux neww tmux-sessionizer<CR>",
          desc = "Open tmux-sessionizer",
        },
        ["<M-q>"] = {
          "<Cmd>confirm qall<CR>",
          desc = "Exit AstroNvim",
        },
        ["<M-b>"] = {
          "<cmd>silent !tmux neww tmux-sessionizer -s 1<cr>",
          desc = "Btop",
        },
        ["<M-m>"] = {
          "<cmd>silent !tmux neww tmux-sessionizer -s 0<cr>",
          desc = "Man Pages Fzf",
        },
        ["<C-o>"] = {
          "<cmd>:silent !xdg-open .<cr>",
          desc = "Open File Explorer",
        },
        ["<M-l>"] = {
          "<cmd>:silent !tmux switch-client -l<cr>",
          desc = "Prev Tmux Session",
        },
        ["<M-w>"] = {
          "<cmd>:silent !tmux last-window<cr>",
          desc = "Prev Tmux Window",
        },
        ["<M-o>"] = {
          "<cmd>b#<cr>",
          -- function() require("myPlugins.buffer_history").switch_prev_buffer() end,
          desc = "Switch buffers",
        },
        ["<Tab>"] = {
          ":bnext<CR>",
          desc = "Next buffers",
          silent = true,
        },
        ["<S-Tab>"] = {
          ":bprev<CR>",
          desc = "Prev buffer",
          silent = true,
        },
        ["<Leader>D"] = {
          desc = "Delete file",
          function()
            local file = vim.fn.expand "%:p" -- full path
            if file == "" then
              print "No file to delete"
              return
            end
            local choice =
              vim.fn.confirm("Delete file?\n" .. file, "&Yes\n&No", 2)
            if choice == 1 then
              local file = vim.fn.expand "%"
              vim.cmd "silent! bdelete" -- close buffer first
              os.remove(file) -- delete file using Lua's os.remove
              print("Deleted file: " .. file)
            else
              print "File deletion cancelled"
            end
          end,
        },
        ["<Leader>bd"] = {
          function()
            require("astroui.status.heirline").buffer_picker(
              function(bufnr) require("strocore.buffer").close(bufnr) end
            )
          end,
          desc = "Close buffer from tabline",
        },

        -- Move cursor between visual lines
        ["k"] = {
          "v:count == 0 ? 'gk' : 'k'",
          expr = true,
          silent = true,
        },
        ["j"] = {
          "v:count == 0 ? 'gj' : 'j'",
          expr = true,
          silent = true,
        },

        -- Keeps the cursor in the middle when search next or prev term
        ["n"] = { "nzzzv", silent = true },
        ["N"] = { "Nzzzv", silent = true },

        -- Remap '0' to go to first non-whitespace character
        -- ["0"] = { "^", noremap = true, silent = true },
        -- ["^"] = { "0", noremap = true, silent = true },

        ["<Leader>w"] = {
          function()
            if vim.fn.expand "%" == "" then
              -- Unsaved file, invoke custom save logic
              require("myPlugins.save_new_file").save_file()
            else
              -- Saved file, fallback to default behavior
              vim.cmd "write"
            end
          end,
          desc = "Save File",
        },
        ["<Leader>xq"] = false,
        ["<Leader>xl"] = false,
        ["<C-q>"] = {
          "<cmd>Trouble quickfix toggle<cr>",
          desc = "Quickfix List",
        },
        ["<C-l>"] = {
          "<cmd>Trouble loclist toggle<cr>",
          desc = "Location list",
        },
        ["<Leader>o"] = {
          "<C-w>w",
          desc = "Switch window",
        },
        ["<Leader>r"] = {
          desc = "ROS2 commands",
        },
        ["<Leader>ri"] = {
          desc = "ROS Interfaces",
        },
        ["<Leader>rim"] = {
          desc = "ROS Messages",
          function() ros2_nvim.messages() end,
        },
        ["<Leader>ria"] = {
          desc = "ROS Actions",
          function() ros2_nvim.actions() end,
        },
        ["<Leader>ris"] = {
          desc = "ROS Services",
          function() ros2_nvim.services() end,
        },
        ["<Leader>rs"] = {
          desc = "ROS Active Services",
          function() ros2_nvim.active_services() end,
        },
        ["<Leader>rt"] = {
          desc = "ROS Active Topics",
          function() ros2_nvim.active_topics() end,
        },
        ["<Leader>rn"] = {
          desc = "ROS Active Nodes",
          function() ros2_nvim.active_nodes() end,
        },
        ["<Leader>rp"] = {
          desc = "ROS Params",
          function() ros2_nvim.param() end,
        },
        ["<Leader>re"] = {
          desc = "ROS Execute Command",
        },
        ["<Leader>ren"] = {
          desc = "Execute Node",
          function() ros2_nvim.exec_nodes() end,
        },
        ["<Leader>rel"] = {
          desc = "Execute Launch File",
          function() ros2_nvim.exec_launch_file() end,
        },
        ["<Leader>rc"] = {
          function() ros2_temminal.terminal_picker() end,
          desc = "Toggle ROS Terminal",
        },
      },
      -- All Visual Mode
      x = {
        -- Replace and keep yank
        ["<C-Q>"] = false,
        ["<C-R>"] = false,
        -- Remap '0' to go to first non-whitespace character
        -- ["0"] = { "^", noremap = true, silent = true },
        -- ["^"] = { "0", noremap = true, silent = true },

        ["<Leader>C"] = false,
        ["<Leader>p"] = { '"_dP', desc = "Replace and keep yank" },
      },
      t = {

        ["<Esc>"] = { "<C-\\><C-n>", desc = "Normal Mode Terminal" },
      },
      -- Visual mode mappings
      v = {
        ["<C-Q>"] = false,
        ["<C-R>"] = false,
        ["<Leader>C"] = false,
        -- Move selected lines up or down
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

        -- Search for visually selected text literally
        ["/"] = {
          function() utils.search_selected() end,
          desc = "Search selected text",
        },
      },
    },
  },
}
