-- AstroCore provides a central place to modify mappings, vim options, autocommands, and more!
-- Configuration documentation can be found with `:h astrocore`
-- NOTE: We highly recommend setting up the Lua Language Server (`:LspInstall lua_ls`)
--       as this provides autocomplete and documentation while editing

---@type LazySpec
return {
  "AstroNvim/astrocore",
  ---@type AstroCoreOpts
  opts = {
    -- Configure project root detection, check status with `:AstroRootInfo`
    rooter = {
      autochdir = false,
    },
    -- Configure core features of AstroNvim
    features = {
      large_buf = { size = 1.5 * 1024 * 1024, lines = 10000 }, -- set global limits for large files for disabling features like treesitter
      autopairs = true, -- enable autopairs at start
      cmp = true, -- enable completion at start
      diagnostics = { virtual_text = true, virtual_lines = false }, -- diagnostic settings on startup
      highlighturl = true, -- highlight URLs at start
      notifications = true, -- enable notifications at start
    },
    -- Diagnostics configuration (for vim.diagnostics.config({...})) when diagnostics are on
    diagnostics = {
      virtual_text = true,
      underline = true,
    },
    -- passed to `vim.filetype.add`
    filetypes = {
      -- see `:h vim.filetype.add` for usage
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
    -- vim options can be configured here
    options = {
      opt = { -- vim.opt.<key>
        spell = false, -- sets vim.opt.spell
        wrap = true, -- sets vim.opt.wrap
        scrolloff = 10,
        showmode = true,
        sidescrolloff = 8,
        colorcolumn = "80",
        cursorline = true,
        -- tabstop = 4, -- Number of spaces per tab
        smartindent = true, -- Highlight current line
        showmatch = true, -- Show matching brackets
      },
      g = { -- vim.g.<key>
        -- configure global vim variables (vim.g)
        -- NOTE: `mapleader` and `maplocalleader` must be set in the AstroNvim opts or before `lazy.setup`
        -- This can be found in the `lua/lazy_setup.lua` file
      },
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
    -- NOTE: keycodes follow the casing in the vimdocs. For example, `<Leader>` must be capitalized
    mappings = {
      n = {
        -- Keeps the cursor in the middle when search next or prev term
        ["n"] = { "nzzzv", silent = true },
        ["N"] = { "Nzzzv", silent = true },

        ["<C-f>"] = {
          "<cmd>silent !tmux-sessionizer<CR>",
          desc = "Open tmux-sessionizer",
          silent = true,
        },
        ["<M-q>"] = {
          "<Cmd>confirm qall<CR>",
          desc = "Exit AstroNvim",
          silent = true,
        },
        ["<M-m>"] = {
          "<cmd>silent !tmux-sessionizer -s 0<cr>",
          desc = "Man Pages Fzf",
          silent = true,
        },
        ["<M-b>"] = {
          "<cmd>silent !tmux-sessionizer -s 1<cr>",
          desc = "Btop",
          silent = true,
        },
        ["<M-o>"] = {
          "<cmd>:silent !xdg-open .<cr>",
          desc = "Open File Explorer",
          silent = true,
        },
        ["<M-l>"] = {
          "<cmd>:silent !tmux switch-client -l<cr>",
          desc = "Prev Tmux Session",
          silent = true,
        },
        ["<M-w>"] = {
          "<cmd>:silent !tmux last-window<cr>",
          desc = "Prev Tmux Window",
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
          "<C-w>w",
          desc = "Switch Window",
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
              file = vim.fn.expand "%"
              vim.cmd "silent! bdelete" -- close buffer first
              os.remove(file) -- delete file using Lua's os.remove
              print("Deleted file: " .. file)
            else
              print "File deletion cancelled"
            end
          end,
        },

        -- Save new file/buffer
        ["<Leader>w"] = {
          function()
            if vim.fn.expand "%" == "" then
              -- Unsaved file, invoke custom save logic
              require("utils.save_new_file").save_file()
            else
              -- Saved file, fallback to default behavior
              vim.cmd "write"
            end
          end,
          desc = "Save File",
        },

        -- View/Edit last command
        ["<Leader>;"] = {
          function()
            local last_cmd = vim.fn.histget(":", -1) -- get last Ex command
            if last_cmd ~= "" then
              vim.api.nvim_feedkeys(":" .. last_cmd, "n", false)
            else
              vim.api.nvim_feedkeys(":", "n", false) -- just open empty cmdline if no history
            end
          end,
          desc = "Last command",
        },

        -- mappings seen under group name "Buffer"
        ["<Leader>bd"] = {
          function()
            require("astroui.status.heirline").buffer_picker(
              function(bufnr) require("astrocore.buffer").close(bufnr) end
            )
          end,
          desc = "Close buffer from tabline",
        },

        ["<Leader>bj"] = {
          function() require("utils.buffer_cycle").buffer_cycle "prev" end,
          desc = "Cycle to Previous Buffer",
        },
        ["<Leader>bk"] = {
          function() require("utils.buffer_cycle").buffer_cycle "next" end,
          desc = "Cycle to Next Buffer",
        },

        ["<Leader>bp"] = {
          "<cmd>b#<cr>",
          desc = "Jump to Previous Buffer",
        },

        ["<Leader>uD"] = {
          function() require("astrocore.toggles").diagnostics() end,
          desc = "Toggle diagnostics",
        },
        ["<Leader>r"] = {
          "<cmd>AstroRoot<CR>",
          desc = "AstroRoot",
          silent = true,
        },
        ["<Leader>pr"] = {
          "<cmd>AstroReload<CR>",
          desc = "Astro Reload",
          silent = true,
        },
      },
      -- All Visual Mode
      x = {
        ["<Leader>p"] = { '"_dP', desc = "Replace and keep yank" },
      },
      t = {

        ["<Esc>"] = { "<C-\\><C-n>", desc = "Normal Mode Terminal" },
      },
      v = {
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
          function()
            local selected_text = require("utils.helper").get_selected_text()

            -- Escape for Vim's literal search
            local escaped_text = vim.fn.escape(selected_text, "/\\")
            vim.fn.setreg("/", "\\V" .. escaped_text) -- set literal search
            -- Trigger normal buffer search
            vim.cmd "normal! n"
          end,
          desc = "Search selected text",
        },
      },
    },
  },
}
