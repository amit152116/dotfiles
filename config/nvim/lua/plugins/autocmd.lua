return {
  {
    "AstroNvim/astrocore",
    opts = {
      autocmds = {
        dotfiles_auto = {
          {
            event = "BufWritePost",
            pattern = "*tmux.conf",
            command = "silent !tmux source-file % && tmux display-message 'Tmux Config Reloaded'",
          },
        },
        executable = {
          {
            event = "BufWritePost",
            pattern = { "*" },
            callback = function()
              -- 1. Check for Shebang first (fastest check)
              local first_line = vim.api.nvim_buf_get_lines(0, 0, 1, false)[1]
              if not first_line or not first_line:match "^#!" then return end

              -- 2. Check current file permissions
              local filename = vim.api.nvim_buf_get_name(0)
              local stat = vim.uv.fs_stat(filename)

              -- 3. Only chmod if the owner execute bit (64) is missing
              --    bit.band(stat.mode, 64) == 0 means "not executable by owner"
              if stat and bit.band(stat.mode, 64) == 0 then
                vim.uv.fs_chmod(filename, 493) -- 493 decimal is 755 octal
                vim.notify(
                  "Made script executable: "
                    .. vim.fn.fnamemodify(filename, ":t"),
                  vim.log.levels.INFO
                )
              end
            end,
          },
        },
        -- astrocore's close_with_q checks buftype only; add ft-specific floats it misses
        close_with_q_ft = {
          {
            event = "FileType",
            pattern = {
              "dap-float",
              "gitsigns-blame",
              "grug-far",
              "neotest-output",
              "neotest-output-panel",
              "neotest-summary",
              "dbout",
              "PlenaryTestPopup",
              "startuptime",
              "tsplayground",
            },
            callback = function(args)
              vim.keymap.set("n", "q", "<cmd>close<cr>", { buffer = args.buf, silent = true, nowait = true })
            end,
          },
        },
        resize_splits = {
          {
            event = "VimResized",
            desc = "Equalize splits on terminal resize",
            callback = function()
              local tab = vim.fn.tabpagenr()
              vim.cmd "tabdo wincmd ="
              vim.cmd("tabnext " .. tab)
            end,
          },
        },
        wrap_spell = {
          {
            event = "FileType",
            pattern = { "text", "plaintex", "typst", "gitcommit", "markdown" },
            callback = function()
              vim.opt_local.wrap = true
              vim.opt_local.spell = true
            end,
          },
        },
        json_conceal = {
          {
            event = "FileType",
            pattern = { "json", "jsonc", "json5" },
            callback = function()
              vim.opt_local.conceallevel = 0
            end,
          },
        },
        man_unlisted = {
          {
            event = "FileType",
            pattern = "man",
            callback = function(args)
              vim.bo[args.buf].buflisted = false
            end,
          },
        },
        lsp_folds = {
          {
            event = "LspAttach",
            desc = "Enable LSP fold expression when server supports foldingRange",
            callback = function(args)
              local client = vim.lsp.get_client_by_id(args.data.client_id)
              if not client or not client:supports_method "textDocument/foldingRange" then return end
              if vim.bo[args.buf].buftype ~= "" then return end
              vim.schedule(function()
                if not vim.api.nvim_buf_is_valid(args.buf) then return end
                local win = vim.api.nvim_get_current_win()
                if vim.wo[win].foldmethod ~= "expr" then
                  vim.wo[win].foldmethod = "expr"
                  vim.wo[win].foldexpr = "v:lua.vim.lsp.foldexpr()"
                end
              end)
            end,
          },
        },
      },
    },
  },
}
