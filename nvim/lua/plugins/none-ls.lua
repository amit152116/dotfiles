-- Customize None-ls sources

---@type LazySpec
return {
    "nvimtools/none-ls.nvim",
    opts = function(_, opts)
        -- opts variable is the default configuration table for the setup function call
        local null_ls = require "null-ls"

        -- Check supported formatters and linters
        -- https://github.com/nvimtools/none-ls.nvim/tree/main/lua/null-ls/builtins/formatting
        -- https://github.com/nvimtools/none-ls.nvim/tree/main/lua/null-ls/builtins/diagnostics

        -- Only insert new sources, do not replace the existing ones
        -- (If you wish to replace, use `opts.sources = {}` instead of the `list_insert_unique` function)

        opts.sources = require("astrocore").list_insert_unique(opts.sources, {
            -- Set a formatter
            -- null_ls.builtins.formatting.stylua,
            -- null_ls.builtins.formatting.prettier,

            null_ls.builtins.formatting.prettier.with {
                extra_args = { "--prose-wrap", "always", "--print-width", "80" },
                filetypes = { "markdown", "md", "urdf" },
            },
            -- null_ls.builtins.diagnostics.cppcheck.with {
            --
            --     extra_args = function(params)
            --         local args = { "--enable=all", "--suppress=missingIncludeSystem" }
            --
            --         -- Locate compile_commands.json
            --         local project_root = vim.fn.findfile("compile_commands.json", ".;")
            --         if project_root ~= "" then table.insert(args, "--project=" .. project_root) end
            --
            --         -- Restrict analysis to the current file
            --         table.insert(args, "--file-filter=" .. params.bufname)
            --
            --         return args
            --     end,
            --     filetypes = { "c", "cpp", "h", "hpp" },
            -- },
            null_ls.builtins.formatting.clang_format.with {
                extra_args = { "--style=file" },
                filetypes = { "c", "cpp", "h", "hpp" },
            },
        })

        -- Create autocommands for documentation-style files
        local doc_filetypes = { "help", "markdown", "md" }

        -- vim.api.nvim_create_autocmd("FileType", {
        --     pattern = doc_filetypes,
        --     callback = function()
        --         -- Set text width and enable wrapping for documentation files
        --         vim.opt_local.textwidth = 80
        --         vim.opt_local.wrap = true
        --     end,
        -- })
        --
        -- -- Handle DevDocs and other read-only markdown files
        -- vim.api.nvim_create_autocmd("BufReadPost", {
        --     pattern = { "*.md", "*.markdown" },
        --     callback = function(event)
        --         if vim.bo[event.buf].readonly then
        --             vim.opt_local.textwidth = 80
        --             vim.opt_local.wrap = true
        --         end
        --     end,
        -- })
    end,
}
