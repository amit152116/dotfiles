-- AstroLSP allows you to customize the features in AstroNvim's LSP configuration engine
-- Configuration documentation can be found with `:h astrolsp`
-- NOTE: We highly recommend setting up the Lua Language Server (`:LspInstall lua_ls`)
--       as this provides autocomplete and documentation while editing

---@type LazySpec
return {
    "AstroNvim/astrolsp",
    ---@type AstroLSPOpts
    opts = {
        -- Configuration table of features provided by AstroLSP
        features = {
            codelens = true, -- enable/disable codelens refresh on start
            inlay_hints = true, -- enable/disable inlay hints on start
            semantic_tokens = true, -- enable/disable semantic token highlighting
            signature_help = true,
            code_actions = true,
        },
        -- customize lsp formatting options
        formatting = {
            -- control auto formatting on save
            format_on_save = {
                enabled = true, -- enable or disable format on save globally
                ignore_filetypes = { -- disable format on save for specified filetypes
                    -- "python",
                },
            },
            disabled = { -- disable formatting capabilities for the listed language servers
                -- disable lua_ls formatting capability if you want to use StyLua to format your lua code
                -- "lua_ls",
            },
            timeout_ms = 3200, -- default format timeout
            -- filter = function(client) -- fully override the default formatting function
            --   return true
            -- end
        },
        -- enable servers that you already have installed without mason
        servers = {
            -- "pyright",
        },
        -- customize language server configuration options passed to `lspconfig`
        ---@diagnostic disable: missing-fields
        config = {
            clangd = {
                cmd = {
                    "clangd",
                    "--background-index",
                    "--clang-tidy",
                    "--clang-tidy-checks=performance-*,readability-*,-readability-magic-numbers,cert-dcl21-cpp,cert-dcl58-cpp,cert-err34-c,cert-err52-cpp,cert-err60-cpp,cert-flp30-c,cert-msc50-cpp,cert-msc51-cpp,cert-str34-c,cppcoreguidelines-interfaces-global-init,cppcoreguidelines-narrowing-conversions,cppcoreguidelines-pro-type-member-init,cppcoreguidelines-pro-type-static-cast-downcast,cppcoreguidelines-slicing,google-default-arguments,google-explicit-constructor,google-runtime-operator,hicpp-exception-baseclass,hicpp-multiway-paths-covered,misc-misplaced-const,misc-new-delete-overloads,misc-no-recursion,misc-non-copyable-objects,misc-throw-by-value-catch-by-reference,misc-unconventional-assign-operator,misc-uniqueptr-reset-release,mpi-buffer-deref,mpi-type-mismatch,openmp-use-default-none,performance-faster-string-find,performance-for-range-copy,performance-implicit-conversion-in-loop,performance-inefficient-algorithm,performance-inefficient-string-concatenation,performance-inefficient-vector-operation,performance-move-const-arg,performance-move-constructor-init,performance-no-automatic-move,performance-noexcept-move-constructor,performance-trivially-destructible,performance-type-promotion-in-math-fn,performance-unnecessary-copy-initialization,performance-unnecessary-value-param,readability-avoid-const-params-in-decls,readability-const-return-type,readability-container-size-empty,readability-convert-member-functions-to-static,readability-delete-null-pointer,readability-deleted-default,readability-inconsistent-declaration-parameter-name,readability-make-member-function-const,readability-misleading-indentation,readability-misplaced-array-index,readability-non-const-parameter,readability-redundant-control-flow,readability-redundant-declaration,readability-redundant-function-ptr-dereference,readability-redundant-smartptr-get,readability-redundant-string-cstr,readability-redundant-string-init,readability-simplify-subscript-expr,readability-static-accessed-through-instance,readability-static-definition-in-anonymous-namespace,readability-string-compare,readability-uniqueptr-delete-release,readability-use-anyofallof,bugprone-*,clang-analyzer-*,modernize-*,portability-*",
                    "--header-insertion=iwyu",
                    "--completion-style=detailed",
                    "--function-arg-placeholders=true",
                },
                -- Add init_options to improve ROS workflow
                init_options = {
                    clangd = {
                        fallbackFlags = {
                            "-std=c++17",
                            "-Wall",
                            -- "-I/usr/include/c++/11",
                            -- "-I/usr/include/x86_64-linux-gnu/c++/11",
                            -- "-I/usr/include/c++/11/bits",
                            -- "-I/usr/include/c++/11/ext",
                            -- "-I/usr/include/c++/11/tr1",
                            -- "-I/usr/include/c++/11/debug",
                            -- "-I/usr/include/c++/11/parallel",
                            "-I/usr/include/eigen3",
                            "-I/usr/include",
                            "-I/usr/include/c++/11",
                            "-I/usr/include/x86_64-linux-gnu/c++/11",
                            "-Wextra",
                            "-isystem/usr/include",
                            "-isystem/usr/local/include",
                        },
                        compilationDatabasePath = ".",
                        compilationDatabaseDirectorySearch = "Recursive",
                        semanticHighlighting = true,
                        includeCleaner = false,
                    },
                },
            },
            gopls = {
                settings = {
                    usePlaceholders = false,
                    completeUnimported = true,
                },
            },
            lemminx = {
                filetypes = { "xml", "urdf" },
                init_options = {
                    documentFormatting = true,
                },
            },
        },
        -- customize how language servers are attached
        handlers = {
            -- a function without a key is simply the default handler, functions take two parameters, the server name and the configured options table for that server
            -- function(server, opts) require("lspconfig")[server].setup(opts) end

            -- the key is the server that is being setup with `lspconfig`
            -- rust_analyzer = false, -- setting a handler to false will disable the set up of that language server
            -- pyright = function(_, opts) require("lspconfig").pyright.setup(opts) end -- or a custom handler function can be passed
        },
        -- Configure buffer local auto commands to add when attaching a language server
        autocmds = {
            -- first key is the `augroup` to add the auto commands to (:h augroup)
            lsp_codelens_refresh = {
                -- Optional condition to create/delete auto command group
                -- can either be a string of a client capability or a function of `fun(client, bufnr): boolean`
                -- condition will be resolved for each client on each execution and if it ever fails for all clients,
                -- the auto commands will be deleted for that buffer
                cond = "textDocument/codeLens",
                -- cond = function(client, bufnr) return client.name == "lua_ls" end,
                -- list of auto commands to set
                {
                    -- events to trigger
                    event = { "InsertLeave", "BufEnter" },
                    -- the rest of the autocmd options (:h nvim_create_autocmd)
                    desc = "Refresh codelens (buffer)",
                    callback = function(args)
                        if require("astrolsp").config.features.codelens then
                            vim.lsp.codelens.refresh { bufnr = args.buf }
                        end
                    end,
                },
            },
        },
        -- A custom `on_attach` function to be run after the default `on_attach` function
        -- takes two parameters `client` and `bufnr`  (`:h lspconfig-setup`)
        on_attach = function(client, bufnr)
            -- this would disable semanticTokensProvider for all clients
            -- client.server_capabilities.semanticTokensProvider = nil
        end,
    },
}
