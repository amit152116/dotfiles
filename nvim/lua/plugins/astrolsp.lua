-- AstroLSP allows you to customize the features in AstroNvim's LSP configuration engine
-- Configuration documentation can be found with `:h astrolsp`

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
          "--header-insertion=iwyu",
          "--completion-style=detailed",
          "--function-arg-placeholders=true",
          "--pch-storage=memory", -- Store PCH in memory for faster access
          "--enable-config", -- Enable .clangd configuration files
          "--malloc-trim", -- Reduce memory usage
          "--log=error", -- Only log errors
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
            inlayHints = {
              parameterNames = true, -- Show parameter names in function calls
              deducedTypes = true, -- Show deduced types for auto
              designators = true, -- Show designators for aggregates
            },
          },
        },
        -- Add more LSP-specific settings
        capabilities = {
          offsetEncoding = { "utf-16" }, -- Important for large files
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

    mappings = {
      -- a `cond` key can provided as the string of a server capability to be required to attach,
      -- or a function with `client` and `bufnr` parameters from the `on_attach` that returns a boolean
      n = {
        -- `gcc` - Toggles the current line using linewise comment
        -- `gbc` - Toggles the current line using blockwise comment
        -- `[count]gcc` - Toggles the number of line given as a prefix-count using linewise
        -- `[count]gbc` - Toggles the number of line given as a prefix-count using blockwise
        -- `gc[count]{motion}` - (Op-pending) Toggles the region using linewise comment
        -- `gb[count]{motion}` - (Op-pending) Toggles the region using blockwise comment

        ["gra"] = false,
        ["gri"] = false,
        ["grn"] = false,
        ["grr"] = false,
        ["grt"] = false,
        ["gK"] = false,
        ["<Leader>lG"] = false, -- original Workspace Symbols
        ["<Leader>lS"] = false, -- original lsp_document_symbols
        ["<Leader>lR"] = false, -- original references
        ["gd"] = {
          function()
            require("telescope.builtin").lsp_definitions {
              reuse_win = true,
            }
          end,
          desc = "Goto Definition",
        },

        ["gs"] = {
          function() require("telescope.builtin").lsp_document_symbols {} end,
          desc = "Search Document Symbols",
        },

        ["gy"] = {
          function()
            require("telescope.builtin").lsp_type_definitions {
              reuse_win = true,
            }
          end,
          desc = "Goto Type Definition",
        },
        ["gw"] = {
          function()
            require("telescope.builtin").lsp_dynamic_workspace_symbols()
          end,
          desc = "Workspace Symbols",
        },
        -- Disable original mappings
        ["<Leader>lr"] = {
          function() require("telescope.builtin").lsp_references() end,
          desc = "Search References",
        },
        ["<Leader>ld"] = {
          function() require("telescope.builtin").diagnostics() end,
          desc = "Search Diagnostics",
        },
        ["<Leader>lD"] = false,
        ["<Leader>ls"] = {
          "<cmd>Trouble symbols toggle<cr>",
          desc = "Symbols Outline",
        },
        ["<Leader>ln"] = {
          function() vim.lsp.buf.rename() end,
          desc = "Rename symbol",
        },
        ["<Leader>xx"] = {
          "<cmd>Trouble<cr>",
          desc = "Toggle Trouble",
        },
        ["<Leader>xt"] = {
          "<cmd>Trouble lsp toggle focus=false win.position=right<cr>",
          desc = "LSP Definitions / references / ... (Trouble)",
        },
      },
      x = {
        -- `gc` - Toggles the region using linewise comment
        -- `gb` - Toggles the region using blockwise comment
        ["gra"] = false,
      },
    },
  },
}
