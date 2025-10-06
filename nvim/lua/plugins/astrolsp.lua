-- AstroLSP allows you to customize the features in AstroNvim's LSP configuration engine
-- Configuration documentation can be found with `:h astrolsp`
local Snacks = require "snacks"
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
      signature_help = false,
    },
    defaults = {
      hover = {
        border = "rounded", -- default border value for hover windows
        silent = false, -- disable hover silence by default
      },
    },
    -- customize lsp formatting options
    formatting = {
      -- control auto formatting on save
      format_on_save = {
        enabled = true, -- enable or disable format on save globally
        allow_filetypes = { -- enable format on save for specified filetypes only
          -- "go",
        },
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
      -- "pyright"
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
        init_options = {},
        capabilities = { offsetEncoding = "utf-16" },
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
    -- Extra configuration for the `mason-lspconfig.nvim` plugin
    mason_lspconfig = {
      servers = {},
    },
    -- mappings to be set up on attaching of a language server
    mappings = {
      n = {
        -- a `cond` key can provided as the string of a server capability to be required to attach, or a function with `client` and `bufnr` parameters from the `on_attach` that returns a boolean

        ["gK"] = false,
        ["<Leader>lG"] = false, -- original Workspace Symbols
        ["<Leader>lR"] = false, -- original references
        -- ["<Leader>lD"] = false,
        -- ["<Leader>ld"] = {
        --   function() Snacks.picker.diagnostics() end,
        --   desc = "Search Diagnostics",
        -- },
        ["gd"] = {
          function()
            Snacks.picker.lsp_definitions {
              reuse_win = true,
            }
          end,
          desc = "Goto Definition",
        },

        ["gs"] = {
          function() Snacks.picker.lsp_symbols() end,
          desc = "Search Document Symbols",
        },

        ["gy"] = {
          function()
            Snacks.picker.lsp_type_definitions {
              reuse_win = true,
            }
          end,
          desc = "Goto Type Definition",
        },
        ["gw"] = {
          function() Snacks.picker.lsp_workspace_symbols() end,
          desc = "Workspace Symbols",
        },
        -- Disable original mappings
        ["<Leader>lr"] = {
          function() Snacks.picker.lsp_references() end,
          desc = "Search References",
        },
        ["<Leader>ln"] = {
          function() vim.lsp.buf.rename() end,
          desc = "Rename symbol",
        },
      },
    },
    -- A custom `on_attach` function to be run after the default `on_attach` function
    -- takes two parameters `client` and `bufnr`  (`:h lspconfig-setup`)
    on_attach = function(client, bufnr)
      -- Disable semantic tokens if needed
      -- client.server_capabilities.semanticTokensProvider = nil

      if client.name == "clangd" then
        local opts =
          { buffer = bufnr, noremap = true, silent = true, desc = "" }

        -- 🧩 Run the current C++ file using CMake plugin
        vim.keymap.set(
          "n",
          "<Leader>le",
          ":CMakeRunCurrentFile<CR>",
          vim.tbl_extend("force", opts, { desc = "Run Current File" })
        )
        vim.keymap.set(
          "n",
          "<Leader>lE",
          ":CMakeQuickRun<CR>",
          vim.tbl_extend("force", opts, { desc = "Run Executable" })
        )
        vim.keymap.set(
          "n",
          "<Leader>lt",
          ":CMakeRunTest<CR>",
          vim.tbl_extend("force", opts, { desc = "Run Tests" })
        )

        -- 🧩 Compile current buffer to assembly (Compiler Explorer)
        vim.keymap.set(
          "n",
          "<Leader>lc",
          ":CECompile! compiler=g114<CR>",
          vim.tbl_extend("force", opts, {
            desc = "Compile & Show Assembly",
          })
        )

        -- 🧩 Compile with live update (auto on save)
        vim.keymap.set(
          "n",
          "<Leader>lC",
          ":CECompileLive! compiler=g114<CR>",
          vim.tbl_extend("force", opts, {
            desc = "Live Assembly View",
          })
        )

        -- 🧩 Open same code in browser on godbolt.org
        vim.keymap.set(
          "n",
          "<Leader>lO",
          ":CEOpenWebsite<CR>",
          vim.tbl_extend("force", opts, {
            desc = "Open in Compiler Explorer Website",
          })
        )
      end
    end,
  },
}
