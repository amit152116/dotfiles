local Snacks = require "snacks"
---@type LazySpec
return {
  "AstroNvim/astrolsp",
  ---@type AstroLSPOpts
  opts = {
    features = {
      codelens = true,
      inlay_hints = true,
      semantic_tokens = true,
      signature_help = false,
    },
    defaults = {
      hover = {
        border = "rounded",
        silent = false,
      },
    },
    formatting = {
      format_on_save = {
        enabled = true,
        allow_filetypes = {},
        ignore_filetypes = {},
      },
      disabled = {
        "lemminx", -- use prettier for XML formatting via conform instead
      },
      timeout_ms = 3200,
    },
    servers = {
      "clangd", -- C/C++ LSP (system binary, not Mason-managed)
    },
    ---@diagnostic disable: missing-fields
    config = {
      clangd = {
        filetypes = { "c", "cpp", "objc", "objcpp", "cuda" },
        -- .git unreliable (nested submodule .git / stray ancestor repos); anchor on
        -- colcon workspace marker instead: nearest src/ dir with a build/ or install/ sibling
        root_dir = function(bufnr, on_dir)
          local fname = vim.api.nvim_buf_get_name(bufnr)
          local dir = vim.fs.dirname(fname)
          local root
          while dir and dir ~= "/" do
            if
              vim.uv.fs_stat(dir .. "/src")
              and (
                vim.uv.fs_stat(dir .. "/build")
                or vim.uv.fs_stat(dir .. "/install")
              )
            then
              root = dir
              break
            end
            dir = vim.fs.dirname(dir)
          end
          if not root then
            local found = vim.fs.find(
              { "compile_commands.json", "compile_flags.txt", ".git" },
              { upward = true, path = fname }
            )[1]
            root = found and vim.fs.dirname(found)
          end
          on_dir(root or vim.fn.getcwd())
        end,
        cmd = {
          "clangd",
          "--background-index",
          "--background-index-priority=normal",
          "--clang-tidy",
          "--header-insertion=iwyu",
          "--completion-style=bundled",
          "--function-arg-placeholders=true",
          "--pch-storage=memory", -- Store PCH in memory for faster access
          "--enable-config", -- Enable .clangd configuration files
          "--malloc-trim", -- Reduce memory usage
          "--log=error", -- Only log errors
        },
      },
      codebook = {
        enabled = false,
        cmd = { vim.fn.stdpath "data" .. "/mason/bin/codebook-lsp", "serve" },
        filetypes = {
          "c",
          "go",
          "java",
          "javascript",
          "lua",
          "markdown",
          "odin",
          "plaintext",
          "python",
          "ruby",
          "rust",
          "toml",
          "typescript",
        },
        init_options = {
          logLevel = "warn",
          checkWhileTyping = true,
          diagnosticSeverity = "information",
        },
        root_dir = function(bufnr, on_dir)
          local fname = vim.api.nvim_buf_get_name(bufnr)
          local root = vim.fs.dirname(
            vim.fs.find({ ".git", ".codebook.toml", "codebook.toml" }, {
              upward = true,
              path = fname,
            })[1]
          ) or vim.fn.getcwd()
          on_dir(root)
        end,
      },
      ruff = {
        on_attach = function(client)
          client.server_capabilities.hoverProvider = false
        end,
      },
      basedpyright = {
        before_init = function(_, c)
          if not c.settings then c.settings = {} end
          if not c.settings.python then c.settings.python = {} end
          c.settings.python.pythonPath = vim.fn.exepath "python"
        end,
        settings = {
          basedpyright = {
            disableOrganizeImports = true,
            analysis = {
              autoimportCompletions = true,
              autoFormatStrings = true,
              autoSearchPaths = true,
              diagnosticMode = "workspace",
              diagnosticSeverityOverrides = {
                reportUnusedImport = "none",
                reportUnusedFunction = "none",
                reportUnusedVariable = "none",
                reportGeneralTypeIssues = "hint",
                reportOptionalMemberAccess = "none",
                reportOptionalSubscript = "none",
                reportPrivateImportUsage = "none",
              },
              inlayHints = {
                variableTypes = true,
                callArgumentNames = true,
                functionReturnTypes = true,
                genericTypes = true,
              },
              typeCheckingMode = "standard",
              useLibraryCodeForTypes = true,
            },
          },

          python = {
            analysis = {
              autoSearchPaths = true,
              useLibraryCodeForTypes = true,
              diagnosticMode = "workspace",
            },
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
        settings = {
          xml = {
            downloadExternalResources = {
              enabled = true,
            },
            validation = {
              enabled = true,
              schema = {
                enabled = true,
              },
            },
            format = {
              enabled = true,
            },
          },
        },
      },
      bashls = {
        filetypes = { "sh", "bash", "zsh" },
      },
      neocmake = {
        cmd = { "neocmakelsp", "stdio" }, -- newer versions use a subcommand instead of --stdio
      },
    },
    handlers = {},
    autocmds = {
      lsp_codelens_refresh = {
        cond = "textDocument/codeLens", -- autocmd group is torn down once no attached client supports codelens
        {
          event = { "InsertLeave", "BufEnter" },
          desc = "Refresh codelens (buffer)",
          callback = function(args)
            if require("astrolsp").config.features.codelens then
              vim.lsp.codelens.enable(true, { bufnr = args.buf })
            end
          end,
        },
      },
    },
    mason_lspconfig = {
      servers = {},
    },
    mappings = {
      n = {
        -- replaced below by Snacks pickers
        ["gK"] = false,
        ["<Leader>lG"] = false, -- original Workspace Symbols
        ["<Leader>lR"] = false, -- original references
        ["<Leader>lD"] = false,
        ["<Leader>ld"] = {
          function()
            Snacks.picker.diagnostics {
              focus = "list",
            }
          end,
          desc = "Search Diagnostics",
        },
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
        ["grr"] = {
          function() Snacks.picker.lsp_references() end,
          desc = "Search References",
        },

        ["<Leader>ln"] = {
          function() vim.lsp.buf.rename() end,
          desc = "Rename symbol",
        },
      },
    },
  },
}
