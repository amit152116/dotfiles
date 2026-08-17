local Snacks = require "snacks"
local helper = require "utils.helper"
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
      "basedpyright",
      "ruff",
      "gopls",
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
        init_options = {
          settings = {
            lineLength = 88,
            lint = { preview = true },
            format = { preview = true },
          },
        },
        on_attach = function(client)
          -- basedpyright handles hover; ruff only does lint/format actions
          client.server_capabilities.hoverProvider = false
        end,
      },
      basedpyright = {
        before_init = function(_, c)
          if not c.settings then c.settings = {} end
          if not c.settings.python then c.settings.python = {} end
          (c.settings.python --[[@as {pythonPath: string}]]).pythonPath =
            vim.fn.exepath "python"
        end,
        settings = {
          basedpyright = {
            disableOrganizeImports = true, -- ruff handles imports
            analysis = {
              autoimportCompletions = true,
              autoFormatStrings = true,
              autoSearchPaths = true,
              diagnosticMode = "workspace",
              typeCheckingMode = "standard",
              useLibraryCodeForTypes = true,
              inlayHints = {
                variableTypes = true,
                callArgumentNames = true,
                functionReturnTypes = true,
                genericTypes = true,
              },
              diagnosticSeverityOverrides = {
                -- ruff handles unused imports/vars; suppress pyright duplicates
                reportUnusedImport = "none",
                reportUnusedVariable = "none",
                -- surface type issues at appropriate severity
                reportGeneralTypeIssues = "warning",
                reportOptionalMemberAccess = "warning",
                reportOptionalSubscript = "warning",
                reportPrivateImportUsage = "none",
                reportUnusedFunction = "information",
                reportMissingTypeArgument = "information",
              },
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
              enabled = false, -- fetching schemas on open hangs/crashes nvim for ROS XML
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
    commands = {
      LspLog = {
        function() vim.cmd.edit(vim.lsp.log.get_filename()) end,
        desc = "Open LSP log file",
      },
      LspInfo = {
        function() vim.cmd "checkhealth vim.lsp" end,
        desc = "Show LSP info (checkhealth)",
      },
      LspSymbolKinds = {
        function(args)
          local bufnr = 0
          local method, params
          if args.bang then
            method = "workspace/symbol"
            params = { query = args.args ~= "" and args.args or "a" }
          else
            method = "textDocument/documentSymbol"
            params =
              { textDocument = vim.lsp.util.make_text_document_params(bufnr) }
          end

          local responses =
            vim.lsp.buf_request_sync(bufnr, method, params, 2000)
          if not responses then
            vim.notify("No LSP response", vim.log.levels.WARN)
            return
          end

          local kinds = {}
          local function walk(items)
            for _, item in ipairs(items or {}) do
              kinds[vim.lsp.protocol.SymbolKind[item.kind]] = true
              if item.children then walk(item.children) end
            end
          end
          for _, resp in pairs(responses) do
            walk(resp.result)
          end

          local list = vim.tbl_keys(kinds)
          table.sort(list)
          vim.notify(
            ("%s symbol kinds (%s): %s"):format(
              args.bang and "Workspace" or "Document",
              vim.bo[bufnr].filetype,
              #list > 0 and table.concat(list, ", ") or "none"
            ),
            vim.log.levels.INFO
          )
        end,
        desc = "Dump LSP symbol kinds seen in buffer (! for workspace, optional query arg)",
        bang = true,
        nargs = "?",
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
          function() Snacks.picker.lsp_symbols { title = "Document Symbols" } end,
          desc = "Search Document Symbols",
        },

        ["gsc"] = {
          function()
            Snacks.picker.lsp_symbols {
              title = "Document Symbols: Classes/Modules",
              filter = helper.lsp_symbol_filter {
                "Class",
                "Struct",
                "Interface",
                "Enum",
                "EnumMember",
                "Trait",
                "TypeParameter",
                "Namespace",
                "Module",
                "Package",
              },
            }
          end,
          desc = "Document Symbols: Classes/Modules",
        },
        ["gsv"] = {
          function()
            Snacks.picker.lsp_symbols {
              title = "Document Symbols: Variables",
              filter = helper.lsp_symbol_filter {
                "Variable",
                "Field",
                "Property",
                "Constant",
              },
            }
          end,
          desc = "Document Symbols: Variables",
        },
        ["gsf"] = {
          function()
            Snacks.picker.lsp_symbols {
              title = "Document Symbols: Functions",
              filter = helper.lsp_symbol_filter {
                "Function",
                "Method",
                "Constructor",
              },
            }
          end,
          desc = "Document Symbols: Functions",
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
          function()
            Snacks.picker.lsp_workspace_symbols { title = "Workspace Symbols" }
          end,
          desc = "Workspace Symbols",
        },

        ["gwc"] = {
          function()
            Snacks.picker.lsp_workspace_symbols {
              title = "Workspace Symbols: Classes/Modules",
              filter = helper.lsp_symbol_filter {
                "Class",
                "Struct",
                "Interface",
                "Enum",
                "EnumMember",
                "Trait",
                "TypeParameter",
                "Namespace",
                "Module",
                "Package",
              },
            }
          end,
          desc = "Workspace Symbols: Classes/Modules",
        },
        ["gwv"] = {
          function()
            Snacks.picker.lsp_workspace_symbols {
              title = "Workspace Symbols: Variables",
              filter = helper.lsp_symbol_filter {
                "Variable",
                "Field",
                "Property",
                "Constant",
              },
            }
          end,
          desc = "Workspace Symbols: Variables",
        },
        ["gwf"] = {
          function()
            Snacks.picker.lsp_workspace_symbols {
              title = "Workspace Symbols: Functions",
              filter = helper.lsp_symbol_filter {
                "Function",
                "Method",
                "Constructor",
              },
            }
          end,
          desc = "Workspace Symbols: Functions",
        },
        ["grr"] = {
          function() Snacks.picker.lsp_references() end,
          desc = "Search References",
        },

        ["<Leader>ln"] = {
          function() vim.lsp.buf.rename() end,
          desc = "Rename symbol",
        },
        ["<Leader>la"] = {
          function() vim.lsp.buf.code_action() end,
          desc = "LSP code action",
          cond = "textDocument/codeAction",
        },
      },
    },
  },
}
