-- Customize None-ls sources
-- Make sure null-ls is installed
local null_ls = require "null-ls"

null_ls.setup {
  sources = {

    -- Dockerfile linter
    null_ls.builtins.diagnostics.hadolint.with {
      filetypes = { "dockerfile" },
    },

    -- Kotlin linter
    null_ls.builtins.diagnostics.ktlint.with {
      filetypes = { "kotlin" },
    },

    -- Markdown linter
    null_ls.builtins.diagnostics.markdownlint.with {
      filetypes = { "markdown" },
    },

    -- Lua linter
    null_ls.builtins.diagnostics.selene.with {
      filetypes = { "lua" },
    },

    -- ========================
    -- Code Actions
    -- ========================
    null_ls.builtins.code_actions.refactoring, -- generic code refactoring

    -- Go-specific actions (since you work with Go)
    null_ls.builtins.code_actions.gomodifytags,
    null_ls.builtins.code_actions.impl,
  },
}
---@type LazySpec
return {
  {
    "stevearc/conform.nvim",
    event = "User AstroFile",
    -- enabled = false,
    cmd = "ConformInfo",
    specs = {
      {
        "AstroNvim/astrolsp",
        optional = true,
        opts = { formatting = { disabled = true } },
      },
      {
        "jay-babu/mason-null-ls.nvim",
        optional = true,
        opts = { methods = { formatting = false } },
      },
    },
    dependencies = {
      { "williamboman/mason.nvim", optional = true },
      {
        "AstroNvim/astrocore",
        opts = {
          options = {
            opt = { formatexpr = "v:lua.require'conform'.formatexpr()" },
          },
          commands = {
            Format = {
              function(args)
                local range = nil
                if args.count ~= -1 then
                  local end_line = vim.api.nvim_buf_get_lines(
                    0,
                    args.line2 - 1,
                    args.line2,
                    true
                  )[1]
                  range = {
                    start = { args.line1, 0 },
                    ["end"] = { args.line2, end_line:len() },
                  }
                end
                require("conform").format { async = true, range = range }
              end,
              desc = "Format buffer",
              range = true,
            },
          },
          mappings = {
            n = {
              ["<Leader>lf"] = {
                function() vim.cmd.Format() end,
                desc = "Format buffer",
              },
              ["<Leader>lz"] = {
                function() vim.cmd.ConformInfo() end,
                desc = "Conform information",
              },
              ["<Leader>uf"] = {
                function()
                  vim.b.autoformat =
                    not vim.F.if_nil(vim.b.autoformat, vim.g.autoformat, true)
                  require("astrocore").notify(
                    string.format(
                      "Buffer autoformatting %s",
                      vim.b.autoformat and "on" or "off"
                    )
                  )
                end,
                desc = "Toggle autoformatting (buffer)",
              },
              ["<Leader>uF"] = {
                function()
                  vim.g.autoformat, vim.b.autoformat =
                    not vim.F.if_nil(vim.g.autoformat, true), nil
                  require("astrocore").notify(
                    string.format(
                      "Global autoformatting %s",
                      vim.g.autoformat and "on" or "off"
                    )
                  )
                end,
                desc = "Toggle autoformatting (global)",
              },
            },
          },
        },
      },
    },
    opts = {
      -- Use LSP as fallback
      default_format_opts = { lsp_format = "fallback" },

      -- Format on save
      format_on_save = function(bufnr)
        if vim.F.if_nil(vim.b[bufnr].autoformat, vim.g.autoformat, true) then
          return { timeout_ms = 500, lsp_format = "fallback" }
        end
      end,

      -- 🎯 Configure formatters
      formatters_by_ft = {
        lua = { "stylua" },
        sh = { "shfmt", "beautysh" },
        bash = { "shfmt", "beautysh" },
        zsh = { "shfmt", "beautysh" },
        c = { "clang_format" },
        proto = { "buf_ls" },
        cpp = { "clang_format" },
        go = { "goimports", "gofumpt", "golines", "gomodifytags", "impl" },
        python = { "ruff", "isort", "black" },
        json = { "prettier" }, -- if you install prettier later
        yaml = { "prettier" },
        markdown = { "markdownlint" },
        toml = { "taplo" },
        java = { "google-java-format" },
        kotlin = { "ktlint" },
        sql = { "pgformatter", "postgres-language-server" },
        qss = { "prettier" },
      },

      -- 🎯 Custom formatter setup
      formatters = {
        clang_format = {
          command = "clang-format",
          args = { "--style=file" },
        },
        shfmt = {
          command = "shfmt",
        },
      },
    },
  },
}
