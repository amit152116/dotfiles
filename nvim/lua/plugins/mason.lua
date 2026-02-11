-- Customize Mason

---@type LazySpec
return {
  -- use mason-tool-installer for automatically installing Mason packages
  {
    "WhoIsSethDaniel/mason-tool-installer.nvim",
    -- overrides `require("mason-tool-installer").setup(...)`
    opts = {
      -- Make sure to use the names found in `:Mason`

      ensure_installed = {
        -- Language Servers (LSPs)
        "lua-language-server",
        "clangd", -- C/C++
        "gopls", -- Go
        "bash-language-server", -- Bash
        "kotlin-language-server", -- Kotlin
        "taplo", -- TOML
        "basedpyright", -- Python

        -- Formatters
        "stylua", -- Lua
        "clang-format", -- C/C++
        "goimports", -- Go
        "gofumpt", -- Go
        "golines", -- Go
        "shfmt", -- Shell
        "beautysh", -- Shell
        "prettier", -- JS/TS/JSON/etc.
        "ktlint", -- Kotlin
        "google-java-format", -- Java

        -- Linters
        "ruff", -- Python
        "selene", -- Lua
        "markdownlint", -- Markdown
        "shellcheck", -- Shell
        "hadolint", -- Dockerfile

        -- Debuggers
        "debugpy", -- Python
        "delve", -- Go

        -- Go Code Actions
        "gomodifytags", -- Go struct tag manipulation
        "impl", -- Go interface implementation generator

        -- Other Tools
        "tree-sitter-cli", -- Syntax parsing / highlighting
        "cmakelang", -- CMake syntax
      },
    },
  },
}
