-- Customize Mason

---@type LazySpec
return {
  -- use mason-tool-installer for automatically installing Mason packages
  {
    "WhoIsSethDaniel/mason-tool-installer.nvim",
    -- overrides `require("mason-tool-installer").setup(...)`
    -- NOTE: function form so this runs *after* astrocommunity packs' opts
    -- functions (which inject their own tools), letting us filter the
    -- final merged list before mason-tool-installer ever sees it
    opts = function(_, opts)
      -- Make sure to use the names found in `:Mason`

      local our_tools = {
        -- Language Servers (LSPs)
        "lua-language-server",
        "clangd", -- C/C++
        "bash-language-server", -- Bash
        "kotlin-language-server", -- Kotlin
        "taplo", -- TOML

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
        "selene", -- Lua
        "markdownlint", -- Markdown
        "shellcheck", -- Shell
        "hadolint", -- Dockerfile

        -- Debuggers
        "delve", -- Go

        -- Go Code Actions
        "gomodifytags", -- Go struct tag manipulation
        "impl", -- Go interface implementation generator

        -- Other Tools
        "tree-sitter-cli", -- Syntax parsing / highlighting
        "cmakelang", -- CMake syntax
      }

      opts.ensure_installed = require("astrocore").list_insert_unique(
        opts.ensure_installed,
        our_tools
      )

      -- Mason package name -> actual binary name, only where they differ
      local bin_name = {
        ["tree-sitter-cli"] = "tree-sitter",
        ["delve"] = "dlv",
        ["debugpy"] = "debugpy-adapter",
        ["cmakelang"] = "cmake-format",
      }

      -- Prefer a system-installed binary over Mason's copy: temporarily
      -- strip Mason's own bin dir from $PATH and do one `exepath` lookup
      -- (like `command -v`), instead of Mason always "winning" because
      -- it prepends itself to $PATH
      local mason_bin = vim.fn.stdpath "data" .. "/mason/bin"
      local function has_system_binary(pkg)
        local bin = bin_name[pkg] or pkg
        local real_path = vim.env.PATH
        vim.env.PATH = real_path:gsub(vim.pesc(mason_bin) .. ":?", "")
        local found = vim.fn.exepath(bin) ~= ""
        vim.env.PATH = real_path
        return found
      end

      local skipped, kept = {}, {}
      for _, tool in ipairs(opts.ensure_installed) do
        table.insert(has_system_binary(tool) and skipped or kept, tool)
      end
      opts.ensure_installed = kept

      -- If a tool already has a Mason copy installed but a system binary
      -- now also exists for it, uninstall the Mason copy so there's only
      -- one binary in play (defer: mason-registry isn't ready this early)
      if #skipped > 0 then
        vim.schedule(function()
          local ok, registry = pcall(require, "mason-registry")
          if not ok then return end
          registry.refresh(function()
            for _, pkg in ipairs(skipped) do
              local pkg_ok, p = pcall(registry.get_package, pkg)
              if pkg_ok and p:is_installed() then p:uninstall() end
            end
          end)
        end)
      end
    end,
  },
}
