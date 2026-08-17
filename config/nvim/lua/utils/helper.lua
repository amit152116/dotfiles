local M = {}
local Snacks = require "snacks"

---@type fun(...)
_G.inspect = function(...) Snacks.debug.inspect(...) end

---@param msg? string|string[]
---@param opts? snacks.notify.Opts
_G.backtrace = function(msg, opts) Snacks.debug.backtrace(msg, opts) end

---@param fn fun()
---@param opts? {count?: number, flush?: boolean, title?: string}
_G.profile = function(fn, opts) Snacks.debug.profile(fn, opts) end

---@param opts snacks.debug.cmd
_G.cmd = function(opts) Snacks.debug.cmd(opts) end

vim.print = _G.inspect

-- filetypes with per-language lsp_symbols filter overrides in plugins/snacksPicker.lua;
-- symbol-kind filter must repeat these keys or the lang-specific list wins over `default`
local SYMBOL_FILTER_LANGS = {
  "c",
  "cpp",
  "python",
  "lua",
  "go",
  "rust",
  "sh",
  "bash",
  "zsh",
  "markdown",
  "help",
  "xml",
}

---@param kinds string[] LSP symbol kinds to keep, e.g. {"Class","Struct"}
---@return table<string, string[]> filter for Snacks.picker.lsp_symbols/lsp_workspace_symbols
function M.lsp_symbol_filter(kinds)
  local filter = { default = kinds }
  for _, ft in ipairs(SYMBOL_FILTER_LANGS) do
    filter[ft] = kinds
  end
  return filter
end

function M.get_selected_text(opts)
  opts = opts or {}
  -- Yank the visually selected text into the default register
  vim.cmd 'normal! "vy'

  -- Get the yanked text
  local selected_text = vim.fn.getreg '"'
  return selected_text
end

function M.ensure_directory_exist(path)
  local dir = path:match "(.*[/\\])"
  if dir then
    dir = dir:gsub("[/\\]$", "") -- remove trailing slash/backslash
    if vim.fn.isdirectory(dir) == 0 then vim.fn.mkdir(dir, "p") end
  end
end

-- filenames that may hold secrets -- AI suggestion plugins must never see these buffers
local secret_patterns = {
  "%.env$",
  "%.env%.",
  "%.envrc$",
  "%.netrc$",
  "%.npmrc$",
  "%.pgpass$",
  "credentials%.json$",
  "secrets%.ya?ml$",
  "secrets%.json$",
  "%.pem$",
  "%.key$",
  "%.p12$",
  "%.pfx$",
  "id_rsa$",
  "id_ed25519$",
  "id_ecdsa$",
  "known_hosts$",
  "%.kube/config$",
  "%.aws/credentials$",
}

---@param bufnr integer
---@return boolean
function M.is_secret_buf(bufnr)
  local fname = vim.api.nvim_buf_get_name(bufnr)
  for _, pat in ipairs(secret_patterns) do
    if fname:match(pat) then return true end
  end
  return false
end

-- NOTE: Don't escape these as they should work in very magic mode:
-- . * + ? ^ $ ( ) [ ] { } | \
function M.vim_regex_escape(vim_pattern)
  -- In very magic mode (\v), these characters have special meaning and need escaping:
  vim_pattern = vim_pattern:gsub("@", "\\@") -- @ is special in vim regex
  vim_pattern = vim_pattern:gsub("&", "\\&") -- & is special
  vim_pattern = vim_pattern:gsub("<", "\\<") -- < for word boundaries
  vim_pattern = vim_pattern:gsub(">", "\\>") -- > for word boundaries
  vim_pattern = vim_pattern:gsub("~", "\\~") -- ~ is special
  vim_pattern = vim_pattern:gsub("#", "\\#") -- # can be special

  return vim_pattern
end

return M
