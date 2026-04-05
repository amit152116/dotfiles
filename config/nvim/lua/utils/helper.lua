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
