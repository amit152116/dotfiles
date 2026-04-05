---
--- git-flow.nvim - common.lua
--- Shared git utilities used across backup and worktree modules.
--- Uses vim.fn.system (sync, no external deps) for path resolution.
--- Worktree module keeps plenary.job for its async picker operations.
---

local M = {}

--- Run a git command synchronously from a specific directory.
--- Safe: uses a list-form call so no shell-injection is possible.
---@param args string[] Git subcommand arguments
---@param cwd string|nil Directory to run git in (defaults to current buffer dir)
---@return string output Trimmed stdout
---@return number code Shell exit code (0 = success)
function M.git_sync(args, cwd)
  local dir = cwd or vim.fn.expand "%:p:h"
  local cmd = vim.list_extend({ "git", "-C", dir }, args)
  local output = vim.trim(vim.fn.system(cmd))
  return output, vim.v.shell_error
end

--- Resolve the git repository root from a given directory.
---@param dir string|nil Directory to start from (defaults to current buffer dir)
---@return string|nil git_root Absolute path, or nil if not a git repo
function M.get_git_root(dir)
  local root, code = M.git_sync({ "rev-parse", "--show-toplevel" }, dir)
  if code ~= 0 or root == "" then return nil end
  return root
end

--- Resolve the git common directory (bare repo root or the .git folder path).
--- Useful for placing new worktrees as siblings of the bare repo.
---@param dir string|nil Directory to start from (defaults to current buffer dir)
---@return string|nil Absolute path, or nil
function M.get_git_common_dir(dir)
  local path = dir or vim.fn.expand "%:p:h"
  local result, code = M.git_sync({ "rev-parse", "--git-common-dir" }, path)
  if code ~= 0 or result == "" then return nil end

  -- git may return a relative path (e.g. ".git") – resolve it
  if not result:match "^/" then result = path .. "/" .. result end

  -- Normalise and strip trailing slash
  return vim.fn.fnamemodify(result, ":p"):gsub("/$", "")
end

--- Derive a filesystem-safe repository name from the git root path.
---@param git_root string Absolute path returned by get_git_root()
---@return string name Alphanumeric + hyphens only
function M.get_repo_name(git_root)
  return vim.fn.fnamemodify(git_root, ":t"):gsub("[^%w%-]", "_")
end

--- Ensure a directory exists, creating it (and parents) if needed.
---@param dir string
function M.ensure_dir(dir)
  if vim.fn.isdirectory(dir) == 0 then vim.fn.mkdir(dir, "p") end
end

return M
