local M = {}
function M.show_debug(...)
  local args = { ... }
  if #args == 0 then return end
  local buf = vim.api.nvim_create_buf(false, true)
  local win = vim.api.nvim_open_win(buf, true, {
    relative = "cursor",
    width = 60,
    height = 10,
    col = 2,
    row = 1,
    style = "minimal",
    border = "rounded",
  })
  vim.api.nvim_buf_set_lines(
    buf,
    0,
    -1,
    false,
    vim.split(vim.inspect(args), "\n")
  )
end

function M.run_command(command, stdout, stderr)
  -- vim.notify("Running command: " .. command, vim.log.levels.INFO)
  vim.fn.jobstart(command, {
    stdout_buffered = true,
    on_stdout = stdout,
    on_stderr = stderr,
  })
end

function M.ReloadConfig()
  -- Save all buffers
  vim.cmd "wall"

  -- Clear all loaded Lua modules from your config so they get reloaded
  for name, _ in pairs(package.loaded) do
    if
      name:match "^user" -- Change "user" to your config's Lua module root
      or name:match "^plugins"
      or name:match "^config"
    then
      package.loaded[name] = nil
    end
  end

  -- Source init.lua
  dofile(vim.fn.stdpath "config" .. "/init.lua")

  -- Reload lazy.nvim setup
  require("lazy").reload {}

  vim.notify("Neovim config reloaded!", vim.log.levels.INFO)
end

-- Search visually selected text
function M.search_selected(opts)
  opts = opts or {}

  -- Yank the visually selected text into the default register
  vim.cmd 'normal! "vy'

  -- Get the yanked text
  local selected_text = vim.fn.getreg '"'

  -- Escape for Vim's literal search
  local escaped_text = vim.fn.escape(selected_text, "/\\")
  vim.fn.setreg("/", "\\V" .. escaped_text) -- set literal search

  -- Trigger normal buffer search
  vim.cmd "normal! n"

  return selected_text
end

-- Create a command and keymap
vim.api.nvim_create_user_command("ReloadConfig", M.ReloadConfig, {})
return M
