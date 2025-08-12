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

function table.empty(self)
  for _, _ in pairs(self) do
    return false
  end
  return true
end

function M.trim(str) return str:match "^%s*(.-)%s*$" end

function M.ReloadConfig()
  for name, _ in pairs(package.loaded) do
    if name:match "^user" then -- replace 'user' with your config module prefix
      package.loaded[name] = nil
    end
  end
  local config_file = vim.fn.stdpath "config" .. "/init.lua"
  dofile(config_file)
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
vim.api.nvim_create_user_command("SourceConfig", M.ReloadConfig, {})
return M
