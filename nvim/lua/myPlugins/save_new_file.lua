local M = {}
local Snacks = require "snacks"
local helper = require "utils.helper"

local function save_to_file(file_path)
  local ok, err = pcall(function()
    helper.ensure_directory_exist(file_path)

    local buf = vim.api.nvim_get_current_buf()
    local content = vim.api.nvim_buf_get_lines(buf, 0, -1, false)

    vim.fn.writefile(content, file_path)
    vim.api.nvim_buf_set_name(buf, file_path)
    vim.cmd("silent! edit " .. vim.fn.fnameescape(file_path))
  end)

  if ok then
    vim.notify("File saved to: " .. file_path, vim.log.levels.INFO)
  else
    vim.notify("Failed to save file: " .. err, vim.log.levels.ERROR)
  end
end

local function create_snacks_picker(file_name, opts)
  opts = opts or {}
  opts.cwd = opts.cwd or vim.fn.getcwd()

  local finder = function()
    local dirs = vim.fn.systemlist "fdfind --type d --hidden --exclude .git ."

    if not dirs or vim.tbl_isempty(dirs) then
      local glob_dirs = vim.fn.globpath(opts.cwd, "**/", false, true)
      dirs = {}
      for _, dir in ipairs(glob_dirs) do
        table.insert(dirs, vim.fn.fnamemodify(dir, ":."))
      end
    end

    -- Ensure current directory is always included
    table.insert(dirs, 1, ".")

    -- Convert to Snacks items
    local items = {}
    for _, dir in ipairs(dirs) do
      table.insert(items, {
        text = dir:gsub("^./", ""), -- what shows in the list
        value = dir:gsub("/$", ""), -- value for confirm()
      })
    end
    return items
  end
  Snacks.picker.pick {
    title = string.format("Save file '%s' to :", file_name),
    format = "text",
    layout = { preset = "vscode" },
    finder = finder,
    confirm = function(picker, item)
      picker:close()
      local dir_path
      if item and item.value then
        -- Existing directory selection
        dir_path = vim.fn.fnamemodify(item.value, ":p")
      else
        -- If user typed a custom path
        local typed_path = picker.input.filter.pattern or "" -- picker.input is the typed text
        if typed_path == "" then return end
        dir_path = vim.fn.fnamemodify(typed_path, ":p")

        dir_path = dir_path:gsub("/$", "") .. "/"
      end
      local file_path = dir_path .. file_name
      save_to_file(file_path)
    end,
  }
end

function M.save_file(opts)
  opts = opts or {}
  Snacks.input({
    prompt = "Enter file name: ",
    completion = "file",
  }, function(input)
    if not input or input == "" then return end

    -- Add default extension if none provided
    if not input:match "%..+$" then
      local buf_ft = vim.bo.filetype
      if buf_ft ~= "" then input = input .. "." .. buf_ft end
    end

    create_snacks_picker(input, opts)
  end)
end

return M
