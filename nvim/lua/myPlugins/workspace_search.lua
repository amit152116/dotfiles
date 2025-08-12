local M = {}
local action_state = require "telescope.actions.state"
local actions = require "telescope.actions"
local pickers = require "telescope.pickers"
local finders = require "telescope.finders"
local conf = require("telescope.config").values
local previewers = require "telescope.previewers"

-- Create the replace picker (separated for better organization)
function M.create_replace_picker(find_text)
  -- Get matches using ripgrep
  local cmd =
    { "rg", "--vimgrep", "--hidden", "--glob", "!.git", "--", find_text }
  local results = vim.fn.systemlist(cmd)
  if #results == 0 then
    print("No matches found for '" .. find_text .. "'")
    return
  end

  -- Parse matches
  local entries = {}
  for _, result in ipairs(results) do
    local file, line, col, text = result:match "([^:]+):(%d+):(%d+):(.*)"
    if file then
      table.insert(entries, {
        filename = file,
        lnum = tonumber(line),
        col = tonumber(col),
        text = text,
        display = string.format("%s:%s: %s", file, line, text),
      })
    end
  end

  -- Previewer that updates based on current prompt
  local previewer = previewers.new_buffer_previewer {
    title = "Replace Preview",
    define_preview = function(self, entry, _)
      if not entry or not entry.value then return end
      local bufnr = self.state.bufnr

      local ok, lines = pcall(vim.fn.readfile, entry.value.filename)
      if not ok then
        vim.api.nvim_buf_set_lines(
          bufnr,
          0,
          -1,
          false,
          { "Error reading file" }
        )
        return
      end

      local prompt = action_state.get_current_line() -- replacement text
      local preview_lines = {}
      local context_before = math.max(1, entry.value.lnum - 3)
      local context_after = math.min(#lines, entry.value.lnum + 3)

      for i = context_before, context_after do
        local line = lines[i] or ""
        if i == entry.value.lnum then
          table.insert(preview_lines, "- " .. line)
          local replaced_line = line:gsub(M.escape_pattern(find_text), prompt)
          table.insert(preview_lines, "+ " .. replaced_line)
        else
          table.insert(preview_lines, string.format("%3d  %s", i, line))
        end
      end

      vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, preview_lines)
      vim.bo[bufnr].filetype = "diff"
    end,
  }

  pickers
    .new({}, {
      prompt_title = string.format(
        "Replace '%s' → (type replacement)",
        find_text
      ),
      finder = finders.new_table {
        results = entries,
        entry_maker = function(entry)
          return {
            value = entry,
            display = entry.display,
            filename = entry.filename,
            lnum = entry.lnum,
            col = entry.col,
            ordinal = entry.display,
          }
        end,
      },
      sorter = conf.generic_sorter {},
      previewer = previewer,
      attach_mappings = function(prompt_bufnr, map)
        local function do_replace()
          local replace_text = action_state.get_current_line()
          actions.close(prompt_bufnr)
          M.execute_replace(find_text, replace_text, results)
        end

        map("i", "<C-y>", do_replace)
        map("n", "<C-y>", do_replace)
        return true
      end,
      on_input_filter_cb = function(prompt)
        -- This forces live re-render of preview when prompt changes
        return { prompt = prompt }
      end,
    })
    :find()
end

-- Execute the actual replacement
function M.execute_replace(find_text, replace_text, results)
  local confirm = vim.fn.confirm(
    string.format(
      "Replace all %d occurrences of '%s' with '%s'?",
      #results,
      find_text,
      replace_text
    ),
    "&Yes\n&No",
    2
  )

  if confirm ~= 1 then
    print "Replace cancelled"
    return
  end

  -- Set quickfix list and execute replacement
  vim.fn.setqflist({}, " ", { title = "Find & Replace", lines = results })

  local escaped_find = vim.fn.escape(find_text, "/\\")
  local escaped_replace = vim.fn.escape(replace_text, "/\\")

  vim.cmd(
    string.format(
      [[silent! cfdo %%s/%s/%s/ge | update]],
      escaped_find,
      escaped_replace
    )
  )

  print(
    string.format(
      "✓ Replaced %d occurrences of '%s' with '%s'",
      #results,
      find_text,
      replace_text
    )
  )
end

-- Utility function for pattern escaping
function M.escape_pattern(text)
  return text:gsub("[%(%)%.%%%+%-%*%?%[%]%^%$]", "%%%1")
  -- return text:gsub("([^%w])", "%%%1")
end

return M
