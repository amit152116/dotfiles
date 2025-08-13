local M = {}
local action_state = require "telescope.actions.state"
local actions = require "telescope.actions"
local pickers = require "telescope.pickers"
local finders = require "telescope.finders"
local conf = require("telescope.config").values
local previewers = require "telescope.previewers"
local make_entry = require "telescope.make_entry"

-- RECOMMENDED: Space-separated with intuitive keywords
-- Structure: FIND_TERM [r:REPLACE] [t:TYPES] [x:EXCLUDE]
--
-- Examples:
-- "function"                    -> find 'function'
-- "function r:method"          -> find 'function' for replacement with 'method'
-- "function t:lua,js"          -> find 'function' in lua/js files
-- "function t:lua r:method"    -> find 'function' in lua files, replace with 'method'
-- "function x:test,spec"       -> find 'function' excluding test/spec files
-- "function g:src/*"          -> find 'function' in the src glob

local function parse_filters(filter_str)
  if not filter_str or filter_str == "" then return {} end

  local filter_list = {}
  for ext in filter_str:gmatch "[^,]+" do
    ext = vim.trim(ext)
    if ext ~= "" then table.insert(filter_list, ext) end
  end
  return filter_list
end
local query = {}
local function parse_telescope_prompt(prompt, opts)
  if not prompt or prompt == "" then return nil end
  query = {
    find_text = nil,
    replace = nil,
  }
  local args = { "rg" }

  if opts and opts.additional_args then
    vim.list_extend(args, opts.additional_args)
  end

  -- Find all directive positions first
  local positions = {}
  for pos, directive in prompt:gmatch "()([rtxg]):" do
    table.insert(positions, { pos = pos, type = directive })
  end

  -- Extract find term (from start until first directive)
  if #positions > 0 then
    -- Sort by position
    table.sort(positions, function(a, b) return a.pos < b.pos end)
    query.find_text = vim.trim(prompt:sub(1, positions[1].pos - 1))
  else
    query.find_text = vim.trim(prompt)
  end

  vim.list_extend(args, { "-e", query.find_text })

  -- Process each directive
  for i = 1, #positions do
    local current = positions[i]
    local next_pos = positions[i + 1] and positions[i + 1].pos or (#prompt + 1)

    -- Extract value from after "x:" to before next directive
    local directive_start = current.pos + 2 -- skip "x:"
    local value = vim.trim(prompt:sub(directive_start, next_pos - 1))

    if current.type == "r" then
      query.replace_text = value
    elseif current.type == "t" then
      -- Add include types
      for _, ext in ipairs(parse_filters(value)) do
        vim.list_extend(args, { "-t", ext })
      end
    elseif current.type == "g" then
      -- Add include types
      for _, ext in ipairs(parse_filters(value)) do
        vim.list_extend(args, { "-g", ext })
      end
    elseif current.type == "x" then
      -- Add exclude types
      for _, ext in ipairs(parse_filters(value)) do
        vim.list_extend(args, { "-T", ext })
      end
    end
  end

  return vim.tbl_flatten {
    args,
    {
      "--color=never",
      "--no-heading",
      "--with-filename",
      "--line-number",
      "--auto-hybrid-regex",
      "--column",
      "--smart-case",
    },
  }
end

-- Execute the actual replacement
local function execute_replace(find_text, replace_text, results)
  -- Ask for confirmation
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

  -- Build quickfix list entries: "filename:lnum:col:text"
  local qflines = {}
  for _, entry in ipairs(results) do
    -- Ensure we have lnum and col; default to 1 if missing
    local lnum = entry.lnum or 1
    local col = entry.col or 1
    local text = entry.text or entry.value or ""
    table.insert(
      qflines,
      string.format("%s:%d:%d:%s", entry.filename or "", lnum, col, text)
    )
  end
  vim.fn.setqflist({}, " ", {
    title = "Find & Replace",
    lines = qflines,
  })

  -- Escape Lua quotes and backslashes for the Ex command
  local escaped_find = find_text
    :gsub("\\", "\\\\") -- double all backslashes
    :gsub("'", "\\'") -- escape single quotes
  local escaped_replace = replace_text:gsub("'", "\\'")

  -- Construct substitute command in very-magic mode (\v)
  -- 'ge' = global on each line, suppress error if no match
  local sub_cmd = string.format(
    "silent! cfdo %%s/\\v%s/%s/ge | update ",
    escaped_find,
    escaped_replace
  )

  -- Execute it
  vim.cmd(sub_cmd)

  print(
    string.format(
      "✓ Replaced %d occurrences of '%s' with '%s'",
      #results,
      find_text,
      replace_text
    )
  )
end

function M.live_multigrep(opts)
  opts = opts or {}
  opts.cwd = opts.cwd or vim.uv.cwd()
  opts.default_text = opts.default_text or ""
  opts.prompt_title = opts.prompt_title or "Multi Grep (Smart Hybrid)"

  local finder = finders.new_async_job {
    command_generator = function(prompt)
      return parse_telescope_prompt(prompt, opts)
    end,
    entry_maker = make_entry.gen_from_vimgrep(opts),
    cwd = opts.cwd,
  }
  pickers
    .new(opts, {
      debounce = 250,
      prompt_title = opts.prompt_title,
      finder = finder,
      default_text = opts.default_text,
      previewer = conf.grep_previewer(opts),
      sorter = require("telescope.sorters").empty(),
      attach_mappings = function(prompt_bufnr, map)
        local function do_replace()
          local picker = action_state.get_current_picker(prompt_bufnr)
          if not picker then
            vim.notify("No picker found", vim.log.levels.ERROR)
            return
          end

          -- Collect all currently visible/filtered results
          local collected_results = {}
          for entry in picker.manager:iter() do
            table.insert(collected_results, entry)
          end

          if #collected_results == 0 then
            vim.notify("No results to process", vim.log.levels.WARN)
            return
          end

          actions.close(prompt_bufnr)

          -- for _, item in ipairs(collected_results) do
          --   -- Handle display function vs string
          --   local display_text = ""
          --   if type(item.display) == "function" then
          --     display_text = "display_function"
          --   elseif type(item.display) == "string" then
          --     display_text = item.display
          --   else
          --     display_text = "no_display"
          --   end
          --
          --   local info = string.format(
          --     "File: %s | Line: %s | Text: %s | Display: %s",
          --     item.filename or "no_file",
          --     item.lnum or "no_line",
          --     item.text or item.value or "no_content",
          --     display_text
          --   )
          --   vim.notify(info)
          -- end

          -- Execute your replace function
          execute_replace(
            query.find_text,
            query.replace_text,
            collected_results
          )
        end
        map({ "n", "i" }, "<C-y>", do_replace)
        return true
      end,
    })
    :find()
end
M.live_multigrep()
return M
