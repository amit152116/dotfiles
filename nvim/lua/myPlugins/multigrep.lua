local M = {}
local action_state = require "telescope.actions.state"
local actions = require "telescope.actions"
local pickers = require "telescope.pickers"
local finders = require "telescope.finders"
local conf = require("telescope.config").values
local previewers = require "telescope.previewers"
local make_entry = require "telescope.make_entry"

-- Custom grep previewer with green highlight on match

local Path = require "plenary.path"
local from_entry = require "telescope.from_entry"

local ns_previewer = vim.api.nvim_create_namespace "telescope.previewers"

-- Green background for matches
vim.api.nvim_set_hl(0, "TelescopePreviewMatch", {
  fg = "#FFFFFF",
  bg = "#005500",
  blend = 25, -- 0 = opaque, 100 = fully transparent
})

vim.api.nvim_set_hl(0, "TelescopePreviewReplace", {
  fg = "#000000",
  bg = "#CC4444", -- dark red  #AA3939
  blend = 25, -- 0 = opaque, 100 = fully transparent
})

local function custom_vimgrep(opts)
  opts = opts or {}
  local cwd = opts.cwd or vim.loop.cwd()
  opts.flag = false

  local jump_to_line = function(self, bufnr, entry)
    -- Cache in previewer state instead of global table
    self._original_lines = self._original_lines or {}

    -- Highlight entire range of matched lines uniformly
    local hl_group = "TelescopePreviewLine"

    -- Compute zero-based start/end lines & columns
    local start_ln = entry.lnum - 1
    local end_ln = (entry.lnend and entry.lnend > entry.lnum)
        and (entry.lnend - 1)
      or start_ln
    local col, colend = 0, -1
    local replace_col, replace_colend = 0, -1

    if entry.col then col = entry.col - 1 end

    for ln = start_ln, end_ln do
      if not self._original_lines[ln] then
        local line_text =
          vim.api.nvim_buf_get_lines(bufnr, ln, ln + 1, false)[1]
        self._original_lines[ln] = line_text
      end

      -- Update from cached original
      local original_line = self._original_lines[ln]

      -- Build a pattern that captures the entire match
      local find_pat
      if opts.use_regex then
        find_pat = "()(" .. opts.find_text .. ")"
      else
        find_pat = "(" .. vim.pesc(opts.find_text) .. ")"
      end

      local replace_pat = ""
      if opts.replace_text == nil or opts.replace_text == "" then
        -- No explicit replace_text: default to the first capture group
        replace_pat = "%1"
      else
        -- User provided replace_text: translate \1 → %1, \2 → %2, etc.
        replace_pat = opts.replace_text:gsub("\\(%d+)", "%%%1")
        hl_group = "TelescopePreviewMatch"
      end

      -- Perform substitution with callback to expand %1, %2, … against captures
      local replace_line = original_line:gsub(
        find_pat,
        function(start, full, ...)
          local caps = { ... }
          -- Expand all %n references in raw_replace
          local rep = replace_pat:gsub(
            "%%(%d+)",
            function(n) return caps[tonumber(n)] or "" end
          )
          -- Compute lengths
          col = start - 1
          colend = col + #full
          replace_col = col
          replace_colend = replace_col + #rep

          return rep
        end
      )

      if opts.replace_text and opts.replace_text ~= "" then
        -- Prefix lines for user-friendly diff-like preview
        local orig_display = "- " .. original_line
        local replace_display = "+ " .. replace_line
        local n = 2
        col = col + n
        colend = colend + n
        replace_col = replace_col + 2
        replace_colend = replace_colend + 2
        opts.flag = true

        -- Update buffer: overwrite the original line and then insert the replaced_line below it
        vim.api.nvim_buf_set_lines(
          bufnr,
          ln, -- start at this line
          ln + 1, -- replace exactly one line
          false, -- strict indexing
          { orig_display } -- restore original (unmodified) line
        )
        vim.api.nvim_buf_set_lines(
          bufnr,
          ln + 1, -- insert after the original
          ln + 2, -- replace one line
          false,
          { replace_display }
        )
      else
        vim.api.nvim_buf_set_lines(
          bufnr,
          ln, -- start at this line
          ln + 1, -- replace exactly one line
          false, -- strict indexing
          { original_line } -- restore original (unmodified) line
        )
        if opts.flag then
          vim.api.nvim_buf_set_lines(
            bufnr,
            ln + 1, -- insert after the original
            ln + 2, -- replace one line
            false,
            {}
          )
        end
      end
    end

    for ln = start_ln, end_ln do
      vim.api.nvim_buf_add_highlight(
        bufnr,
        ns_previewer,
        hl_group,
        ln,
        col,
        colend
      )
    end

    if opts.replace_text and opts.replace_text ~= "" then
      for ln = start_ln, end_ln do
        vim.api.nvim_buf_add_highlight(
          bufnr,
          ns_previewer,
          "TelescopePreviewReplace",
          ln + 1,
          replace_col,
          replace_colend
        )
      end
    end
    -- TelescopePreviewLine

    -- Center the first matched line in the window
    -- Clamp the target line within buffer bounds
    local middle_ln = math.floor(start_ln + (end_ln - start_ln) / 2)
    pcall(vim.api.nvim_win_set_cursor, self.state.winid, { middle_ln + 1, 0 })
    vim.api.nvim_buf_call(bufnr, function() vim.cmd "norm! zz" end)
  end

  return previewers.new_buffer_previewer {
    title = "Grep Preview",
    dyn_title = function(_, entry)
      return Path:new(from_entry.path(entry, false, false)):normalize(cwd)
    end,

    get_buffer_by_name = function(_, entry)
      return from_entry.path(entry, false, false)
    end,

    define_preview = function(self, entry)
      local p = from_entry.path(entry, true, false)
      if not p or p == "" then return end

      conf.buffer_previewer_maker(p, self.state.bufnr, {
        bufname = self.state.bufname,
        winid = self.state.winid,
        preview = opts.preview,
        callback = function(bufnr) jump_to_line(self, bufnr, entry) end,
        file_encoding = opts.file_encoding,
      })
    end,
  }
end

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

local function parse_telescope_prompt(prompt, opts)
  opts.find_text = nil
  opts.replace_text = nil
  opts.use_regex = true
  if not prompt or prompt == "" then return nil end
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
    opts.find_text = vim.trim(prompt:sub(1, positions[1].pos - 1))
  else
    opts.find_text = vim.trim(prompt)
  end

  vim.list_extend(args, { "-e", opts.find_text })

  -- Process each directive
  for i = 1, #positions do
    local current = positions[i]
    local next_pos = positions[i + 1] and positions[i + 1].pos or (#prompt + 1)

    -- Extract value from after "x:" to before next directive
    local directive_start = current.pos + 2 -- skip "x:"
    local value = vim.trim(prompt:sub(directive_start, next_pos - 1))

    if current.type == "r" then
      opts.replace_text = value
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
  if opts.prompt_title then
    opts.prompt_title = opts.prompt_title .. " (Multi-grep)"
  else
    opts.prompt_title = "Multi Grep (Smart Hybrid)"
  end

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
      previewer = custom_vimgrep(opts),
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

          -- Execute your replace function
          execute_replace(opts.find_text, opts.replace_text, collected_results)
        end
        map({ "n", "i" }, "<C-y>", do_replace)
        return true
      end,
    })
    :find()
end
-- M.live_multigrep()
return M
