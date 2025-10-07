---@class snacks.picker.multigrep.Config : snacks.picker.grep.Config
---@field find_pattern string|nil The text/pattern to find
---@field replace_pattern string|nil The text to replace with
---@field regex boolean|nil Use regex mode (default: true)

local M = {}
local Snacks = require "snacks"

-- Constants
local NAMESPACE_ID = vim.api.nvim_create_namespace "snacks_multigrep_highlight"
local DELIMITER = "#"

-- Directives mapping
local DIRECTIVES = {
  r = "replace",
  g = "glob",
  t = "type",
  x = "exclude",
}

local RG_FLAGS = {
  glob = "-g",
  type = "-t",
  exclude = "-T",
}

---Parse comma/space separated filters
---@param filter_str string|nil
---@return table
local function parse_filters(filter_str)
  if not filter_str or filter_str == "" then return {} end
  return vim.split(filter_str, "[,%s]+", { trimempty = true })
end

---Build base ripgrep arguments
---@param opts snacks.picker.multigrep.Config
---@return table
local function build_base_args(opts)
  local args = {
    "--color=never",
    "--no-heading",
    "--with-filename",
    "--line-number",
    "--column",
    "--smart-case",
    "--no-messages",
  }

  -- Exclude patterns
  for _, pattern in ipairs(opts.exclude or {}) do
    vim.list_extend(args, { "-g", "!" .. pattern })
  end

  -- Hidden files
  if opts.hidden then table.insert(args, "--hidden") end

  -- Ignored files
  if opts.ignored then table.insert(args, "--no-ignore") end

  -- Follow symlinks
  if opts.follow then table.insert(args, "-L") end

  -- Extra arguments
  vim.list_extend(args, opts.args or {})

  return args
end

---Find directive positions in prompt
---@param prompt string
---@return table
local function find_directives(prompt)
  local positions = {}
  for pos, directive in prompt:gmatch "()([rtxg]):" do
    if DIRECTIVES[directive] then
      table.insert(positions, { pos = pos, type = directive })
    end
  end
  table.sort(positions, function(a, b) return a.pos < b.pos end)
  return positions
end

---Extract directive value from prompt
---@param prompt string
---@param current table
---@param next_pos number|nil
---@return string
local function extract_directive_value(prompt, current, next_pos)
  local start_pos = current.pos + 2
  local end_pos = next_pos or (#prompt + 1)
  return vim.trim(prompt:sub(start_pos, end_pos - 1))
end

---Process directives and update args/opts
---@param prompt string
---@param positions table
---@param args table
---@param opts snacks.picker.multigrep.Config
local function process_directives(prompt, positions, args, opts)
  for i, current in ipairs(positions) do
    local next_pos = positions[i + 1] and positions[i + 1].pos
    local value = extract_directive_value(prompt, current, next_pos)

    if current.type == "r" then
      opts.replace_pattern = value
    else
      local flag = RG_FLAGS[DIRECTIVES[current.type]]
      if flag then
        for _, item in ipairs(parse_filters(value)) do
          vim.list_extend(args, { flag, item })
        end
      end
    end
  end
end

---Parse search prompt and build ripgrep arguments
---@param prompt string
---@param opts snacks.picker.multigrep.Config
---@return table|nil
function M.parse_search_prompt(prompt, opts)
  if not prompt or prompt == "" then return nil end

  opts = opts or {}
  opts.find_pattern = nil
  opts.replace_pattern = nil

  local args = build_base_args(opts)

  -- Check for literal string mode
  local is_literal = prompt:sub(1, 2) == "f:"
  if is_literal then
    prompt = vim.trim(prompt:sub(3))
    opts.regex = false
  end

  -- Set search mode
  if is_literal or opts.regex == false then
    table.insert(args, "--fixed-strings")
  else
    table.insert(args, "--auto-hybrid-regex")
  end

  -- Find and process directives
  local positions = find_directives(prompt)

  -- Extract find pattern
  local find_pattern = #positions > 0
      and vim.trim(prompt:sub(1, positions[1].pos - 1))
    or prompt

  opts.find_pattern = find_pattern ~= "" and find_pattern or prompt
  vim.list_extend(args, { "-e", opts.find_pattern })

  if find_pattern == "" then return args end

  -- Process directives
  process_directives(prompt, positions, args, opts)

  return args
end

-- Module-level variable to remember invalid regex
local invalid_regex_notified = false

---Prepare pattern for matching
---@param pattern string
---@param is_regex boolean
---@param ignore_case boolean
---@return string|vim.regex|nil
local function prepare_pattern(pattern, is_regex, ignore_case)
  if is_regex then
    -- User wrote regex - just add \v prefix and case flag
    local vim_pattern = "\\v" .. pattern
    if ignore_case then vim_pattern = "\\c" .. vim_pattern end

    -- Safely attempt to compile regex
    local ok, compiled = pcall(vim.regex, vim_pattern)
    if not ok and not invalid_regex_notified then
      vim.notify("❌ Invalid regex pattern: " .. pattern, vim.log.levels.ERROR)
      invalid_regex_notified = true
    end
    return ok and compiled or nil
  else
    -- Literal mode - return as-is for string.find with plain=true
    return ignore_case and pattern:lower() or pattern
  end
end

---Find match in text
---@param text string
---@param pattern string|vim.regex
---@param is_regex boolean
---@param ignore_case boolean
---@return number|nil start
---@return number|nil end
local function find_match(text, pattern, is_regex, ignore_case)
  if is_regex then
    return pattern:match_str(text)
  else
    local search_text = ignore_case and text:lower() or text
    local start, end_ = search_text:find(pattern, 1, true)
    if start then start = start - 1 end
    return start, end_
  end
end

---Perform regex replacement
---@param text string
---@param find_pattern string
---@param replace_pattern string
---@param ignore_case boolean
---@return boolean success
---@return string|nil result
local function regex_replace(text, find_pattern, replace_pattern, ignore_case)
  -- User wrote the regex pattern - just add \v prefix
  local vim_pattern = "\\v" .. find_pattern
  if ignore_case then vim_pattern = "\\c" .. vim_pattern end

  -- Convert $1, $2 to \1, \2 for Vim
  local vim_replace = replace_pattern:gsub("%$(%d+)", "\\%1")

  return pcall(
    function() return vim.fn.substitute(text, vim_pattern, vim_replace, "") end
  )
end

---Perform literal replacement
---@param text string
---@param start_pos number
---@param end_pos number
---@param replace_pattern string
---@return string
local function literal_replace(text, start_pos, end_pos, replace_pattern)
  local before = text:sub(1, start_pos - 1)
  local after = text:sub(end_pos + 1)
  return before .. replace_pattern .. after
end

---Process find and replace for an item
---@param opts snacks.picker.multigrep.Config
---@param item table
---@param key string|nil
---@return any
local function process_find_replace(opts, item, key)
  local text = item.line
  if not text then return key and rawget(item, key) or true end

  local ignore_case = not opts.find_pattern:match "%u"
  local pattern = prepare_pattern(opts.find_pattern, opts.regex, ignore_case)

  -- ✅ Handle invalid regex pattern (prepare_pattern returned nil)
  if opts.regex and not pattern then
    return key and rawget(item, key) or true
  end

  local start_pos, end_pos = find_match(text, pattern, opts.regex, ignore_case)

  if not start_pos then return key and rawget(item, key) or true end

  -- Store match info
  local match_info = {
    find = { start_col = start_pos, end_col = end_pos },
  }

  -- Perform replacement if requested
  if opts.replace_pattern and opts.replace_pattern ~= "" then
    local success, result

    if opts.regex then
      success, result = regex_replace(
        text,
        opts.find_pattern,
        opts.replace_pattern,
        ignore_case
      )
    else
      start_pos = start_pos + 1
      result = literal_replace(text, start_pos, end_pos, opts.replace_pattern)
      success = true
      start_pos = start_pos - 1
    end

    if success and result ~= text then
      local replace_len = #opts.replace_pattern
      match_info.replace = {
        start_col = start_pos,
        end_col = start_pos + replace_len,
      }
      rawset(item, "replace_line", result)
    end
  end

  rawset(item, "match_info", match_info)
  return key and rawget(item, key) or true
end

---Build quickfix list from items
---@param items table
---@return table
local function build_quickfix_list(items)
  local qf_list = {}
  for _, item in ipairs(items) do
    table.insert(qf_list, {
      filename = Snacks.picker.util.path(item),
      lnum = item.pos and item.pos[1] or 1,
      col = item.pos and item.pos[2] + 1 or 1,
      text = item.line or item.text or "",
    })
  end
  return qf_list
end

---Prepare substitute command
---@param find_pattern string
---@param replace_pattern string
---@param is_regex boolean
---@param ignore_case boolean
---@return string
local function prepare_substitute_cmd(
  find_pattern,
  replace_pattern,
  is_regex,
  ignore_case
)
  local pattern, replacement

  if is_regex then
    -- User wrote regex - just add \v prefix
    pattern = "\\v" .. find_pattern
    if ignore_case then pattern = "\\c" .. pattern end
    replacement = replace_pattern:gsub("%$(%d+)", "\\%1")
  else
    -- Literal mode - escape for \V (very nomagic)
    pattern = "\\V" .. vim.fn.escape(find_pattern, "\\")
    if ignore_case then pattern = "\\c" .. pattern end
    replacement = replace_pattern
  end

  -- Escape delimiter in replacement
  if replacement:find(DELIMITER, 1, true) then
    replacement = replacement:gsub(vim.pesc(DELIMITER), "\\" .. DELIMITER)
  end

  return string.format(
    "silent! cfdo %%s%s%s%s%s%sg | update | stopinsert",
    DELIMITER,
    pattern,
    DELIMITER,
    replacement,
    DELIMITER
  )
end

-- Create a temporary keymap for undoing multi-grep replacements
local function setup_undo_for_qf_replace(qf_list)
  -- Map <Leader>U for undo of this replacement
  vim.keymap.set(
    "n", -- normal mode
    "<Leader>U", -- keybinding
    function()
      -- Ensure all buffers are loaded
      for _, buf in ipairs(qf_list) do
        local bufnr = vim.fn.bufnr(buf.filename)

        if not vim.api.nvim_buf_is_loaded(bufnr) then
          vim.cmd("silent! edit " .. vim.fn.fnameescape(buf.filename))
          bufnr = vim.fn.bufnr(buf.filename) -- get bufnr after loading
          buf.bufnr = bufnr
        end
      end
      vim.cmd "cfdo undo | update | stopinsert"
      vim.notify("✓ Undo of multi-grep replacement done", vim.log.levels.INFO)

      -- Remove the keymap immediately after first use
      vim.keymap.del("n", "<Leader>U")
    end,
    { desc = "Undo Replace", silent = true }
  )

  -- Delete the buffers
  -- for _, qf in ipairs(qf_list) do
  --   if qf.bufnr and vim.api.nvim_buf_is_valid(qf.bufnr) then
  --     vim.api.nvim_buf_delete(qf.bufnr, {})
  --   end
  -- end

  vim.notify(
    "Press <Leader>U to undo this multi-grep replace",
    vim.log.levels.INFO
  )
end
---Execute bulk replacement
---@param items table
---@param opts snacks.picker.multigrep.Config
function M.execute_replace(items, opts)
  if not items or #items == 0 then
    vim.notify(
      "No items to replace",
      vim.log.levels.WARN,
      { title = "Multi-Grep" }
    )
    return
  end

  -- Confirmation
  local confirm_msg = string.format(
    "Replace %d occurrences of '%s' with '%s'?",
    #items,
    opts.find_pattern,
    opts.replace_pattern
  )

  if vim.fn.confirm(confirm_msg, "&Yes\n&No", 2) ~= 1 then
    vim.notify(
      "Replace cancelled",
      vim.log.levels.INFO,
      { title = "Multi-Grep" }
    )
    return
  end

  -- Build quickfix list
  local qf_list = build_quickfix_list(items)
  vim.fn.setqflist({}, "r", { title = "Find & Replace", items = qf_list })

  -- Ensure buffers are loaded before cfdo
  for _, qf in ipairs(qf_list) do
    if qf.filename then
      -- Load the buffer (edit)
      local bufnr = vim.fn.bufnr(qf.filename)
      if not vim.api.nvim_buf_is_loaded(bufnr) then
        vim.cmd("silent! edit " .. vim.fn.fnameescape(qf.filename))
        bufnr = vim.fn.bufnr(qf.filename) -- get bufnr after loading
      end
      qf.bufnr = bufnr
    end
  end

  -- Prepare and execute command
  local ignore_case = not opts.find_pattern:match "%u"
  local sub_cmd = prepare_substitute_cmd(
    opts.find_pattern,
    opts.replace_pattern,
    opts.regex,
    ignore_case
  )

  local success, err = pcall(function() vim.cmd(sub_cmd) end)

  -- Report result
  if success then
    vim.notify(
      string.format("✓ Replaced %d occurrences", #qf_list),
      vim.log.levels.INFO,
      { title = "Multi-Grep" }
    )
    setup_undo_for_qf_replace(qf_list)
  else
    vim.notify(
      string.format("Replace failed: %s", err),
      vim.log.levels.ERROR,
      { title = "Multi-Grep" }
    )
  end
end

---Create finder for picker
---@param opts snacks.picker.multigrep.Config
---@return function
function M.createFinder(opts)
  opts = opts or {}

  return function(picker_opts, ctx)
    local input = ctx.filter.search
    local args = M.parse_search_prompt(input, opts)
    invalid_regex_notified = false

    return require("snacks.picker.source.proc").proc({
      picker_opts,
      {
        cmd = "rg",
        args = args,
        notify = false,
        cwd = opts.cwd,
        transform = function(item)
          item.cwd = opts.cwd

          -- Parse ripgrep output
          local file, lnum, col, text =
            item.text:match "^(.+):(%d+):(%d+):(.*)$"

          if not file then
            if not item.text:match "WARNING" then
              Snacks.notify.error("Invalid grep output:\n" .. item.text)
            end
            return false
          end

          -- Normalize and store
          item.line = text
          item.file = file
          item.pos = { tonumber(lnum), tonumber(col) - 1 }

          -- Schedule replacement processing
          vim.schedule(
            function() process_find_replace(opts, item, "replace_text") end
          )

          return true
        end,
      },
    }, ctx)
  end
end

---Add highlight to buffer
---@param buf number
---@param lnum number
---@param col number
---@param end_col number
---@param hl_group string
---@param priority number
local function add_highlight(buf, lnum, col, end_col, hl_group, priority)
  vim.api.nvim_buf_set_extmark(buf, NAMESPACE_ID, lnum - 1, col, {
    end_col = end_col,
    hl_group = hl_group,
    priority = priority,
  })
end

---Add line highlight to buffer
---@param buf number
---@param lnum number
local function add_line_highlight(buf, lnum)
  vim.api.nvim_buf_set_extmark(buf, NAMESPACE_ID, lnum - 1, 0, {
    hl_group = "Search",
    end_line = lnum,
    priority = 200,
  })
end

---Create preview for picker
---@param ctx snacks.picker.preview.ctx
---@return boolean
function M.createPreview(ctx)
  local item = ctx.item
  if not item or not item.file or not item.pos then return false end

  local buf = ctx.buf
  local filepath = item.cwd .. "/" .. item.file

  -- Read file safely
  local ok, lines = pcall(vim.fn.readfile, filepath)
  if not ok then
    vim.notify("Failed to read file: " .. filepath, vim.log.levels.ERROR)
    return false
  end

  local lnum = item.pos[1]
  local col = item.pos[2]

  -- Handle replacement preview
  if item.replace_line then lines[lnum] = item.replace_line end

  -- Set buffer content
  ctx.preview:set_lines(lines)
  ctx.preview:highlight { file = item.file }

  -- Add match highlighting
  if item.match_info then
    local match = item.replace_line and item.match_info.replace
      or item.match_info.find
    local hl_group = item.replace_line and "Substitute" or "IncSearch"

    add_highlight(buf, lnum, match.start_col, match.end_col, hl_group, 300)
  end

  -- Add line highlight
  add_line_highlight(buf, lnum)

  -- Position cursor
  vim.api.nvim_win_set_cursor(ctx.win, { lnum, col })
  vim.api.nvim_win_call(ctx.win, function() vim.cmd "normal! zz" end)

  return true
end

return M
