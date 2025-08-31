local M = {}
local helper = require "utils/helper"
local Snacks = require "snacks"
-- Add custom highlighting
local ns_id = vim.api.nvim_create_namespace "snacks_multigrep_highlight"

--- Performs a bulk replacement on a list of items.
-- @param find_text string The text/pattern to find.
-- @param replace_text string The text to replace with.
-- @param results table A list of snacks.picker.Item objects.
local function execute_replace(items, opts)
  local find_pattern = opts.find_pattern
  local replace_pattern = opts.replace_pattern
  local confirm_msg = string.format(
    "Replace %d occurrences of '%s' with '%s'?",
    #items,
    find_pattern,
    replace_pattern
  )
  local confirm = vim.fn.confirm(confirm_msg, "&Yes\n&No", 2)
  if confirm ~= 1 then
    vim.notify(
      "Replace cancelled.",
      vim.log.levels.INFO,
      { title = "Multi Grep" }
    )
    return
  end

  -- Build quickfix list entries: "filename:lnum:col:text"
  local qf = {}
  for _, item in ipairs(items) do
    qf[#qf + 1] = {
      filename = Snacks.picker.util.path(item),
      lnum = item.pos and item.pos[1] or 1,
      col = item.pos and item.pos[2] + 1 or 1,
      text = item.line or item.text or "",
    }
  end
  vim.fn.setqflist({}, "r", {
    title = "Find & Replace",
    items = qf,
  })

  local delim = "#"
  local ignore_case = not find_pattern:match "%u"
  if opts.regex then
    -- REGEX MODE: Use very magic mode and handle capture groups
    find_pattern = "\\v" .. helper.vim_regex_escape(find_pattern)
    if ignore_case then find_pattern = "\\c" .. find_pattern end

    -- convert `$1`, `$2` ... into `\1`, `\2` for Vim substitute
    replace_pattern = replace_pattern:gsub("%$(%d+)", "\\%1")

    if replace_pattern:find(delim, 1, true) then
      replace_pattern = replace_pattern:gsub(vim.pesc(delim), "\\" .. delim)
    end
  else
    -- LITERAL MODE: Escape everything for literal matching
    find_pattern = "\\V" .. vim.fn.escape(find_pattern, "\\")
    if ignore_case then find_pattern = "\\c" .. find_pattern end
  end

  -- Construct substitute command
  -- Using 'e' flag to suppress "pattern not found" errors
  local sub_cmd = string.format(
    "silent! cfdo %%s%s%s%s%s%sg | update",
    delim, -- opening delimiter
    find_pattern, -- find pattern with \v prefix
    delim, -- middle delimiter
    replace_pattern, -- replace pattern
    delim -- closing delimiter
  )

  -- Execute the substitute command
  local success, err = pcall(function() vim.cmd(sub_cmd) end)
  if success then
    -- Final success message
    vim.notify(
      string.format("✓ Replaced %d occurrences.", #qf),
      vim.log.levels.INFO,
      { title = "Multi Grep" }
    )
  else
    -- Error message
    vim.notify(
      string.format("Replace failed: %s", err),
      vim.log.levels.ERROR,
      { title = "Multi Grep" }
    )
    return
  end
end

-- RECOMMENDED: Space-separated with intuitive keywords
-- Structure: FIND_TERM [r:REPLACE] [g:GLOB] [t:TYPES] [x:EXCLUDE]
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
  return vim.split(filter_str, "[,%s]+", { trimempty = true })
end

local function parse_search_prompt(prompt, opts)
  if not prompt or prompt == "" then return nil end
  opts = opts or {}
  opts.find_pattern = nil
  opts.replace_pattern = nil

  -- Base ripgrep args
  local args = {
    "--color=never",
    "--no-heading",
    "--with-filename",
    "--line-number",
    "--column",
    "--smart-case",
    "--no-messages",
  }

  if opts.regex == false then
    table.insert(args, "--fixed-strings") -- literal search
  else
    table.insert(args, "--auto-hybrid-regex") -- regex search
  end

  if opts.args then vim.list_extend(args, opts.args) end

  -- Find all directive positions
  local positions = {}
  for pos, directive in prompt:gmatch "()([rtxg]):" do
    table.insert(positions, { pos = pos, type = directive })
  end
  table.sort(positions, function(a, b) return a.pos < b.pos end)

  -- Extract find text (from start until first directive)
  local find_pattern = #positions > 0
      and vim.trim(prompt:sub(1, positions[1].pos - 1))
    or prompt

  opts.find_pattern = (find_pattern ~= "" and find_pattern) or prompt

  vim.list_extend(args, { "-e", opts.find_pattern })
  if find_pattern == "" then return args end

  -- Process each directive
  for i, current in ipairs(positions) do
    local start_pos = current.pos + 2
    local end_pos = (positions[i + 1] and positions[i + 1].pos) or (#prompt + 1)
    local value = prompt:sub(start_pos, end_pos - 1)

    if current.type == "r" then
      opts.replace_pattern = value
    else
      local flag = (current.type == "g" and "-g")
        or (current.type == "t" and "-t")
        or (current.type == "x" and "-T")

      if flag then
        for _, ext in ipairs(parse_filters(value)) do
          vim.list_extend(args, { flag, ext })
        end
      end
    end
  end
  return args
end

---comment
---@param opts table
---@param key string
local function process_find_replace(opts, item, key)
  local text = item.line
  local find_pattern = opts.regex and opts.find_pattern or opts.find_pattern
  local s, e
  local ignore_case = not find_pattern:match "%u"

  if opts.regex then
    find_pattern = "\\v" .. helper.vim_regex_escape(find_pattern)
    if ignore_case then find_pattern = "\\c" .. find_pattern end
    local regex = vim.regex(find_pattern)
    s, e = regex:match_str(text)
  else
    if ignore_case then text = text:lower() end
    s, e = text:find(find_pattern, 1, true) -- literal search
  end

  if not s then return true end
  local match_info = {
    find = { start_col = s, end_col = e },
  }

  -- 2. REPLACEMENT (only if requested)
  if opts.replace_pattern and opts.replace_pattern ~= "" then
    local result
    local success = true
    if opts.regex then
      -- convert `$1`, `$2` ... into `\1`, `\2` for Vim substitute
      local replace_pattern = opts.replace_pattern:gsub("%$(%d+)", "\\%1")
      -- Use vim's substitute function for proper capture group handling
      success, result = pcall(
        function()
          return vim.fn.substitute(text, find_pattern, replace_pattern, "")
        end
      )
    else
      -- Fixed: Use string.sub to extract the matched text and replace it
      local before = text:sub(1, s - 1)
      local after = text:sub(e + 1)
      result = before .. opts.replace_pattern .. after
    end

    if success and result ~= text then
      local replace_len = #result - #text + (e - s)
      match_info.replace = {
        start_col = s,
        end_col = s + replace_len,
      }
    end
    rawset(item, "replace_line", result)
  end
  rawset(item, "match_info", match_info)
  if key then return rawget(item, key) end
end

local function createFinder(opts)
  opts = opts or {}

  ---@param picker_opts snacks.picker.proc.Config
  ---@type snacks.picker.finder
  return function(picker_opts, ctx)
    local input = ctx.filter.search
    local args = parse_search_prompt(input, opts)
    return require("snacks.picker.source.proc").proc({
      picker_opts,
      {
        cmd = "rg",
        args = args,
        cwd = opts.cwd,
        transform = function(item)
          if vim.tbl_isempty(item) then return true end

          item.cwd = opts.cwd
          local file, lnum, col, text =
            item.text:match "^(.+):(%d+):(%d+):(.*)$"
          if not file then
            if not item.text:match "WARNING" then
              Snacks.notify.error("invalid grep output:\n" .. item.text)
            end
            return false
          end

          -- Normalize values
          lnum, col = tonumber(lnum), tonumber(col) - 1
          item.line, item.file, item.pos = text, file, { lnum, col }

          vim.schedule(
            function() process_find_replace(opts, item, "replace_text") end
          )
          return true
        end,
      },
    }, ctx)
  end
end
---Preview for the Snacks Picker
---@param ctx snacks.picker.preview.ctx
---@return boolean
local function createPreview(ctx)
  local item = ctx.item

  local buf = ctx.buf
  local lines = vim.fn.readfile(item.file)
  local lnum = item.pos[1]
  local col = item.pos[2]
  -- --- Logic ---
  if item.replace_line then
    -- 1. Create the two new diff lines
    -- local old_line = item.line
    local new_line = item.replace_line

    -- 2. Replace the original line with new diff line in the table
    table.remove(lines, lnum)
    table.insert(lines, lnum, new_line)

    -- 1. Set the buffer content
    ctx.preview:set_lines(lines)

    -- 2. Set the syntax highlighting using the highlight method
    ctx.preview:highlight { file = item.file }

    if item.match_info then
      vim.api.nvim_buf_set_extmark(buf, ns_id, lnum - 1, col, {
        end_col = item.match_info.replace.end_col,
        hl_group = "Substitute",
        priority = 300,
      })
    else
      vim.notify "match_info not present"
    end
  else
    -- 1. Set the buffer content
    ctx.preview:set_lines(lines)

    -- 2. Set the syntax highlighting using the highlight method
    ctx.preview:highlight { file = item.file }

    if item.match_info then
      vim.api.nvim_buf_set_extmark(buf, ns_id, lnum - 1, col, {
        end_col = item.match_info.find.end_col,
        hl_group = "IncSearch",
        priority = 300,
      })
    else
      vim.notify "match_info not present"
    end
  end

  vim.api.nvim_buf_set_extmark(buf, ns_id, lnum - 1, 0, {
    hl_group = "Search",
    end_line = lnum,
    priority = 200,
  })

  -- 3. Manually set cursor and scroll the view to the match
  local winid = ctx.win
  vim.api.nvim_win_set_cursor(winid, { lnum, col })
  vim.api.nvim_win_call(winid, function() vim.cmd "normal! zz" end)
  return true
end

function M.live_multigrep(opts)
  opts = opts or {}
  opts.regex = (opts.regex == nil) and true or opts.regex
  opts.cwd = opts.cwd or vim.uv.cwd()
  if opts.title then
    opts.title = opts.title .. " (Multi-grep)"
  else
    opts.title = "Multi Grep (Smart Hybrid)"
  end

  Snacks.picker.pick("grep", {
    search = opts.search,
    cwd = opts.cwd,
    title = opts.title,
    notify = false,
    finder = createFinder(opts),
    matcher = {
      frecency = true,
      smartcase = true,
    },
    preview = createPreview,
    actions = {
      replace_selected = function(picker, _)
        if opts.replace_pattern then
          local sel = picker:selected()
          local results = #sel > 0 and sel or picker:items()
          if not results or #results == 0 then
            vim.notify(
              "No results to replace.",
              vim.log.levels.WARN,
              { title = "Multi Grep" }
            )
            return
          end
          execute_replace(results, opts)
        end
      end,
    },
    win = {
      input = {
        keys = {
          ["<c-y>"] = {
            "replace_selected",
            mode = { "n", "i" },
            desc = "Replace All",
          },
        },
      },
    },
  })
end
return M
