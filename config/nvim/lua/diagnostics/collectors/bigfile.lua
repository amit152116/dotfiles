local config = require "diagnostics.config"
local writer = require "diagnostics.writer"
local uv = vim.uv

local M = {}

local GROUP = "Diagnostics_bigfile"
local EARLY_GROUP = "Diagnostics_bigfile_early"
local EARLY_MAX = 50
local ATTACH_DELAY_MS = 500
local LINE_SAMPLE = 1000

local pending = {}
-- reads finished before start(); nil = not capturing, false = already replayed
local early = nil
local early_err

---@param buf integer
local function begin_read(buf)
  local file = vim.api.nvim_buf_get_name(buf)
  if file == "" or vim.bo[buf].buftype ~= "" then return end
  -- NOTE(diagnostics): sync stat; cheap locally, may block on network filesystems
  local st = uv.fs_stat(file)
  pending[buf] = {
    start = uv.hrtime(),
    size = st and st.size,
    size_source = st and "stat" or "unavailable",
  }
end

-- minified files freeze nvim while staying under any byte limit; sample head and tail lines
---@param buf integer
---@param lines integer
---@return integer max_len
---@return boolean sampled
local function max_line_len(buf, lines)
  local sampled = lines > 2 * LINE_SAMPLE
  -- byte offsets, not get_lines: copying a 50 MB minified line is the cost being measured
  -- get_offset counts every EOL as one byte, and the final one only if it would be written
  local bo = vim.bo[buf]
  local final_eol = (bo.eol or (bo.fixeol and not bo.binary)) and 1 or 0
  local offset = vim.api.nvim_buf_get_offset
  local max = 0
  local function measure(from, to)
    local start = offset(buf, from)
    for i = from, to do
      local stop = offset(buf, i + 1)
      local len = stop - start - (i < lines - 1 and 1 or final_eol)
      if len > max then max = len end
      start = stop
    end
  end
  if sampled then
    measure(0, LINE_SAMPLE - 1)
    measure(lines - LINE_SAMPLE, lines - 1)
  else
    measure(0, lines - 1)
  end
  return max, sampled
end

---@param buf integer
---@return {buf: integer, file: string, ms: number, size: integer|nil, size_source: string, lines: integer, max_line_len: integer, line_len_sampled: boolean}|nil
local function finish_read(buf)
  local p = pending[buf]
  pending[buf] = nil
  if not p then return nil end
  local ms = (uv.hrtime() - p.start) / 1e6
  local lines = vim.api.nvim_buf_line_count(buf)
  local max_len, sampled = max_line_len(buf, lines)
  return {
    buf = buf,
    file = vim.api.nvim_buf_get_name(buf),
    ms = ms,
    size = p.size,
    size_source = p.size_source,
    lines = lines,
    max_line_len = max_len,
    line_len_sampled = sampled,
  }
end

---@param buf integer
---@return string[]
local function mitigations(buf)
  local out = {}
  if vim.b[buf].large_buf then out[#out + 1] = "large_buf" end
  -- bigfile.nvim sets 0 on every checked buffer, and 0 is truthy in Lua
  if vim.b[buf].bigfile_detected == 1 then
    out[#out + 1] = "bigfile_detected"
  end
  if vim.bo[buf].filetype == "bigfile" then out[#out + 1] = "ft_bigfile" end
  return out
end

---@param ctx {guard: fun(fn: function): function}
---@param buf integer
---@param file string
local function report_attach(ctx, buf, file)
  vim.defer_fn(
    ctx.guard(function()
      if not vim.api.nvim_buf_is_valid(buf) then return end
      writer.emit("bigfile_attach", nil, {
        buf = buf,
        file = file,
        ft = vim.bo[buf].filetype,
        clients = vim.tbl_map(
          function(c) return c.name end,
          vim.lsp.get_clients { bufnr = buf }
        ),
        -- read-only lookup; get_parser() would start parsing the file being measured
        treesitter = vim.treesitter.highlighter.active[buf] ~= nil,
        mitigations = mitigations(buf),
      })
    end),
    ATTACH_DELAY_MS
  )
end

---@param ctx {guard: fun(fn: function): function}
---@param read table result of finish_read
local function report(ctx, read)
  local min_bytes = config.get_value "thresholds.bigfile_bytes"
  local long_line = read.max_line_len
    >= config.get_value "thresholds.bigfile_line_length"
  if read.size and read.size < min_bytes and not long_line then return end
  writer.emit("bigfile", read.ms, {
    buf = read.buf,
    file = read.file,
    size = read.size,
    size_source = read.size_source,
    lines = read.lines,
    max_line_len = read.max_line_len,
    line_len_sampled = read.line_len_sampled,
  })
  report_attach(ctx, read.buf, read.file)
end

--- Time reads that happen before setup (argv files load before VimEnter).
function M.capture_early()
  if early ~= nil then return end
  early = {}
  local function safe(fn)
    return function(ev)
      local ok, err = pcall(fn, ev)
      if not ok then
        early_err = err
        M.stop_early()
      end
    end
  end
  local group = vim.api.nvim_create_augroup(EARLY_GROUP, { clear = true })
  vim.api.nvim_create_autocmd("BufReadPre", {
    group = group,
    callback = safe(function(ev) begin_read(ev.buf) end),
  })
  vim.api.nvim_create_autocmd("BufReadPost", {
    group = group,
    callback = safe(function(ev)
      local read = finish_read(ev.buf)
      if read and #early < EARLY_MAX then early[#early + 1] = read end
    end),
  })
end

function M.stop_early() pcall(vim.api.nvim_del_augroup_by_name, EARLY_GROUP) end

---@param ctx {guard: fun(fn: function): function}
function M.start(ctx)
  M.stop_early()
  local group = vim.api.nvim_create_augroup(GROUP, { clear = true })

  vim.api.nvim_create_autocmd("BufReadPre", {
    group = group,
    callback = ctx.guard(function(ev) begin_read(ev.buf) end),
  })

  -- NOTE(diagnostics): registered at VimEnter, so read time includes BufReadPost handlers defined earlier
  vim.api.nvim_create_autocmd("BufReadPost", {
    group = group,
    callback = ctx.guard(function(ev)
      local read = finish_read(ev.buf)
      if read then report(ctx, read) end
    end),
  })

  vim.api.nvim_create_autocmd("BufWipeout", {
    group = group,
    callback = function(ev) pending[ev.buf] = nil end,
  })

  if early_err then
    vim.notify(
      "[diagnostics] early big-file capture failed: " .. tostring(early_err),
      vim.log.levels.ERROR
    )
    early_err = nil
  end
  for _, read in ipairs(early or {}) do
    if vim.api.nvim_buf_is_valid(read.buf) then report(ctx, read) end
  end
  early = false
end

function M.stop()
  pcall(vim.api.nvim_del_augroup_by_name, GROUP)
  pending = {}
end

-- test-only
M._max_line_len = max_line_len

return M
