local M = {}

local SCHEMA_VERSION = 1
local DEFAULT_SINCE = 86400
local UNITS = { m = 60, h = 3600, d = 86400 }
local TOP_N = 10

---@param fargs string[]
---@return {since: integer|nil, session: "current"|"last"|nil}|nil opts
---@return string|nil err
function M.parse_args(fargs)
  local opts = { since = DEFAULT_SINCE }
  local i = 1
  while i <= #fargs do
    local arg = fargs[i]
    local key, value = arg:match "^%-%-(%w+)=(.+)$"
    if not key then
      key = arg:match "^%-%-(%w+)$"
      i = i + 1
      value = fargs[i]
    end
    if key == "since" then
      if value == "all" then
        opts.since = nil
      else
        local n, unit = (value or ""):match "^(%d+)([mhd])$"
        if not n then
          return nil,
            "invalid --since: "
              .. tostring(value)
              .. " (use 30m, 1h, 1d or all)"
        end
        opts.since = tonumber(n) * UNITS[unit]
      end
    elseif key == "session" then
      if value ~= "current" and value ~= "last" then
        return nil,
          "invalid --session: " .. tostring(value) .. " (use current or last)"
      end
      opts.session = value
    else
      return nil, "unknown argument: " .. arg
    end
    i = i + 1
  end
  return opts
end

--- Read backup then active log, then unflushed lines; bad lines are counted.
---@param path string
---@param extra string[]|nil
---@return table[] events
---@return {skipped: integer} stats
function M.load(path, extra)
  local events, skipped = {}, 0
  local function add(line)
    if line == "" then return end
    local ok, ev = pcall(vim.json.decode, line)
    if
      ok
      and type(ev) == "table"
      and ev.v == SCHEMA_VERSION
      and type(ev.ts) == "number"
      and type(ev.kind) == "string"
      and (ev.ms == nil or type(ev.ms) == "number")
      and (ev.data == nil or type(ev.data) == "table")
    then
      events[#events + 1] = ev
    else
      skipped = skipped + 1
    end
  end
  for _, file in ipairs { path .. ".1", path } do
    local f = io.open(file, "r")
    if f then
      for line in f:lines() do
        add(line)
      end
      f:close()
    end
  end
  for _, line in ipairs(extra or {}) do
    add(vim.trim(line))
  end
  table.sort(events, function(a, b) return a.ts < b.ts end)
  return events, { skipped = skipped }
end

---@param events table[]
---@param opts {since: integer|nil, session: "current"|"last"|nil}
---@param current_sid string
---@return table[]
function M.filter(events, opts, current_sid)
  local cutoff = opts.since and (os.time() - opts.since)
  local sid
  if opts.session == "current" then
    sid = current_sid
  elseif opts.session == "last" then
    for i = #events, 1, -1 do
      if events[i].sid ~= current_sid then
        sid = events[i].sid
        break
      end
    end
    if not sid then return {} end
  end
  return vim.tbl_filter(
    function(ev)
      return (not cutoff or ev.ts >= cutoff) and (not sid or ev.sid == sid)
    end,
    events
  )
end

---@param ev table
---@return table
local function data_of(ev) return type(ev.data) == "table" and ev.data or {} end

---@param ts number
---@return string
local function fmt_time(ts) return os.date("%Y-%m-%d %H:%M:%S", math.floor(ts)) end

---@param size any
---@return string
local function fmt_size(size)
  return type(size) == "number" and string.format("%.1f MB", size / 1048576)
    or "size ?"
end

---@param list table[]
---@param n integer
---@return table[]
local function top(list, n)
  local sorted = vim.list_slice(list)
  table.sort(sorted, function(a, b) return (a.ms or 0) > (b.ms or 0) end)
  return vim.list_slice(sorted, 1, n)
end

---@param events table[] sorted by ts
---@param stats {skipped: integer, dropped: integer|nil}
---@return string[]
function M.render(events, stats)
  local lines = { "# Nvim diagnostics" }
  local function add(fmt, ...)
    lines[#lines + 1] = select("#", ...) > 0 and string.format(fmt, ...) or fmt
  end
  local function header(title, n)
    add ""
    add("## %s (%d)", title, n)
  end

  if #events == 0 then
    add ""
    add "no events yet"
    add(
      "skipped lines: %d, dropped events: %d",
      stats.skipped,
      stats.dropped or 0
    )
    return lines
  end

  local by_kind, sessions = {}, {}
  for _, ev in ipairs(events) do
    by_kind[ev.kind] = by_kind[ev.kind] or {}
    table.insert(by_kind[ev.kind], ev)
    sessions[ev.sid] = true
  end
  local function kind(k) return by_kind[k] or {} end

  header("Summary", #events)
  add("- sessions: %d", vim.tbl_count(sessions))
  add("- range: %s -> %s", fmt_time(events[1].ts), fmt_time(events[#events].ts))
  local kinds = vim.tbl_keys(by_kind)
  table.sort(kinds)
  for _, k in ipairs(kinds) do
    add("- %s: %d", k, #by_kind[k])
  end
  add(
    "- skipped lines: %d, dropped events: %d",
    stats.skipped,
    stats.dropped or 0
  )

  local startups = kind "startup"
  header("Startup", #startups)
  local total, worst, runs = 0, 0, 0
  for _, ev in ipairs(startups) do
    if data_of(ev).available ~= false and ev.ms then
      total, worst, runs = total + ev.ms, math.max(worst, ev.ms), runs + 1
    end
  end
  if runs > 0 then
    add(
      "- startup avg %.1f ms, worst %.1f ms over %d runs",
      total / runs,
      worst,
      runs
    )
  end
  for _, ev in ipairs(top(kind "plugin_load", TOP_N)) do
    local d = data_of(ev)
    add(
      "- %.1f ms  %s  (%s)",
      ev.ms or 0,
      tostring(d.name),
      tostring(d.reason or "?")
    )
  end

  local stalls = kind "stall"
  header("Stalls", #stalls)
  for _, ev in ipairs(top(stalls, TOP_N)) do
    local d = data_of(ev)
    local recent = {}
    for _, r in ipairs(type(d.recent) == "table" and d.recent or {}) do
      recent[#recent + 1] = type(r) == "table" and tostring(r.kind) or "?"
    end
    add(
      "- %s  %.0f ms late  buf=%s ft=%s mode=%s  recent=%s",
      fmt_time(ev.ts),
      ev.ms or 0,
      tostring(d.buf),
      tostring(d.ft),
      tostring(d.mode),
      table.concat(recent, ",")
    )
  end

  local attach = {}
  for _, ev in ipairs(kind "bigfile_attach") do
    attach[ev.sid .. ":" .. tostring(data_of(ev).buf)] = data_of(ev)
  end
  local bigfiles = kind "bigfile"
  header("Big files", #bigfiles)
  for _, ev in ipairs(top(bigfiles, TOP_N)) do
    local d = data_of(ev)
    local a = attach[ev.sid .. ":" .. tostring(d.buf)] or {}
    local clients = type(a.clients) == "table" and table.concat(a.clients, ",")
      or "?"
    add(
      "- %.0f ms  %s  %s  %s lines  ft=%s lsp=%s ts=%s",
      ev.ms or 0,
      tostring(d.file),
      fmt_size(d.size),
      tostring(d.lines or "?"),
      tostring(a.ft or "?"),
      clients == "" and "none" or clients,
      tostring(a.treesitter == nil and "?" or a.treesitter)
    )
  end

  local slow, overdue = kind "lsp", kind "lsp_overdue"
  header("LSP", #slow + #overdue)
  local groups, cancelled = {}, 0
  for _, ev in ipairs(slow) do
    local d = data_of(ev)
    if d.cancelled == true then cancelled = cancelled + 1 end
    local key = tostring(d.client) .. "  " .. tostring(d.method)
    groups[key] = groups[key] or {}
    table.insert(groups[key], ev.ms or 0)
  end
  local rows = {}
  for key, list in pairs(groups) do
    table.sort(list)
    rows[#rows + 1] = {
      key = key,
      n = #list,
      p50 = list[math.ceil(#list / 2)],
      max = list[#list],
    }
  end
  table.sort(rows, function(a, b) return a.max > b.max end)
  for _, r in ipairs(vim.list_slice(rows, 1, TOP_N)) do
    add("- %s  n=%d  p50=%.0f ms  max=%.0f ms", r.key, r.n, r.p50, r.max)
  end
  if cancelled > 0 then add("- cancelled after threshold: %d", cancelled) end
  for _, ev in ipairs(top(overdue, TOP_N)) do
    local d = data_of(ev)
    add(
      "- overdue  %s  %s  pending %.0f ms",
      tostring(d.client),
      tostring(d.method),
      ev.ms or 0
    )
  end

  local notes = kind "notify"
  header("Notifications", #notes)
  local counts, order = {}, {}
  for _, ev in ipairs(notes) do
    local d = data_of(ev)
    local first = vim.split(tostring(d.msg), "\n", { plain = true })[1]
    local key = tostring(d.level) .. "  " .. first
    if not counts[key] then
      counts[key] = 0
      order[#order + 1] = key
    end
    counts[key] = counts[key] + 1
  end
  table.sort(order, function(a, b) return counts[a] > counts[b] end)
  for _, key in ipairs(vim.list_slice(order, 1, TOP_N)) do
    add("- %dx  %s", counts[key], key)
  end

  return lines
end

---@param opts {since: integer|nil, session: "current"|"last"|nil}
function M.open(opts)
  local config = require "diagnostics.config"
  local writer = require "diagnostics.writer"
  local events, stats = M.load(config.options.log_file, writer.snapshot())
  stats.dropped = writer.status().dropped
  local lines = M.render(M.filter(events, opts, writer.sid()), stats)

  vim.cmd "botright new"
  local buf = vim.api.nvim_get_current_buf()
  local win = vim.api.nvim_get_current_win()
  vim.bo[buf].buftype = "nofile"
  vim.bo[buf].bufhidden = "wipe"
  vim.bo[buf].swapfile = false
  vim.api.nvim_buf_set_lines(buf, 0, -1, false, lines)
  vim.bo[buf].modifiable = false
  vim.bo[buf].filetype = "markdown"
  -- after filetype, so ftplugins can't override the per-section folds
  vim.wo[win].foldmethod = "expr"
  vim.wo[win].foldexpr = "getline(v:lnum)=~'^## '?'>1':'='"
  vim.wo[win].foldlevel = 99
end

return M
