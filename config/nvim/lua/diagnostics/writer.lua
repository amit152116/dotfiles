local uv = vim.uv

local M = {}

local SCHEMA_VERSION = 1
local RECENT_MAX = 3
local FILE_MODE = 384 -- 0600: log holds local paths and tracebacks

local S

local function reset()
  S = {
    fd = nil,
    path = nil,
    max_size = 0,
    max_queue = 0,
    queue = {},
    in_flight = false,
    paused = false,
    enabled = false,
    dropped = 0,
    bytes = 0,
    recent = {},
    timer = nil,
    sid = string.format("%x-%d", uv.hrtime(), uv.os_getpid()),
  }
end
reset()

---@param msg string
local function notify_error(msg)
  -- may run in a uv callback, where vim.notify is not allowed
  vim.schedule(
    function() vim.notify("[diagnostics] " .. msg, vim.log.levels.ERROR) end
  )
end

---@param path string
---@return integer|nil fd
---@return string|nil err
---@return integer size
local function open_fd(path)
  local fd, err = uv.fs_open(path, "a", FILE_MODE)
  if not fd then return nil, err, 0 end
  local st = uv.fs_fstat(fd)
  return fd, nil, st and st.size or 0
end

---@param err any
local function fail(err)
  S.enabled = false
  notify_error("write failed, logging disabled: " .. tostring(err))
end

---@param state table
---@param n integer
local function drop_front(state, n)
  local q, len = state.queue, #state.queue
  for i = 1, len - n do
    q[i] = q[i + n]
  end
  for i = len - n + 1, len do
    q[i] = nil
  end
end

-- only called between writes, so no write can land in the renamed file
---@return boolean ok
---@return string|nil err
local function rotate()
  uv.fs_close(S.fd)
  S.fd = nil
  uv.fs_rename(S.path, S.path .. ".1")
  local fd, err = open_fd(S.path)
  if not fd then return false, err end
  S.fd, S.bytes = fd, 0
  return true
end

-- several nvims share the log: follow the path if another one rotated it, and
-- cap on the real file size, not just our own bytes (two stats per flush)
---@return boolean ok
---@return string|nil err
local function sync_with_disk()
  local disk, mine = uv.fs_stat(S.path), uv.fs_fstat(S.fd)
  if disk and mine and disk.ino == mine.ino then
    S.bytes = mine.size
    return true
  end
  uv.fs_close(S.fd)
  local fd, err, size = open_fd(S.path)
  if not fd then return false, err end
  S.fd, S.bytes = fd, size
  return true
end

local function wait_idle()
  vim.wait(200, function() return not S.in_flight end, 5)
end

---@param opts {log_file: string, max_size: integer, max_queue: integer, flush_ms: integer}
---@return boolean ok
function M.open(opts)
  if S.fd then return true end
  pcall(vim.fn.mkdir, vim.fs.dirname(opts.log_file), "p")
  local fd, err, size = open_fd(opts.log_file)
  if not fd then
    notify_error("cannot open " .. opts.log_file .. ": " .. tostring(err))
    return false
  end
  S.fd, S.path, S.bytes = fd, opts.log_file, size
  S.max_size, S.max_queue = opts.max_size, opts.max_queue
  S.enabled = true
  S.timer = uv.new_timer()
  S.timer:start(opts.flush_ms, opts.flush_ms, M.flush)
  return true
end

--- Queue one event; cheap and safe from fast-event callbacks.
---@param kind string
---@param ms number|nil
---@param data table|nil
function M.emit(kind, ms, data)
  if #S.queue >= S.max_queue then
    S.dropped = S.dropped + 1
    return
  end
  local sec, usec = uv.gettimeofday()
  local ts = sec + math.floor(usec / 1000) / 1000
  local ok, line = pcall(vim.json.encode, {
    v = SCHEMA_VERSION,
    ts = ts,
    sid = S.sid,
    kind = kind,
    ms = ms and math.floor(ms * 10 + 0.5) / 10,
    data = data,
  })
  if not ok then
    S.dropped = S.dropped + 1
    return
  end
  S.queue[#S.queue + 1] = line .. "\n"
  S.recent[#S.recent + 1] = { kind = kind, ts = ts }
  if #S.recent > RECENT_MAX then table.remove(S.recent, 1) end
end

--- Start one async write of everything queued; no-op if a write is in flight.
function M.flush()
  if not S.enabled or S.paused or S.in_flight or #S.queue == 0 then return end
  local state = S -- callback must not touch state reset by close()
  local n = #state.queue
  local chunk = table.concat(state.queue, "", 1, n)
  local synced, sync_err = sync_with_disk()
  if not synced then return fail(sync_err) end
  -- PERF(diagnostics): rotation is sync fs work but runs once per max_size bytes
  if state.bytes > 0 and state.bytes + #chunk > state.max_size then
    local ok, err = rotate()
    if not ok then return fail(err) end
  end
  state.in_flight = true
  uv.fs_write(state.fd, chunk, -1, function(err, written)
    state.in_flight = false
    if state ~= S then return end
    if err then return fail(err) end
    drop_front(state, n)
    if written < #chunk then
      table.insert(state.queue, 1, chunk:sub(written + 1))
    end
    state.bytes = state.bytes + written
  end)
end

---@return string[]
function M.snapshot() return vim.list_slice(S.queue) end

---@return {kind: string, ts: number}[]
function M.recent() return vim.deepcopy(S.recent) end

---@return {enabled: boolean, path: string|nil, queued: integer, dropped: integer, bytes: integer, in_flight: boolean, sid: string}
function M.status()
  return {
    enabled = S.enabled,
    path = S.path,
    queued = #S.queue,
    dropped = S.dropped,
    bytes = S.bytes,
    in_flight = S.in_flight,
    sid = S.sid,
  }
end

---@return string
function M.sid() return S.sid end

--- Final drain on exit; sync because an async callback would never run.
function M.shutdown()
  if S.timer then S.timer:stop() end
  wait_idle()
  if S.enabled and S.fd and #S.queue > 0 then
    uv.fs_write(S.fd, table.concat(S.queue))
  end
  M.close()
end

--- Drop queued events and truncate both log files.
---@return boolean ok
function M.clear()
  if not S.path then return false end
  S.paused = true
  wait_idle()
  S.queue, S.recent, S.dropped = {}, {}, 0
  if S.fd then
    uv.fs_close(S.fd)
    S.fd = nil
  end
  uv.fs_unlink(S.path)
  uv.fs_unlink(S.path .. ".1")
  local fd, err = open_fd(S.path)
  S.paused = false
  if not fd then
    fail(err)
    return false
  end
  S.fd, S.bytes, S.enabled = fd, 0, true
  return true
end

function M.close()
  if S.timer then
    S.timer:stop()
    S.timer:close()
  end
  if S.fd then uv.fs_close(S.fd) end
  reset()
end

-- test-only
---@return table
function M._state() return S end

return M
