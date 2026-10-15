-- minimal assertion runner for headless `nvim -l` test scripts
local T = { failures = 0 }

---@param name string
---@param fn fun()
function T.test(name, fn)
  local ok, err = xpcall(fn, debug.traceback)
  print((ok and "PASS " or "FAIL ") .. name)
  if not ok then
    print(err)
    T.failures = T.failures + 1
  end
end

---@param actual any
---@param expected any
---@param msg string|nil
function T.eq(actual, expected, msg)
  if not vim.deep_equal(actual, expected) then
    error(
      string.format(
        "%s: expected %s, got %s",
        msg or "eq",
        vim.inspect(expected),
        vim.inspect(actual)
      ),
      2
    )
  end
end

---@param value any
---@param msg string|nil
function T.ok(value, msg)
  if not value then error(msg or "expected truthy value", 2) end
end

---@return string
function T.tmpdir()
  local dir = vim.fn.tempname()
  vim.fn.mkdir(dir, "p")
  return dir
end

-- blocks the event loop on purpose; vim.wait would keep processing events
---@param ms number
function T.busy(ms)
  local start = vim.uv.hrtime()
  while (vim.uv.hrtime() - start) / 1e6 < ms do
  end
end

---@param path string
---@return table[]
function T.read_events(path)
  local events = {}
  local f = io.open(path, "r")
  if not f then return events end
  for line in f:lines() do
    events[#events + 1] = vim.json.decode(line)
  end
  f:close()
  return events
end

---@param kind string|nil
---@return table[]
function T.queued(kind)
  local out = {}
  for _, line in ipairs(require("diagnostics.writer").snapshot()) do
    local ev = vim.json.decode(line)
    if not kind or ev.kind == kind then out[#out + 1] = ev end
  end
  return out
end

---@return {msg: any, level: integer|nil}[]
function T.capture_notify()
  local calls = {}
  vim.notify = function(msg, level)
    calls[#calls + 1] = { msg = msg, level = level }
  end
  return calls
end

-- nvim itself may warn (e.g. unwritable nvim.log), so count only our messages
---@param calls {msg: any, level: integer|nil}[]
---@return {msg: any, level: integer|nil}[]
function T.own(calls)
  return vim.tbl_filter(
    function(c)
      return type(c.msg) == "string" and vim.startswith(c.msg, "[diagnostics]")
    end,
    calls
  )
end

---@param overrides table|nil
---@return string log_file
function T.open_writer(overrides)
  local opts = vim.tbl_extend("force", {
    log_file = T.tmpdir() .. "/diagnostics.jsonl",
    max_size = 1024 * 1024,
    max_queue = 1000,
    flush_ms = 60000,
  }, overrides or {})
  assert(require("diagnostics.writer").open(opts), "writer.open failed")
  return opts.log_file
end

function T.done()
  print(T.failures == 0 and "ALL PASS" or (T.failures .. " FAILED"))
  os.exit(T.failures == 0 and 0 or 1)
end

return T
