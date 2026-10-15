local config = require "diagnostics.config"
local writer = require "diagnostics.writer"
local uv = vim.uv

local M = {}

-- depth 3, functions split by line, full paths, rows >=1%, 1 ms sampling
-- PERF(diagnostics): 1 ms sampling callbacks also run during auto captures, in an already slow session
local JIT_MODE = "3Fplm1i1"

local state
local seq = 0

---@return table|nil
local function snacks_profiler()
  local mod = package.loaded["snacks.profiler"]
  if mod then return mod end
  local ok, loaded = pcall(require, "snacks.profiler")
  return ok and loaded or nil
end

---@return string
-- TODO(diagnostics): profile files are never pruned; keep only the newest N
local function capture_file()
  -- pid + counter: back-to-back captures (same process or two nvims) never collide
  seq = seq + 1
  return string.format(
    "%s/diagnostics-profile-%s-%d-%d.txt",
    vim.fs.dirname(config.options.log_file),
    os.date "%Y%m%d-%H%M%S",
    uv.os_getpid(),
    seq
  )
end

---@return boolean
function M.running() return state ~= nil end

---@return {backend: string, file: string|nil, reason: string, quiet: boolean|nil, elapsed_s: number}|nil
function M.status()
  if not state then return nil end
  return {
    backend = state.backend,
    file = state.file,
    reason = state.reason,
    quiet = state.quiet,
    elapsed_s = (uv.hrtime() - state.start) / 1e9,
  }
end

--- Start a deep profile capture.
---@param opts {backend: "jit"|"snacks"|nil, duration_s: number|nil, reason: string|nil, quiet: boolean|nil}
---@return boolean ok
---@return string|nil err
function M.start(opts)
  opts = opts or {}
  if state then return false, "capture already running" end
  local backend = opts.backend or "jit"
  local s = {
    backend = backend,
    reason = opts.reason or "manual",
    quiet = opts.quiet,
    start = uv.hrtime(),
  }

  if backend == "jit" then
    local diag = require "diagnostics"
    -- jit.p and the stall profiler share the single jit.profile slot
    s.resume_profile = diag.is_running "profile"
    if s.resume_profile then diag.stop_collector "profile" end
    s.file = capture_file()
    local ok, err = pcall(require("jit.p").start, JIT_MODE, s.file)
    if not ok then
      if s.resume_profile then diag.start_collector "profile" end
      return false, "jit.p failed to start: " .. tostring(err)
    end
  elseif backend == "snacks" then
    local profiler = snacks_profiler()
    if not profiler then return false, "snacks.profiler is not available" end
    -- don't adopt a profile the user started; stopping it would be ours to log
    if profiler.running() then
      return false, "snacks profiler already running"
    end
    local ok, err = pcall(profiler.start)
    if not ok then return false, "snacks profiler failed: " .. tostring(err) end
  else
    return false, "unknown backend: " .. tostring(backend)
  end

  state = s
  if opts.duration_s then
    s.timer = uv.new_timer()
    s.timer:start(
      math.floor(opts.duration_s * 1000),
      0,
      vim.schedule_wrap(function()
        if state == s then M.stop() end
      end)
    )
  end
  return true
end

--- Stop the running capture and log it.
---@return boolean ok
---@return string|nil err
function M.stop()
  local s = state
  if not s then return false, "capture not running" end
  state = nil
  if s.timer then
    s.timer:stop()
    s.timer:close()
  end
  local stop_err
  if s.backend == "jit" then
    -- jit.p writes the report on stop; a full disk throws here
    local ok, e = pcall(require("jit.p").stop)
    if not ok then stop_err = tostring(e) end
    -- jit.p's gc finalizer stops whatever capture is current, so collect this
    -- capture's stale proxy now, while nothing is running
    -- PERF(diagnostics): one full gc per capture stop
    collectgarbage()
    if s.resume_profile then
      require("diagnostics").start_collector "profile"
    end
  else
    -- snacks opens its picker on stop; it keeps no file
    local profiler = snacks_profiler()
    if profiler and profiler.running() then profiler.stop() end
  end
  writer.emit("profile_capture", nil, {
    backend = s.backend,
    file = s.file,
    reason = s.reason,
    duration_s = math.floor((uv.hrtime() - s.start) / 1e8) / 10,
    error = stop_err,
  })
  if stop_err then
    vim.notify(
      "[diagnostics] profile capture stop failed: " .. stop_err,
      vim.log.levels.ERROR
    )
  end
  return true
end

--- True while a jit capture holds LuaJIT's single profiler slot.
---@return boolean
function M.owns_profiler() return state ~= nil and state.backend == "jit" end

--- Resume the stall profiler when the current capture stops.
function M.resume_profile_after()
  if state then state.resume_profile = true end
end

--- Profile startup when $DIAG_PROFILE_STARTUP is set; call before lazy setup.
---@return boolean started
function M.startup_from_env()
  local env = vim.env.DIAG_PROFILE_STARTUP
  if not env or env == "" or env == "0" then return false end
  -- NOTE(diagnostics): runs before setup, so the file lands next to the default log_file
  if not M.start { reason = "startup", quiet = true } then return false end
  M.startup_profiled = true
  -- headless never fires UIEnter
  local event = #vim.api.nvim_list_uis() > 0 and "UIEnter" or "VimEnter"
  vim.api.nvim_create_autocmd(event, {
    once = true,
    callback = vim.schedule_wrap(function()
      local file = state and state.reason == "startup" and state.file
      if not file or not M.stop() then return end
      vim.notify("[diagnostics] startup profile: " .. file, vim.log.levels.INFO)
    end),
  })
  return true
end

return M
