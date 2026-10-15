local writer = require "diagnostics.writer"

local M = {}

local GROUP = "Diagnostics_notify"
local MAX_MSG = 500
local MAX_TRACE_LINES = 20
local OWN_PREFIX = "[diagnostics]"

local active = false
local busy = false
local guarded
local current, current_inner

---@param msg any
---@param level integer
local function record(msg, level)
  local text = type(msg) == "string" and msg or vim.inspect(msg)
  if vim.startswith(text, OWN_PREFIX) then return end
  local trace = vim.split(debug.traceback("", 2), "\n", { plain = true })
  writer.emit("notify", nil, {
    level = level >= vim.log.levels.ERROR and "ERROR" or "WARN",
    msg = text:sub(1, MAX_MSG),
    trace = table.concat(vim.list_slice(trace, 2, MAX_TRACE_LINES + 1), "\n"),
  })
end

local install

-- each wrapper calls the notify it replaced, never a shared upvalue, so chains can't cycle
---@param inner function
---@return function
local function make_wrapper(inner)
  return function(msg, level, opts)
    if busy or not active then return inner(msg, level, opts) end
    busy = true
    if type(level) == "number" and level >= vim.log.levels.WARN then
      guarded(msg, level)
    end
    local ok, ret = pcall(inner, msg, level, opts)
    busy = false
    -- snacks swaps vim.notify on its first call; wrap whatever replaced us
    if vim.notify ~= current then install() end
    if not ok then error(ret, 0) end
    return ret
  end
end

install = function()
  if not active or vim.notify == current then return end
  current_inner = vim.notify
  current = make_wrapper(current_inner)
  vim.notify = current
end

---@param ctx {guard: fun(fn: function): function}
function M.start(ctx)
  if active then return end
  active = true
  guarded = ctx.guard(record)
  install()
  -- WARN(diagnostics): a plugin replacing vim.notify is only re-wrapped on the next notify or LazyVimStarted
  vim.api.nvim_create_autocmd("User", {
    group = vim.api.nvim_create_augroup(GROUP, { clear = true }),
    pattern = "LazyVimStarted",
    callback = install,
  })
end

function M.stop()
  pcall(vim.api.nvim_del_augroup_by_name, GROUP)
  active = false
  if vim.notify == current then vim.notify = current_inner end
  current, current_inner = nil, nil
end

return M
