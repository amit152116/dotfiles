local config = require "diagnostics.config"
local writer = require "diagnostics.writer"

local M = {}

-- profile before stall so stalls can carry stacks from the first one
M.COLLECTORS = {
  "startup",
  "profile",
  "stall",
  "bigfile",
  "lsp",
  "notify",
  "memory",
  "input",
}

local running = {}
local setup_done = false

---@param msg string
local function notify_error(msg)
  vim.notify("[diagnostics] " .. msg, vim.log.levels.ERROR)
end

---@param name string
---@return table|nil
local function load_collector(name)
  local ok, mod = pcall(require, "diagnostics.collectors." .. name)
  return ok and mod or nil
end

---@param name string
function M.stop_collector(name)
  if not running[name] then return end
  running[name] = nil
  local ok, err = pcall(load_collector(name).stop)
  if not ok then
    notify_error(
      string.format("collector %s stop failed: %s", name, tostring(err))
    )
  end
end

---@param name string
---@return fun(fn: function): function
local function make_guard(name)
  return function(fn)
    return function(...)
      local ok, err = pcall(fn, ...)
      if ok then return end
      -- may be a uv callback: stop and notify on the main loop, once
      vim.schedule(function()
        if not running[name] then return end
        M.stop_collector(name)
        notify_error(
          string.format(
            "collector %s disabled after error: %s",
            name,
            tostring(err)
          )
        )
      end)
    end
  end
end

---@param name string
---@return boolean started
function M.start_collector(name)
  if running[name] then return true end
  local mod = load_collector(name)
  if not mod then
    notify_error("unknown collector: " .. name)
    return false
  end
  local ok, err = pcall(mod.start, { guard = make_guard(name) })
  if not ok then
    pcall(mod.stop)
    notify_error(
      string.format("collector %s failed to start: %s", name, tostring(err))
    )
    return false
  end
  running[name] = true
  return true
end

---@param name string
---@return boolean
function M.is_running(name) return running[name] == true end

---@param opts table|nil see diagnostics.config defaults
function M.setup(opts)
  if setup_done then return end
  setup_done = true
  -- early capture ends here whether or not the bigfile collector starts
  require("diagnostics.collectors.bigfile").stop_early()
  for _, msg in ipairs(config.setup(opts)) do
    notify_error(msg)
  end
  local options = config.options
  if not options.enabled or not writer.open(options) then return end

  for _, name in ipairs(M.COLLECTORS) do
    if options.collectors[name] then M.start_collector(name) end
  end
  require("diagnostics.commands").register()

  vim.api.nvim_create_autocmd("VimLeavePre", {
    group = vim.api.nvim_create_augroup("Diagnostics", { clear = true }),
    once = true,
    callback = function()
      for _, name in ipairs(M.COLLECTORS) do
        local mod = running[name] and load_collector(name)
        if mod and mod.on_exit then
          local ok, err = pcall(mod.on_exit)
          if not ok then
            notify_error(
              string.format(
                "collector %s on_exit failed: %s",
                name,
                tostring(err)
              )
            )
          end
        end
      end
      for _, name in ipairs(M.COLLECTORS) do
        M.stop_collector(name)
      end
      writer.shutdown()
    end,
  })
  vim.g.diagnostics_setup_done = true
end

return M
