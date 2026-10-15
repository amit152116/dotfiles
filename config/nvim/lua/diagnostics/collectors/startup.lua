local config = require "diagnostics.config"
local writer = require "diagnostics.writer"

local M = {}

local GROUP = "Diagnostics_startup"
local REASON_KEYS =
  { "event", "ft", "cmd", "keys", "require", "plugin", "start", "runtime" }

local times_disabled = false
local recorded = {}

-- overridable for tests; LazyVimStarted never fires without a UI
---@return boolean
function M.headless() return #vim.api.nvim_list_uis() == 0 end

---@param loaded table
---@return string
local function reason_of(loaded)
  for _, key in ipairs(REASON_KEYS) do
    if loaded[key] ~= nil then return key .. ":" .. tostring(loaded[key]) end
  end
  return "?"
end

-- NOTE(diagnostics): reads lazy.nvim private plugin._.loaded; re-check after :Lazy update
---@param plugin table lazy.nvim plugin spec
---@return {name: string, ms: number, reason: string}|nil entry nil when not loaded
---@return boolean shape_ok false when lazy internals no longer match
function M.plugin_time(plugin)
  if type(plugin) ~= "table" or type(plugin._) ~= "table" then
    return nil, false
  end
  local loaded = plugin._.loaded
  if loaded == nil then return nil, true end
  if type(loaded) ~= "table" or type(loaded.time) ~= "number" then
    return nil, false
  end
  return {
    name = plugin.name,
    ms = loaded.time / 1e6,
    reason = reason_of(loaded),
  },
    true
end

local function shape_changed()
  times_disabled = true
  vim.notify(
    "[diagnostics] lazy.nvim plugin data changed shape; per-plugin times disabled",
    vim.log.levels.WARN
  )
end

---@param plugin table
local function record_plugin(plugin)
  if times_disabled then return end
  local entry, shape_ok = M.plugin_time(plugin)
  if not shape_ok then return shape_changed() end
  -- lazy schedules LazyLoad, so it can repeat a load the startup pass saw; a reload changes the time
  if not entry or recorded[entry.name] == entry.ms then return end
  recorded[entry.name] = entry.ms
  if entry.ms > config.get_value "thresholds.plugin_ms" then
    writer.emit(
      "plugin_load",
      entry.ms,
      { name = entry.name, reason = entry.reason }
    )
  end
end

---@param lazy table
local function record_startup(lazy)
  local stats = lazy.stats()
  local available = (stats.startuptime or 0) > 0
  writer.emit("startup", available and stats.startuptime or nil, {
    available = available,
    count = stats.count,
    loaded = stats.loaded,
    times = stats.times,
  })
  for _, plugin in pairs(require("lazy.core.config").plugins) do
    record_plugin(plugin)
  end
end

---@param ctx {guard: fun(fn: function): function}
function M.start(ctx)
  local ok, lazy = pcall(require, "lazy")
  if not ok then return end
  local group = vim.api.nvim_create_augroup(GROUP, { clear = true })
  local started = false
  local on_started = ctx.guard(function()
    started = true
    record_startup(lazy)
  end)

  if (lazy.stats().startuptime or 0) > 0 or M.headless() then
    on_started()
  else
    vim.api.nvim_create_autocmd("User", {
      group = group,
      pattern = "LazyVimStarted",
      once = true,
      callback = on_started,
    })
  end

  vim.api.nvim_create_autocmd("User", {
    group = group,
    pattern = "LazyLoad",
    callback = ctx.guard(function(ev)
      if not started then return end
      local plugin = require("lazy.core.config").plugins[ev.data]
      if plugin then record_plugin(plugin) end
    end),
  })
end

function M.stop() pcall(vim.api.nvim_del_augroup_by_name, GROUP) end

-- test-only
function M._reset() times_disabled = false end

return M
