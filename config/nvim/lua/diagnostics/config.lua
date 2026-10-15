local M = {}

---@class DiagnosticsThresholds
---@field plugin_ms number
---@field stall_ms number
---@field heartbeat_ms number
---@field bigfile_bytes number
---@field lsp_ms number
---@field lsp_overdue_ms number

---@class DiagnosticsConfig
---@field enabled boolean
---@field log_file string
---@field max_size integer
---@field max_queue integer
---@field flush_ms integer
---@field collectors table<string, boolean>
---@field thresholds DiagnosticsThresholds

---@type DiagnosticsConfig
M.defaults = {
  enabled = true,
  log_file = vim.fn.stdpath "state" .. "/diagnostics.jsonl",
  max_size = 5 * 1024 * 1024,
  max_queue = 2000,
  flush_ms = 1000,
  collectors = {
    startup = true,
    stall = true,
    bigfile = true,
    lsp = true,
    notify = true,
  },
  thresholds = {
    plugin_ms = 5,
    stall_ms = 200,
    heartbeat_ms = 100,
    bigfile_bytes = 1024 * 1024,
    lsp_ms = 100,
    lsp_overdue_ms = 5000,
  },
}

---@type DiagnosticsConfig
M.options = vim.deepcopy(M.defaults)

---@param value any
---@return boolean
local function positive(value) return type(value) == "number" and value > 0 end

--- Merge user opts over defaults; invalid values fall back to the default.
---@param opts table|nil
---@return string[] errors
function M.setup(opts)
  local merged =
    vim.tbl_deep_extend("force", vim.deepcopy(M.defaults), opts or {})
  local errors = {}

  for key, value in pairs(merged.thresholds) do
    local default = M.defaults.thresholds[key]
    if default == nil then
      errors[#errors + 1] = "unknown threshold: " .. key
      merged.thresholds[key] = nil
    elseif not positive(value) then
      errors[#errors + 1] = string.format(
        "thresholds.%s must be a positive number, using %s",
        key,
        default
      )
      merged.thresholds[key] = default
    end
  end

  for name in pairs(merged.collectors) do
    if M.defaults.collectors[name] == nil then
      errors[#errors + 1] = "unknown collector: " .. name
      merged.collectors[name] = nil
    end
  end

  for _, key in ipairs { "max_size", "max_queue", "flush_ms" } do
    if not positive(merged[key]) then
      errors[#errors + 1] = string.format(
        "%s must be a positive number, using %s",
        key,
        M.defaults[key]
      )
      merged[key] = M.defaults[key]
    end
  end

  if type(merged.log_file) ~= "string" or merged.log_file == "" then
    errors[#errors + 1] = "log_file must be a non-empty string, using default"
    merged.log_file = M.defaults.log_file
  end

  M.options = merged
  return errors
end

---@param key string dotted path, e.g. "thresholds.stall_ms"
---@return any
function M.get_value(key)
  return vim.tbl_get(M.options, unpack(vim.split(key, ".", { plain = true })))
end

return M
