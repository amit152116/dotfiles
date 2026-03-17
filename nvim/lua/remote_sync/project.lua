---
--- Per-project configuration management
--- Handles loading/saving TOML config files and hostname history
---

local config = require "remote_sync.config"
local logger = require "remote_sync.logger"
local utils = require "remote_sync.utils"

local M = {}

-- Constants
local MAX_HOSTNAME_HISTORY = 10

---@class RemoteSyncProjectConfig
---@field remote_addresses string[] List of remote addresses to try
---@field remote_base_path string Base path on remote machine
---@field local_base_path string Base path on local machine (auto-set)
---@field ssh_key string|nil Path to SSH private key (optional, overrides ~/.ssh/config)
---@field ignorefile_paths string[]|nil Gitignore-style files for exclusions
---@field remote_includes string|string[]|nil Files to include even if ignored
---@field sync_on_save boolean|nil Auto-sync on save (overrides global)
---@field auto_create_dirs boolean Create directories on remote
---@field show_diff_on_receive boolean Show diff before receiving
---@field last_working_address string|nil Last address that worked

-- Cache for loaded project configs
---@type table<string, RemoteSyncProjectConfig>
local project_cache = {}

-- Parser limits to prevent DoS
local MAX_CONFIG_SIZE = 100000 -- 100KB
local MAX_LINE_COUNT = 1000
local MAX_LINE_LENGTH = 1000
local MAX_KEY_LENGTH = 64
local MAX_VALUE_LENGTH = 4096
local MAX_ARRAY_ITEMS = 100

--- Validate a TOML key name
---@param key string
---@return boolean valid
local function is_valid_key(key)
  if not key or #key == 0 or #key > MAX_KEY_LENGTH then return false end
  -- Only allow alphanumeric and underscore
  return key:match "^[%w_]+$" ~= nil
end

--- Simple TOML parser for our config format
--- Handles strings, booleans, and inline/multiline string arrays
--- Includes validation to prevent DoS and injection attacks
---@param content string TOML file content
---@return table|nil config Parsed configuration or nil on error
---@return string|nil error Error message if parsing failed
local function parse_toml(content)
  -- Validate content size
  if not content then return nil, "Empty content" end
  if #content > MAX_CONFIG_SIZE then return nil, "Config file too large" end

  local result = {}
  local in_multiline_array = false
  local array_key = nil
  local array_values = {}
  local line_count = 0

  for line in content:gmatch "[^\r\n]+" do
    line_count = line_count + 1
    if line_count > MAX_LINE_COUNT then
      return nil, "Too many lines in config"
    end

    -- Validate line length before processing
    if #line > MAX_LINE_LENGTH then
      return nil, string.format("Line %d too long", line_count)
    end

    line = vim.trim(line)

    -- Skip comments and empty lines
    if line == "" or line:match "^#" then
      -- skip
    elseif in_multiline_array then
      -- Collecting multiline array values
      if line:match "%]%s*$" then
        -- Last line of array
        for val in line:gmatch '"([^"]*)"' do
          if #array_values >= MAX_ARRAY_ITEMS then
            return nil, "Too many array items"
          end
          if #val > MAX_VALUE_LENGTH then return nil, "Array value too long" end
          table.insert(array_values, val)
        end
        result[array_key] = array_values
        in_multiline_array = false
        array_key = nil
        array_values = {}
      else
        -- Middle line
        for val in line:gmatch '"([^"]*)"' do
          if #array_values >= MAX_ARRAY_ITEMS then
            return nil, "Too many array items"
          end
          if #val > MAX_VALUE_LENGTH then return nil, "Array value too long" end
          table.insert(array_values, val)
        end
      end
    else
      -- Handle key-value pairs
      local key, value = line:match "^([%w_]+)%s*=%s*(.+)$"
      if key and value then
        -- Validate key
        if not is_valid_key(key) then
          return nil, string.format("Invalid key on line %d", line_count)
        end

        value = vim.trim(value)

        -- Check for array
        if value:match "^%[" then
          if value:match "%]$" then
            -- Inline array: key = ["a", "b", "c"]
            local arr = {}
            for val in value:gmatch '"([^"]*)"' do
              if #arr >= MAX_ARRAY_ITEMS then
                return nil, "Too many array items"
              end
              if #val > MAX_VALUE_LENGTH then
                return nil, "Array value too long"
              end
              table.insert(arr, val)
            end
            result[key] = arr
          else
            -- Multiline array start
            in_multiline_array = true
            array_key = key
            array_values = {}
            for val in value:gmatch '"([^"]*)"' do
              if #array_values >= MAX_ARRAY_ITEMS then
                return nil, "Too many array items"
              end
              if #val > MAX_VALUE_LENGTH then
                return nil, "Array value too long"
              end
              table.insert(array_values, val)
            end
          end
        elseif value == "true" then
          result[key] = true
        elseif value == "false" then
          result[key] = false
        elseif value:match '^".*"$' then
          local str_val = value:match '^"(.-)"$'
          if #str_val > MAX_VALUE_LENGTH then
            return nil, string.format("Value too long on line %d", line_count)
          end
          result[key] = str_val
        else
          local num = tonumber(value)
          result[key] = num or value
        end
      end
    end
  end

  -- Check for unclosed multiline array
  if in_multiline_array then return nil, "Unclosed multiline array" end

  return result, nil
end

--- Apply default values from global config to a project config
---@param proj_config RemoteSyncProjectConfig
---@param project_root string
local function apply_defaults(proj_config, project_root)
  proj_config.local_base_path = project_root

  -- Use global settings for ignorefile_paths if not specified per-project
  if not proj_config.ignorefile_paths or #proj_config.ignorefile_paths == 0 then
    proj_config.ignorefile_paths = config.get_value "ignorefile_paths"
  end

  -- Normalize remote_includes to array and apply default
  if proj_config.remote_includes then
    if type(proj_config.remote_includes) == "string" then
      proj_config.remote_includes = { proj_config.remote_includes }
    end
  else
    proj_config.remote_includes = config.get_value "remote_includes" or {}
  end

  -- Use global settings (can be overridden per-project if specified in TOML)
  if proj_config.auto_create_dirs == nil then
    proj_config.auto_create_dirs = config.get_value "auto_create_dirs"
  end
  if proj_config.show_diff_on_receive == nil then
    proj_config.show_diff_on_receive = config.get_value "show_diff_on_receive"
  end
  if proj_config.sync_on_save == nil then
    proj_config.sync_on_save = config.get_value "sync_on_save"
  end
end

--- Convert config table to TOML format
---@param proj_config RemoteSyncProjectConfig
---@return string TOML content
local function to_toml(proj_config)
  local lines = {
    "# Remote Sync Project Configuration",
    "# This file is auto-generated - edit with care",
    "",
  }

  -- Write remote_addresses array
  if proj_config.remote_addresses and #proj_config.remote_addresses > 0 then
    local addr_strings = {}
    for _, addr in ipairs(proj_config.remote_addresses) do
      table.insert(addr_strings, string.format('"%s"', addr))
    end
    table.insert(
      lines,
      string.format("remote_addresses = [%s]", table.concat(addr_strings, ", "))
    )
  end

  -- Write remote_base_path
  if proj_config.remote_base_path then
    table.insert(
      lines,
      string.format('remote_base_path = "%s"', proj_config.remote_base_path)
    )
  end

  -- Write ssh_key if specified (explicit SSH key override)
  if proj_config.ssh_key then
    table.insert(lines, "")
    table.insert(lines, "# SSH private key to use (overrides ~/.ssh/config)")
    table.insert(lines, string.format('ssh_key = "%s"', proj_config.ssh_key))
  end

  table.insert(lines, "")

  -- Write ignorefile_paths if specified (different from global default)
  if proj_config.ignorefile_paths and #proj_config.ignorefile_paths > 0 then
    local ignore_strings = {}
    for _, path in ipairs(proj_config.ignorefile_paths) do
      table.insert(ignore_strings, string.format('"%s"', path))
    end
    table.insert(lines, "# Gitignore-style files for exclusions")
    table.insert(
      lines,
      string.format(
        "ignorefile_paths = [%s]",
        table.concat(ignore_strings, ", ")
      )
    )
  end

  -- Write remote_includes if specified
  local includes = proj_config.remote_includes
  if includes then
    -- Normalize to array
    if type(includes) == "string" then includes = { includes } end
    if #includes > 0 then
      local include_strings = {}
      for _, inc in ipairs(includes) do
        table.insert(include_strings, string.format('"%s"', inc))
      end
      table.insert(lines, "# Files to sync even if matched by ignore patterns")
      table.insert(
        lines,
        string.format(
          "remote_includes = [%s]",
          table.concat(include_strings, ", ")
        )
      )
    end
  end

  -- Write sync_on_save if explicitly set (per-project override)
  if proj_config.sync_on_save ~= nil then
    table.insert(lines, "")
    table.insert(lines, "# Auto-sync on save (overrides global setting)")
    table.insert(
      lines,
      string.format("sync_on_save = %s", tostring(proj_config.sync_on_save))
    )
  end

  -- Write last_working_address if set
  if proj_config.last_working_address then
    table.insert(lines, "")
    table.insert(lines, "# Auto-updated by plugin")
    table.insert(
      lines,
      string.format(
        'last_working_address = "%s"',
        proj_config.last_working_address
      )
    )
  end

  table.insert(lines, "")
  return table.concat(lines, "\n")
end

--- Get the project root directory
---@return string
function M.get_project_root() return vim.fn.getcwd() end

--- Get the config file path for current project
---@return string
function M.get_config_path()
  return M.get_project_root() .. "/" .. config.get_value "project_config_path"
end

--- Check if config exists for current project
---@return boolean
function M.config_exists() return utils.file_exists(M.get_config_path()) end

--- Load project configuration
---@return RemoteSyncProjectConfig|nil
function M.load()
  local config_path = M.get_config_path()
  local project_root = M.get_project_root()

  -- Check cache first
  if project_cache[project_root] then
    logger.debug("project config loaded from cache", { root = project_root })
    return project_cache[project_root]
  end

  logger.debug("loading project config", { path = config_path })

  -- Check if file exists
  if not utils.file_exists(config_path) then return nil end

  -- Read and parse file
  local ok, content = pcall(vim.fn.readfile, config_path)
  if not ok then
    logger.error("failed to read project config", { path = config_path })
    utils.error "Failed to read config file"
    return nil
  end

  local proj_config, parse_err = parse_toml(table.concat(content, "\n"))
  if not proj_config then
    logger.error("TOML parse failed", { path = config_path, error = parse_err })
    utils.error(
      "Failed to parse config file: " .. (parse_err or "invalid TOML")
    )
    return nil
  end

  -- Validate required fields
  if not proj_config.remote_base_path then
    logger.error(
      "project config missing remote_base_path",
      { path = config_path }
    )
    utils.error "Invalid config: missing remote_base_path"
    return nil
  end

  -- Handle backward compatibility (remote_host -> remote_addresses)
  if proj_config.remote_host and not proj_config.remote_addresses then
    proj_config.remote_addresses = { proj_config.remote_host }
    utils.info "Migrated config to multi-address format"
    M.save(proj_config)
  end

  if not proj_config.remote_addresses or #proj_config.remote_addresses == 0 then
    logger.error(
      "project config missing remote_addresses",
      { path = config_path }
    )
    utils.error "Invalid config: missing remote_addresses"
    return nil
  end

  -- Apply defaults from global config
  apply_defaults(proj_config, project_root)

  -- Cache it
  project_cache[project_root] = proj_config

  logger.info("project config loaded", {
    root = project_root,
    addresses = proj_config.remote_addresses,
    remote_base = proj_config.remote_base_path,
  })

  return proj_config
end

--- Save project configuration
---@param proj_config RemoteSyncProjectConfig
---@return boolean success
function M.save(proj_config)
  local config_path = M.get_config_path()
  local project_root = M.get_project_root()

  logger.debug("saving project config", { path = config_path })

  -- Apply defaults from global config
  apply_defaults(proj_config, project_root)

  -- Ensure remote_addresses is a table
  if type(proj_config.remote_addresses) ~= "table" then
    proj_config.remote_addresses = { proj_config.remote_addresses }
  end

  -- Ensure parent directory exists
  local config_dir = utils.dirname(config_path)
  if not utils.dir_exists(config_dir) then vim.fn.mkdir(config_dir, "p") end

  -- Generate TOML content
  local content = to_toml(proj_config)

  -- Write to file
  local ok, err = pcall(vim.fn.writefile, vim.split(content, "\n"), config_path)
  if not ok then
    logger.error(
      "failed to save project config",
      { path = config_path, error = tostring(err) }
    )
    utils.error("Failed to save config: " .. tostring(err))
    return false
  end

  -- Update cache
  project_cache[project_root] = proj_config

  -- Add to .gitignore
  M.add_to_gitignore()

  logger.info("project config saved", { path = config_path })
  utils.info("Config saved to " .. config_path)
  return true
end

--- Reload project configuration (clear cache)
function M.reload()
  local project_root = M.get_project_root()
  logger.debug("reloading project config", { root = project_root })
  project_cache[project_root] = nil
  return M.load()
end

--- Update last working address
---@param address string
function M.update_last_working_address(address)
  local proj_config = M.load()
  if not proj_config then return end

  logger.debug("updating last working address", { address = address })
  proj_config.last_working_address = address
  M.save(proj_config)
end

--- Add config file to .gitignore
function M.add_to_gitignore()
  local gitignore_path = M.get_project_root() .. "/.gitignore"
  local config_filename = config.get_value "project_config_path"

  if not utils.file_exists(gitignore_path) then return end

  local lines = vim.fn.readfile(gitignore_path)

  -- Check if already present
  for _, line in ipairs(lines) do
    if vim.trim(line) == config_filename then return end
  end

  -- Add to .gitignore
  table.insert(lines, "")
  table.insert(lines, "# Remote sync configuration (machine-specific)")
  table.insert(lines, config_filename)

  vim.fn.writefile(lines, gitignore_path)
end

--- Get hostname history
---@return string[]
function M.get_hostname_history()
  local history_path = config.get_hostname_history_path()

  if not utils.file_exists(history_path) then return {} end

  local lines = vim.fn.readfile(history_path)
  local history = {}

  for _, line in ipairs(lines) do
    line = vim.trim(line)
    if line ~= "" then table.insert(history, line) end
  end

  return history
end

--- Add hostname to history
---@param hostname string
function M.add_hostname(hostname)
  if not hostname or hostname == "" then return end

  local history = M.get_hostname_history()

  -- Remove if already exists (will be re-added at top)
  for i, h in ipairs(history) do
    if h == hostname then
      table.remove(history, i)
      break
    end
  end

  -- Add at beginning
  table.insert(history, 1, hostname)

  -- Limit size
  while #history > MAX_HOSTNAME_HISTORY do
    table.remove(history)
  end

  -- Save
  local history_path = config.get_hostname_history_path()
  vim.fn.writefile(history, history_path)
end

--- Get default remote path based on current project
---@return string
function M.get_default_remote_path()
  local cwd = M.get_project_root()
  local home = vim.fn.expand "~"

  -- If cwd is under home, suggest same relative path on remote
  if cwd:sub(1, #home) == home then return cwd end

  return cwd
end

return M
