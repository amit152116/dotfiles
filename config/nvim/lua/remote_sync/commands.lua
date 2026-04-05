---
--- User commands and autocmds for remote_sync
---

local config = require "remote_sync.config"
local project = require "remote_sync.project"
local ssh = require "remote_sync.ssh"
local sync = require "remote_sync.sync"
local utils = require "remote_sync.utils"

local M = {}

-- Augroup for autocmds
local augroup = nil

--- Check if address has valid format (user@host or configured ssh host)
---@param address string
---@return boolean valid
---@return string|nil warning Warning message if format looks problematic
local function validate_address_format(address)
  -- Check if it has user@ prefix
  if address:match "^[^@]+@" then return true, nil end

  -- Check if it's a configured SSH host (has User in ~/.ssh/config)
  local config_user = ssh.get_config_host(address)
  if config_user then return true, nil end

  -- Looks like bare hostname/IP without user
  local warning = string.format(
    "Address '%s' has no user@ prefix and is not in ~/.ssh/config.\n"
      .. "Use format: user@%s or add host to ~/.ssh/config with User directive.",
    address,
    address
  )
  return false, warning
end

--- Prompt user for remote addresses (multi-address support)
---@param callback fun(addresses: string[]|nil)
local function prompt_for_addresses(callback)
  local history = project.get_hostname_history()

  -- Show history as a notification before the input dialog so that
  -- vim.ui.input receives a plain single-line prompt (noice.nvim renders
  -- \n as ^@ control characters inside the input widget).
  if #history > 0 then
    utils.info("Recent addresses: " .. table.concat(history, ", "))
  end

  -- First address
  vim.ui.input(
    { prompt = "Primary address (Host OR user@host/IP): " },
    function(addr1)
      if not addr1 or addr1 == "" then
        callback(nil)
        return
      end

      addr1 = vim.trim(addr1)

      -- Validate address format
      local valid, warning = validate_address_format(addr1)
      if not valid then
        utils.warn(warning)
        callback(nil)
        return
      end

      -- Validate SSH connection
      utils.info("Testing SSH connection to " .. addr1 .. "...")

      ssh.validate_connection(addr1, nil, function(success, message)
        if not success then
          utils.error(message)
          callback(nil)
          return
        end

        utils.info("Connected to " .. addr1)

        -- Ask for additional addresses
        vim.ui.input({
          prompt = "Additional fallback addresses (user@host, comma-separated, or empty): ",
        }, function(additional)
          local addresses = { addr1 }

          if additional and additional ~= "" then
            for addr in additional:gmatch "[^,]+" do
              addr = vim.trim(addr)
              if addr ~= "" and addr ~= addr1 then
                -- Validate format for additional addresses
                local addr_valid, addr_warning = validate_address_format(addr)
                if not addr_valid then
                  utils.warn(addr_warning)
                else
                  table.insert(addresses, addr)
                end
              end
            end

            -- Validate additional addresses (SSH connection test)
            if #addresses > 1 then
              local remaining = #addresses - 1
              local validated = 1

              for i = 2, #addresses do
                ssh.validate_connection(addresses[i], nil, function(ok, _)
                  validated = validated + 1
                  if not ok then
                    utils.warn(
                      "Could not connect to "
                        .. addresses[i]
                        .. " (will keep as fallback)"
                    )
                  end

                  if validated > remaining then callback(addresses) end
                end)
              end
              return
            end
          end

          callback(addresses)
        end)
      end)
    end
  )
end

--- Prompt user to configure remote sync
local function configure()
  utils.info "Starting configuration..."

  prompt_for_addresses(function(addresses)
    if not addresses then return end

    -- Get remote base path
    local default_path = project.get_default_remote_path()

    vim.ui.input({
      prompt = "Remote base path: ",
      default = default_path,
    }, function(remote_path)
      if not remote_path or remote_path == "" then return end

      remote_path = vim.trim(remote_path)

      -- Save configuration
      local proj_config = {
        remote_addresses = addresses,
        remote_base_path = remote_path,
      }

      if project.save(proj_config) then
        -- Add primary address to history
        project.add_hostname(addresses[1])

        utils.info(
          string.format(
            "Configured:\n  Addresses: %s\n  Remote: %s\n  Local: %s",
            table.concat(addresses, ", "),
            remote_path,
            project.get_project_root()
          )
        )
      end
    end)
  end)
end

--- Setup user commands
function M.setup_commands()
  -- Main configure command
  vim.api.nvim_create_user_command(
    "RemoteSyncConfigure",
    function() configure() end,
    { desc = "Configure remote sync for current project" }
  )

  -- Send current file
  vim.api.nvim_create_user_command(
    "RemoteSyncSend",
    function() sync.send_file() end,
    { desc = "Send current file to remote machine" }
  )

  -- Receive current file
  vim.api.nvim_create_user_command(
    "RemoteSyncReceive",
    function() sync.receive_file() end,
    { desc = "Receive current file from remote machine" }
  )

  -- Show diff
  vim.api.nvim_create_user_command(
    "RemoteSyncDiff",
    function() sync.show_diff() end,
    { desc = "Show diff between local and remote file" }
  )

  -- Sync directory up
  vim.api.nvim_create_user_command(
    "RemoteSyncDirUp",
    function() sync.sync_directory "send" end,
    { desc = "Sync entire project directory to remote" }
  )

  -- Sync directory down
  vim.api.nvim_create_user_command(
    "RemoteSyncDirDown",
    function() sync.sync_directory "receive" end,
    { desc = "Sync entire project directory from remote" }
  )

  -- Cancel sync
  vim.api.nvim_create_user_command(
    "RemoteSyncCancel",
    function() sync.cancel() end,
    { desc = "Cancel current sync operation" }
  )

  -- Show/reload config
  vim.api.nvim_create_user_command("RemoteSyncConfig", function(opts)
    local cmd = opts.fargs[1] or "show"

    if cmd == "show" then
      local proj_config = project.load()
      if proj_config then
        utils.info(vim.inspect(proj_config))
      else
        utils.warn "No project config found"
      end
    elseif cmd == "reload" then
      project.reload()
      utils.info "Config reloaded"
    elseif cmd == "global" then
      utils.info(vim.inspect(config.get()))
    else
      utils.error("Unknown subcommand: " .. cmd)
    end
  end, {
    nargs = "?",
    complete = function() return { "show", "reload", "global" } end,
    desc = "Show or reload remote sync config",
  })

  -- Toggle sync on save
  vim.api.nvim_create_user_command("RemoteSyncOnSave", function(opts)
    local cmd = opts.fargs[1] or "toggle"
    local proj_config = project.load()

    -- Helper to get/set sync_on_save
    local function get_current()
      if proj_config then return proj_config.sync_on_save end
      return config.get_value "sync_on_save"
    end

    local function set_value(val)
      if proj_config then
        proj_config.sync_on_save = val
        project.save(proj_config)
      else
        config.set_value("sync_on_save", val)
      end
    end

    if cmd == "enable" then
      set_value(true)
      utils.info "Sync on save enabled"
    elseif cmd == "disable" then
      set_value(false)
      utils.info "Sync on save disabled"
    elseif cmd == "toggle" then
      local current = get_current()
      set_value(not current)
      utils.info("Sync on save " .. (current and "disabled" or "enabled"))
    else
      utils.error("Unknown subcommand: " .. cmd)
    end
  end, {
    nargs = "?",
    complete = function() return { "enable", "disable", "toggle" } end,
    desc = "Toggle sync on save",
  })

  -- Status command
  vim.api.nvim_create_user_command("RemoteSyncStatus", function()
    local state = sync.get_state()
    local proj_config = project.load()

    local status = "Remote Sync Status:\n"
    status = status
      .. "  State: "
      .. (state == sync.State.IDLE and "Idle" or state == sync.State.SYNCING_UP and "Syncing up" or "Syncing down")
      .. "\n"

    if proj_config then
      status = status .. "  Project: " .. project.get_project_root() .. "\n"
      status = status
        .. "  Addresses: "
        .. table.concat(proj_config.remote_addresses, ", ")
        .. "\n"
      if proj_config.ssh_key then
        status = status .. "  SSH key: " .. proj_config.ssh_key .. "\n"
      end
      if proj_config.last_working_address then
        status = status
          .. "  Last working: "
          .. proj_config.last_working_address
          .. "\n"
      end
      status = status
        .. "  Sync on save: "
        .. tostring(proj_config.sync_on_save)
    else
      status = status .. "  Project: Not configured\n"
      status = status
        .. "  Sync on save: "
        .. tostring(config.get_value "sync_on_save")
        .. " (global)"
    end

    utils.info(status)
  end, { desc = "Show remote sync status" })

  -- Test connection command
  vim.api.nvim_create_user_command("RemoteSyncTestConnection", function(opts)
    local hostname = opts.fargs[1]
    local proj_config = project.load()

    -- Get hostname from arg, project config, or prompt
    if not hostname and proj_config and proj_config.remote_addresses then
      hostname = proj_config.last_working_address
        or proj_config.remote_addresses[1]
    end

    if not hostname then
      vim.ui.input({ prompt = "Hostname to test: " }, function(input)
        if input and input ~= "" then
          M.test_connection(
            vim.trim(input),
            proj_config and proj_config.ssh_key
          )
        end
      end)
    else
      M.test_connection(hostname, proj_config and proj_config.ssh_key)
    end
  end, {
    nargs = "?",
    desc = "Test SSH connection to remote",
  })
end

--- Test SSH connection with detailed feedback
---@param hostname string
---@param ssh_key string|nil
function M.test_connection(hostname, ssh_key)
  utils.debug("Testing connection to " .. hostname .. "...")

  ssh.validate_connection(hostname, ssh_key, function(success, message)
    if success then
      utils.info("Successfully connected to " .. hostname)

      -- Show additional info
      local config_key = ssh.get_config_identity(hostname)
      if config_key then
        utils.debug("  Using key from ~/.ssh/config: " .. config_key)
      elseif ssh_key then
        utils.debug("  Using explicit key: " .. ssh_key)
      else
        utils.debug "  Using default SSH key selection"
      end
    else
      utils.error(message)
    end
  end)
end

--- Setup autocmds for sync on save
function M.setup_autocmds()
  -- Create augroup
  if not augroup then
    augroup = vim.api.nvim_create_augroup("remote_sync", { clear = true })
  end

  -- Clear existing autocmds
  vim.api.nvim_clear_autocmds { group = augroup }

  -- Sync on save - checks per-project config which falls back to global
  vim.api.nvim_create_autocmd("BufWritePost", {
    group = augroup,
    callback = function()
      -- Only sync if project is configured
      local proj_config = project.load()
      if proj_config and proj_config.sync_on_save then sync.send_file() end
    end,
  })
end

--- Setup all commands and autocmds
---@param user_augroup number|nil Optional augroup to use
function M.setup(user_augroup)
  if user_augroup then augroup = user_augroup end

  M.setup_commands()
  M.setup_autocmds()
end

return M
