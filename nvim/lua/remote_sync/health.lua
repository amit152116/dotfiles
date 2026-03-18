---
--- Health check for remote_sync
--- Run with :checkhealth remote_sync
---

local M = {}

--- Check if a command is available
---@param cmd string Command to check
---@return boolean available
---@return string|nil version
local function check_command(cmd)
  local found = vim.fn.executable(cmd) == 1
  if not found then return false, nil end

  -- Try to get version
  local version = nil
  if cmd == "rsync" then
    local output = vim.fn.system "rsync --version"
    if vim.v.shell_error == 0 then
      version = output:match "^rsync%s+version%s+([%d%.]+)"
    end
  elseif cmd == "ssh" then
    local output = vim.fn.system "ssh -V 2>&1"
    version = output:match "OpenSSH[_%w]*%s+([%d%.p]+)"
  end

  return true, version
end

--- Main health check function
function M.check()
  vim.health.start "remote_sync"

  -- Check rsync
  local rsync_ok, rsync_version = check_command "rsync"
  if rsync_ok then
    if rsync_version then
      vim.health.ok(string.format("rsync found (v%s)", rsync_version))
    else
      vim.health.ok "rsync found"
    end
  else
    vim.health.error("rsync not found", {
      "Install rsync: sudo apt install rsync",
    })
  end

  -- Check ssh
  local ssh_ok, ssh_version = check_command "ssh"
  if ssh_ok then
    if ssh_version then
      vim.health.ok(string.format("ssh found (OpenSSH %s)", ssh_version))
    else
      vim.health.ok "ssh found"
    end
  else
    vim.health.error("ssh not found", {
      "Install OpenSSH client: sudo apt install openssh-client",
    })
  end

  -- Check ssh-agent and keys (async)
  local ssh = require "remote_sync.ssh"
  local project = require "remote_sync.project"
  local utils = require "remote_sync.utils"

  ssh.check_agent_status(function(agent_running, loaded_keys)
    if agent_running then
      if #loaded_keys > 0 then
        vim.health.ok(
          string.format("ssh-agent running with %d key(s) loaded", #loaded_keys)
        )
        for _, key in ipairs(loaded_keys) do
          vim.health.info("  Key: " .. key)
        end
      else
        vim.health.warn("ssh-agent running but no keys loaded", {
          "Add your SSH key to the agent:",
          "  ssh-add ~/.ssh/id_ed25519",
        })
      end
    else
      vim.health.warn("ssh-agent not running", {
        "Start ssh-agent and add your key:",
        "  eval $(ssh-agent -s)",
        "  ssh-add ~/.ssh/id_ed25519",
      })
    end

    -- Check project configuration
    if project.config_exists() then
      local proj_config = project.load()
      if proj_config then
        vim.health.ok(
          string.format(
            "Project configured: %d remote address(es)",
            #proj_config.remote_addresses
          )
        )

        -- Show addresses
        for _, addr in ipairs(proj_config.remote_addresses) do
          vim.health.info("  Address: " .. addr)
        end

        -- Check SSH key for project
        if proj_config.ssh_key then
          local expanded_key = vim.fn.expand(proj_config.ssh_key)
          if utils.file_exists(expanded_key) then
            vim.health.ok("Project SSH key: " .. proj_config.ssh_key)
          else
            vim.health.error(
              "Project SSH key not found: " .. proj_config.ssh_key,
              {
                "Either create the key or remove ssh_key from config",
              }
            )
          end
        else
          -- Check if host has key in ~/.ssh/config
          local host = proj_config.remote_addresses[1]
          local config_key = ssh.get_config_identity(host)
          if config_key then
            if utils.file_exists(config_key) then
              vim.health.ok("Using key from ~/.ssh/config: " .. config_key)
            else
              vim.health.warn("Key in ~/.ssh/config not found: " .. config_key)
            end
          else
            vim.health.info "Using default SSH key selection (no explicit key configured)"
          end
        end

        -- Show last working address
        if proj_config.last_working_address then
          vim.health.info(
            "Last working address: " .. proj_config.last_working_address
          )
        end
      else
        vim.health.warn "Project config exists but failed to load"
      end
    else
      vim.health.info "No project config (run :RemoteSyncConfigure to set up)"
    end
  end)
end

return M
