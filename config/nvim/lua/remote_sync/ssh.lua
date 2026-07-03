---
--- SSH connection handling for remote_sync
--- Connection testing, multi-address fallback, ControlMaster, and key management
---

local config = require "remote_sync.config"
local logger = require "remote_sync.logger"
local utils = require "remote_sync.utils"

local M = {}

-- Constants
local SSH_BATCH_MODE = "BatchMode=no"

---@class SSHExecOpts
---@field ssh_key string|nil Optional SSH key path
---@field use_test_opts boolean|nil Use test opts (BatchMode) instead of control opts
---@field capture_stdout boolean|nil Capture stdout output
---@field capture_stderr boolean|nil Capture stderr output

--- Create a temporary askpass script that prompts via the live Neovim instance.
--- SSH calls this script with the prompt string as $1 when it needs credentials.
--- The script writes the prompt to a temp file and evaluates inputsecret() in
--- the running Neovim via --remote-expr, printing the result to stdout so SSH
--- can read it as the password/passphrase.
---@return string|nil script_path Path to the executable script, or nil on error
---@return string|nil tmpdir Path to the temp directory to clean up, or nil
local function make_askpass_script()
  local tmpdir = vim.fn.tempname()
  if vim.fn.mkdir(tmpdir, "p") ~= 1 then
    logger.error("askpass: failed to create tmpdir", { path = tmpdir })
    return nil, nil
  end

  local script_path = tmpdir .. "/askpass.sh"
  local prompt_file = tmpdir .. "/prompt.txt"
  local server = vim.v.servername

  -- Write the askpass script.  printf is used (not echo) so no trailing
  -- newline is appended.  readfile() + get() gives a safe fallback label.
  local script_lines = {
    "#!/usr/bin/env bash",
    string.format("printf '%%s' \"$1\" > '%s'", prompt_file),
    string.format(
      "exec nvim --server '%s' --remote-expr"
        .. " \"inputsecret(get(readfile('%s'), 0, 'Password: '))\"",
      server,
      prompt_file
    ),
  }
  local fh = io.open(script_path, "w")
  if not fh then
    logger.error("askpass: failed to write script", { path = script_path })
    return nil, tmpdir
  end
  fh:write(table.concat(script_lines, "\n") .. "\n")
  fh:close()

  -- Make the script executable.
  if
    vim.fn.system { "chmod", "+x", script_path } and vim.v.shell_error ~= 0
  then
    logger.error("askpass: chmod failed", { path = script_path })
    return nil, tmpdir
  end

  return script_path, tmpdir
end

--- Execute SSH command on remote host (internal helper)
---@param host string Remote host (user@host or configured hostname)
---@param remote_cmd string[] Command and arguments to run on remote
---@param opts SSHExecOpts|nil Options
---@param callback fun(success: boolean, stdout: string|nil, stderr: string|nil)
local function exec_ssh(host, remote_cmd, opts, callback)
  opts = opts or {}

  -- Build SSH command
  local cmd = { "ssh" }
  if opts.use_test_opts then
    vim.list_extend(cmd, M.get_test_opts(opts.ssh_key))
  else
    vim.list_extend(cmd, M.get_control_opts(opts.ssh_key))
  end
  table.insert(cmd, host)
  vim.list_extend(cmd, remote_cmd)

  logger.debug("ssh exec", { host = host, cmd = table.concat(remote_cmd, " ") })

  -- Create a per-invocation askpass script so SSH can request credentials
  -- interactively via a Neovim inputsecret() prompt without needing a PTY.
  -- SSH_ASKPASS_REQUIRE=force (OpenSSH ≥8.4) makes SSH always use the askpass
  -- program regardless of whether a terminal is attached.
  local askpass_script, askpass_tmpdir = make_askpass_script()

  local job_env = {
    DISPLAY = os.getenv "DISPLAY" or ":1",
  }
  if askpass_script then
    job_env.SSH_ASKPASS = askpass_script
    job_env.SSH_ASKPASS_REQUIRE = "force"
  end

  local stdout_lines = {}
  local stderr_lines = {}

  vim.fn.jobstart(cmd, {
    stdout_buffered = true,
    stderr_buffered = true,
    env = job_env,
    on_stdout = function(_, data)
      if data and opts.capture_stdout then stdout_lines = data end
    end,
    on_stderr = function(_, data)
      if data and opts.capture_stderr then stderr_lines = data end
    end,
    on_exit = vim.schedule_wrap(function(_, code)
      -- Clean up the temp askpass directory.
      if askpass_tmpdir then vim.fn.delete(askpass_tmpdir, "rf") end

      local stdout = nil
      if opts.capture_stdout then
        -- jobstart with stdout_buffered delivers a trailing empty string as
        -- an EOF sentinel; strip it so callers get clean content.
        while #stdout_lines > 0 and stdout_lines[#stdout_lines] == "" do
          table.remove(stdout_lines)
        end
        stdout = table.concat(stdout_lines, "\n")
      end

      local stderr = nil
      if opts.capture_stderr then
        while #stderr_lines > 0 and stderr_lines[#stderr_lines] == "" do
          table.remove(stderr_lines)
        end
        stderr = vim.trim(table.concat(stderr_lines, "\n"))
      end

      callback(code == 0, stdout, stderr)
    end),
  })
end

--- Check SSH key file and warn about permission issues
---@param key_path string Expanded path to SSH key
---@return boolean valid Whether key is usable
local function validate_ssh_key(key_path)
  logger.debug("validating SSH key", { path = key_path })

  -- Check key exists and is readable
  if vim.fn.filereadable(key_path) ~= 1 then
    logger.error("SSH key not found or not readable", { path = key_path })
    vim.notify(
      string.format("SSH key not found or not readable: %s", key_path),
      vim.log.levels.ERROR
    )
    return false
  end

  -- Check permissions (private keys should be 600 or 400)
  local stat = vim.uv.fs_stat(key_path)
  if stat and stat.mode then
    local mode = stat.mode % 512 -- Get last 9 bits (permission bits)
    if mode > 384 then -- 0600 in octal = 384 decimal
      logger.warn("SSH key has overly permissive permissions", {
        path = key_path,
        mode = string.format("%o", mode),
      })
      vim.notify(
        string.format(
          "SSH key %s has overly permissive permissions (should be 600 or 400)",
          key_path
        ),
        vim.log.levels.WARN
      )
    end
  end

  return true
end

--- Get SSH options for ControlMaster connection reuse
---@param ssh_key string|nil Optional explicit SSH key path
---@return string[] SSH command arguments (empty on key validation failure)
function M.get_control_opts(ssh_key)
  local opts = {
    "-o",
    "ControlMaster=auto",
    "-o",
    "ControlPath=" .. config.get_ssh_control_path(),
    "-o",
    "ControlPersist=" .. config.get_value "ssh_control_persist",
  }

  -- Add explicit identity file if specified
  if ssh_key then
    local expanded_key = vim.fn.expand(ssh_key)
    if not validate_ssh_key(expanded_key) then
      return {} -- Fail safely
    end
    table.insert(opts, "-i")
    table.insert(opts, expanded_key)
    -- Use only this key, don't try others
    table.insert(opts, "-o")
    table.insert(opts, "IdentitiesOnly=yes")
  end

  return opts
end

--- Get SSH options for connection testing (with BatchMode)
---@param ssh_key string|nil Optional explicit SSH key path
---@return string[] SSH command arguments (empty on key validation failure)
function M.get_test_opts(ssh_key)
  local timeout = config.get_value "connect_timeout"
  local opts = {
    "-o",
    SSH_BATCH_MODE,
    "-o",
    "ConnectTimeout=" .. timeout,
  }

  -- Add explicit identity file if specified
  if ssh_key then
    local expanded_key = vim.fn.expand(ssh_key)
    if not validate_ssh_key(expanded_key) then
      return {} -- Fail safely
    end
    table.insert(opts, "-i")
    table.insert(opts, expanded_key)
    table.insert(opts, "-o")
    table.insert(opts, "IdentitiesOnly=yes")
  end

  return opts
end

--- Check if ssh-agent is running and has keys
---@param callback fun(agent_running: boolean, loaded_keys: string[])
function M.check_agent_status(callback)
  local ssh_auth_sock = os.getenv "SSH_AUTH_SOCK"
  if not ssh_auth_sock then
    logger.debug "ssh-agent not running (SSH_AUTH_SOCK not set)"
    callback(false, {})
    return
  end

  local lines = {}
  vim.fn.jobstart({ "ssh-add", "-l" }, {
    stdout_buffered = true,
    on_stdout = function(_, data)
      if data then vim.list_extend(lines, data) end
    end,
    on_exit = vim.schedule_wrap(function(_, code)
      local output = table.concat(lines, "\n")

      if output:match "no identities" or output:match "Could not open" then
        logger.debug "ssh-agent running but no keys loaded"
        callback(true, {}) -- Agent running but no keys
        return
      end

      -- ssh-add -l returns exit code 1 when no identities - still means agent is running
      local agent_running = code == 0 or output ~= ""
      local keys = {}
      for line in output:gmatch "[^\r\n]+" do
        -- Extract key path: "256 SHA256:xxx /home/user/.ssh/id_ed25519 (ED25519)"
        local key_path = line:match "([/%w._-]+/%.ssh/[%w._-]+)"
        if key_path then table.insert(keys, key_path) end
      end

      logger.debug(
        "ssh-agent status",
        { running = agent_running, key_count = #keys, keys = keys }
      )

      callback(agent_running, keys)
    end),
  })
end

--- Parse ~/.ssh/config to find a directive value for a host
---@param hostname string The hostname to look up
---@param directive string The directive to find (e.g., "IdentityFile", "User")
---@return string|nil value Value of the directive if found
local function get_config_directive(hostname, directive)
  local ssh_config = vim.fn.expand "~/.ssh/config"
  if not utils.file_exists(ssh_config) then return nil end

  local ok, lines = pcall(vim.fn.readfile, ssh_config)
  if not ok then return nil end

  local in_matching_host = false
  local value = nil

  for _, line in ipairs(lines) do
    line = vim.trim(line)

    -- Skip comments
    if line:match "^#" or line == "" then
      -- continue
    elseif line:match "^Host%s+" then
      -- Check if this Host block matches our hostname
      local hosts = line:match "^Host%s+(.+)$"
      in_matching_host = false
      if hosts then
        for host_pattern in hosts:gmatch "%S+" do
          -- Simple pattern matching (supports * wildcard)
          -- First escape Lua magic characters, then convert * to .*
          local escaped =
            host_pattern:gsub("([%^%$%(%)%%%.%[%]%+%-%?])", "%%%1")
          local pattern = "^" .. escaped:gsub("%*", ".*") .. "$"
          if hostname:match(pattern) then
            in_matching_host = true
            break
          end
        end
      end
    elseif in_matching_host then
      -- Look for directive in matching host block (case-insensitive match)
      local dir_pattern = "^%s*" .. directive .. "%s+(.+)$"
      local dir_value = line:match(dir_pattern)
      if dir_value then
        value = vim.fn.expand(dir_value)
        break
      end
    end
  end

  return value
end

--- Parse ~/.ssh/config to find IdentityFile for a host
---@param hostname string The hostname to look up
---@return string|nil identity_file Path to identity file if found
function M.get_config_identity(hostname)
  return get_config_directive(hostname, "IdentityFile")
end

--- Parse ~/.ssh/config to find Host
---@param hostname string The hostname to look up
---@return string|nil user Username if found
function M.get_config_host(hostname)
  return get_config_directive(hostname, "Host")
end

--- Build detailed error message for connection failure
---@param hostname string The hostname that failed
---@param ssh_key string|nil The SSH key that was used (if any)
---@param ssh_error string|nil The actual SSH error message from stderr
---@param callback fun(message: string)
function M.build_connection_error(hostname, ssh_key, ssh_error, callback)
  local lines = {
    string.format("Failed to connect to %s", hostname),
  }

  -- Include actual SSH error if provided
  if ssh_error and ssh_error ~= "" then
    table.insert(lines, "")
    table.insert(lines, "SSH error: " .. ssh_error)
  end

  callback(table.concat(lines, "\n"))
end

--- Validate SSH connection to a host
---@param hostname string The hostname to test
---@param ssh_key string|nil Optional explicit SSH key
---@param callback fun(success: boolean, message: string)
function M.validate_connection(hostname, ssh_key, callback)
  logger.info("validating SSH connection", { host = hostname, key = ssh_key })
  exec_ssh(hostname, { "echo", "connected" }, {
    ssh_key = ssh_key,
    use_test_opts = true,
    capture_stderr = true,
  }, function(success, _, stderr)
    if success then
      logger.info("SSH connection validated", { host = hostname })
      callback(true, "SSH connection successful")
    else
      logger.error(
        "SSH connection failed",
        { host = hostname, stderr = stderr }
      )
      M.build_connection_error(
        hostname,
        ssh_key,
        stderr,
        function(message) callback(false, message) end
      )
    end
  end)
end

--- Create remote directory
---@param host string Remote host
---@param remote_dir string Directory path on remote
---@param ssh_key string|nil Optional explicit SSH key
---@param callback fun(success: boolean)|nil Optional callback
function M.create_remote_dir(host, remote_dir, ssh_key, callback)
  logger.debug("creating remote directory", { host = host, dir = remote_dir })
  exec_ssh(host, { "mkdir", "-p", remote_dir }, {
    ssh_key = ssh_key,
  }, function(success)
    if success then
      logger.debug(
        "remote directory created",
        { host = host, dir = remote_dir }
      )
    else
      logger.warn(
        "failed to create remote directory",
        { host = host, dir = remote_dir }
      )
    end
    if callback then
      callback(success)
    elseif not success then
      utils.warn("Failed to create remote directory: " .. remote_dir)
    end
  end)
end

--- Get the size of a file on the remote host in bytes.
--- Uses `stat -c %s` (GNU/Linux). Returns nil on failure so callers
--- can fall back to a content-based comparison strategy.
---@param host string Remote host
---@param remote_path string Path on remote
---@param ssh_key string|nil Optional explicit SSH key
---@param callback fun(size: number|nil)
function M.get_remote_file_size(host, remote_path, ssh_key, callback)
  logger.debug("getting remote file size", { host = host, path = remote_path })
  exec_ssh(host, { "stat", "-c", "%s", remote_path }, {
    ssh_key = ssh_key,
    capture_stdout = true,
  }, function(success, stdout)
    if success and stdout and stdout ~= "" then
      local size = tonumber(vim.trim(stdout))
      logger.debug(
        "remote file size",
        { host = host, path = remote_path, size = size }
      )
      callback(size)
    else
      logger.debug(
        "remote file size unavailable",
        { host = host, path = remote_path }
      )
      callback(nil)
    end
  end)
end

--- Get file content from remote (for diff)
---@param host string Remote host
---@param remote_path string Path on remote
---@param ssh_key string|nil Optional explicit SSH key
---@param callback fun(content: string|nil)
function M.get_remote_file(host, remote_path, ssh_key, callback)
  logger.debug("fetching remote file", { host = host, path = remote_path })
  exec_ssh(host, { "cat", remote_path }, {
    ssh_key = ssh_key,
    capture_stdout = true,
  }, function(success, stdout)
    if not success then
      logger.warn(
        "failed to fetch remote file",
        { host = host, path = remote_path }
      )
    end
    callback(success and stdout or nil)
  end)
end

--- Get remote file and compare with local
---@param host string Remote host
---@param remote_path string|nil Path on remote
---@param local_path string Local file to compare
---@param ssh_key string|nil Optional explicit SSH key
---@param callback fun(content: string|nil, is_same: boolean)
function M.get_remote_file_with_comparison(
  host,
  remote_path,
  local_path,
  ssh_key,
  callback
)
  local threshold = config.get_value "diff_size_threshold"

  -- Get local size
  local local_stat = vim.uv.fs_stat(local_path)
  local local_size = local_stat and local_stat.size or nil

  -- Get remote size first
  M.get_remote_file_size(host, remote_path, ssh_key, function(remote_size)
    -- Quick size check
    local skip_compare = false
    local use_large = false

    if local_size and remote_size then
      if local_size ~= remote_size then
        skip_compare = true
      elseif local_size > threshold then
        use_large = true
      end
    end

    -- Fetch content
    M.get_remote_file(host, remote_path, ssh_key, function(content)
      if not content then
        callback(nil, false)
        return
      end

      -- Compare if sizes matched
      local is_same = false
      if not skip_compare then
        if use_large then
          is_same = utils.compare_files_large(local_path, content, local_size)
        else
          is_same = utils.compare_files_small(local_path, content)
        end
      end

      callback(content, is_same)
    end)
  end)
end

--- Test SSH key against a host
---@param hostname string The hostname to test
---@param key_path string Path to SSH private key
---@param callback fun(success: boolean)
function M.test_key(hostname, key_path, callback)
  local expanded_key = vim.fn.expand(key_path)
  logger.debug("testing SSH key", { host = hostname, key = expanded_key })
  if not utils.file_exists(expanded_key) then
    logger.warn(
      "SSH key not found for test",
      { host = hostname, key = expanded_key }
    )
    callback(false)
    return
  end

  exec_ssh(hostname, { "exit", "0" }, {
    ssh_key = expanded_key,
    use_test_opts = true,
  }, function(success)
    logger.debug(
      "SSH key test result",
      { host = hostname, key = expanded_key, success = success }
    )
    callback(success)
  end)
end

--- Kill existing SSH ControlMaster connections
function M.kill_connections()
  logger.debug "killing SSH ControlMaster connections"
  local control_path = config.get_ssh_control_path()
  vim.fn.jobstart({
    "ssh",
    "-O",
    "exit",
    "-o",
    "ControlPath=" .. control_path,
    "dummy_host",
  }, {
    on_exit = function() end, -- Silently ignore errors
  })
end

return M
