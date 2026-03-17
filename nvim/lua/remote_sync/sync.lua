---
--- Core sync operations for remote_sync
--- Handles rsync execution, file/directory syncing, and diff preview
---

local config = require "remote_sync.config"
local logger = require "remote_sync.logger"
local project = require "remote_sync.project"
local ssh = require "remote_sync.ssh"
local utils = require "remote_sync.utils"

local M = {}

---@enum SyncState
M.State = {
  IDLE = 0,
  SYNCING_UP = 1,
  SYNCING_DOWN = 2,
}

--- Build prioritized address list (last working first, then others)
---@param proj_config RemoteSyncProjectConfig
---@return string[] addresses
local function build_address_list(proj_config)
  local addresses = {}
  local seen = {}

  if proj_config.last_working_address then
    table.insert(addresses, proj_config.last_working_address)
    seen[proj_config.last_working_address] = true
  end

  for _, addr in ipairs(proj_config.remote_addresses) do
    if not seen[addr] then
      table.insert(addresses, addr)
      seen[addr] = true
    end
  end

  return addresses
end

--- Load project config and get current file path (common validation)
---@param callback fun(success: boolean)|nil Optional callback to invoke on failure
---@return RemoteSyncProjectConfig|nil proj_config
---@return string|nil local_path
local function require_config_and_file(callback)
  local proj_config = project.load()
  if not proj_config then
    utils.error "No remote sync config found. Run :RemoteSyncConfigure"
    if callback then callback(false) end
    return nil, nil
  end

  local local_path = vim.fn.expand "%:p"
  if local_path == "" then
    utils.error "No file open"
    if callback then callback(false) end
    return nil, nil
  end

  return proj_config, local_path
end

-- Current sync state
local state = {
  current = M.State.IDLE,
  job_id = -1,
  last_code = 0,
}

--- Build rsync arguments from project config
---@param proj_config RemoteSyncProjectConfig
---@param project_root string
---@param direction "send"|"receive"
---@return string rsync_opts
local function build_rsync_opts(proj_config, project_root, direction)
  local opts = { "-avzP" }

  -- I/O timeout: kill rsync if no data is transferred for this many seconds.
  -- Handles mid-transfer network hangs more reliably than a Lua-side timer.
  local io_timeout = config.get_value "rsync_io_timeout"
  table.insert(opts, "--timeout=" .. io_timeout)

  -- Add includes FIRST (rsync processes rules in order)
  -- These files will be synced even if they match ignore patterns
  local includes = proj_config.remote_includes or {}
  for _, inc in ipairs(includes) do
    local escaped = utils.shell_escape(inc)
    if escaped then
      table.insert(opts, string.format("--include=%s", escaped))
    end
  end

  -- Add ignore file filters
  local ignorefiles = proj_config.ignorefile_paths or {}
  for _, ignorefile in ipairs(ignorefiles) do
    -- Expand ~ and resolve relative to project root
    local expanded = vim.fn.expand(ignorefile)
    if not expanded:match "^/" then
      expanded = project_root .. "/" .. expanded
    end
    if utils.file_exists(expanded) then
      local escaped = utils.shell_escape(expanded)
      if escaped then
        -- --exclude-from=PATH is a single shell token (path contiguous with =),
        -- so the shell-quoted path is parsed correctly by the shell and rsync.
        -- --filter=:- PATH splits into two tokens after shell word-splitting,
        -- causing rsync to receive --filter=:- (no file) and a stray argument.
        table.insert(opts, string.format("--exclude-from=%s", escaped))
      end
    end
  end

  -- Always exclude .git directory
  table.insert(opts, "--exclude=.git/")

  -- Backup: always enabled so overwritten files are never silently lost.
  table.insert(opts, "--backup")
  if direction == "receive" then
    -- On receive the destination is local, so we can point --backup-dir at a
    -- known local path rather than scattering .bak files through the working tree.
    local backup_dir = config.get_value "backup_dir"
    vim.fn.mkdir(backup_dir, "p") -- idempotent; creates parent dirs as needed
    local escaped_backup_dir = utils.shell_escape(backup_dir)
    if escaped_backup_dir then
      table.insert(opts, "--backup-dir=" .. escaped_backup_dir)
    end
    table.insert(opts, "--suffix=.bak")
  else
    -- On send the destination is remote; --backup-dir would need to be a remote
    -- path, so just use a fixed suffix and let rsync place the backup next to
    -- the original file on the remote.
    table.insert(opts, "--suffix=.bak")
  end

  return table.concat(opts, " ")
end

--- Assemble the final rsync shell command string
---@param src string Source path (may include host: prefix)
---@param dest string Destination path (may include host: prefix)
---@param proj_config RemoteSyncProjectConfig
---@param direction "send"|"receive"
---@return string cmd The complete rsync command
local function build_rsync_cmd(src, dest, proj_config, direction)
  local ssh_control_opts =
    table.concat(ssh.get_control_opts(proj_config.ssh_key), " ")
  local ssh_cmd = string.format("ssh %s", ssh_control_opts)
  local rsync_opts =
    build_rsync_opts(proj_config, proj_config.local_base_path, direction)
  return string.format("rsync %s -e '%s' %s %s", rsync_opts, ssh_cmd, src, dest)
end

--- Get current sync state
---@return SyncState
function M.get_state() return state.current end

--- Check if sync is in progress
---@return boolean
function M.is_busy() return state.current ~= M.State.IDLE end

--- Construct remote path from local path and config
---@param local_path string
---@param proj_config RemoteSyncProjectConfig
---@param address string Remote address to use
---@return string|nil remote_path
local function construct_remote_path(local_path, proj_config, address)
  local relative =
    utils.get_relative_path(local_path, proj_config.local_base_path)

  if not relative then
    utils.error(
      string.format(
        "File is not within project:\n  File: %s\n  Project: %s",
        local_path,
        proj_config.local_base_path
      )
    )
    return nil
  end

  local remote_file = proj_config.remote_base_path .. "/" .. relative
  return utils.make_remote_path(address, remote_file)
end

--- Execute rsync command
---@param direction "send"|"receive"
---@param local_path string
---@param remote_path string
---@param proj_config RemoteSyncProjectConfig
---@param callback fun(success: boolean)|nil
local function execute_rsync(
  direction,
  local_path,
  remote_path,
  proj_config,
  callback
)
  local host, remote_file = utils.parse_remote_path(remote_path)

  if not host or not remote_file then
    utils.error "Invalid remote path format (expected host:path)"
    if callback then callback(false) end
    return
  end

  -- Validate path for security
  local valid, err = utils.validate_path(remote_file)
  if not valid then
    utils.error("Invalid remote path: " .. err)
    if callback then callback(false) end
    return
  end

  -- Escape paths
  local escaped_local = utils.shell_escape(local_path)
  local escaped_remote = utils.shell_escape(remote_file)

  if not escaped_local or not escaped_remote then
    utils.error "Failed to escape paths"
    if callback then callback(false) end
    return
  end

  -- Build src and dest
  local src, dest
  if direction == "send" then
    src = escaped_local
    dest = host .. ":" .. escaped_remote
  else
    src = host .. ":" .. escaped_remote
    dest = escaped_local
  end

  -- Update state and show notification immediately, before any async work.
  -- This guarantees "Sending to / Receiving from" is always the FIRST notification
  -- the user sees, regardless of how quickly mkdir or rsync callbacks fire.
  state.current = direction == "send" and M.State.SYNCING_UP
    or M.State.SYNCING_DOWN
  local start_time = vim.loop.now()

  utils.debug(
    string.format(
      "%s %s %s",
      direction == "send" and "Sending to" or "Receiving from",
      host,
      utils.basename(local_path)
    )
  )

  logger.info("rsync start", {
    direction = direction,
    file = utils.basename(local_path),
    host = host,
  })

  -- Call on_sync_start callback
  local on_start = config.get_value "on_sync_start"
  if on_start then on_start(direction, local_path) end

  -- Hard ceiling timer: covers both mkdir and rsync time.
  local timeout_ms = (config.get_value "rsync_timeout") * 1000
  local timed_out = false
  local timeout_timer = vim.loop.new_timer()

  -- Abort helper: cleans up state, fires callbacks, logs failure.
  local function abort(msg)
    timeout_timer:stop()
    timeout_timer:close()
    state.current = M.State.IDLE
    state.job_id = -1
    local on_complete = config.get_value "on_sync_complete"
    if on_complete then on_complete(direction, local_path, false) end
    if callback then callback(false) end
    if msg then utils.warn(msg) end
  end

  timeout_timer:start(
    timeout_ms,
    0,
    vim.schedule_wrap(function()
      timeout_timer:stop()
      timeout_timer:close()
      if state.job_id > 0 then
        timed_out = true
        vim.fn.jobstop(state.job_id)
        logger.warn("rsync timed out", {
          direction = direction,
          file = utils.basename(local_path),
          timeout_s = timeout_ms / 1000,
        })
        utils.error(
          string.format("Sync timed out after %ds", timeout_ms / 1000)
        )
      end
    end)
  )

  -- Start the rsync job (called once mkdir is confirmed, or immediately when
  -- auto_create_dirs is off or direction is receive).
  local function start_rsync_job()
    local cmd = build_rsync_cmd(src, dest, proj_config, direction)
    local stderr_lines = {}

    state.job_id = vim.fn.jobstart(cmd, {
      -- Collect stderr so we can surface rsync's actual error message.
      on_stderr = vim.schedule_wrap(function(_, data)
        for _, line in ipairs(data) do
          if line ~= "" then table.insert(stderr_lines, line) end
        end
      end),

      on_exit = vim.schedule_wrap(function(_, code)
        if not timed_out then
          timeout_timer:stop()
          timeout_timer:close()
        end

        local elapsed = vim.loop.now() - start_time
        state.current = M.State.IDLE
        state.last_code = code
        state.job_id = -1

        local success = code == 0 and not timed_out

        logger.info("rsync complete", {
          direction = direction,
          file = utils.basename(local_path),
          success = success,
          code = code,
          elapsed_ms = elapsed,
        })

        if success then
          utils.info(
            string.format(
              "%s %s completed (%.1fs)",
              direction == "send" and "Sent" or "Received",
              utils.basename(local_path),
              elapsed / 1000
            )
          )
        elseif not timed_out then
          -- Include rsync's stderr output in the notification so the user
          -- sees the actual error message, not just an opaque exit code.
          local detail = #stderr_lines > 0
              and ("\n" .. table.concat(stderr_lines, "\n"))
            or ""
          utils.error(
            string.format(
              "%s failed (exit code %d)%s",
              direction == "send" and "Send" or "Receive",
              code,
              detail
            )
          )
        end

        local on_complete = config.get_value "on_sync_complete"
        if on_complete then on_complete(direction, local_path, success) end

        if callback then callback(success) end
      end),
    })

    if state.job_id <= 0 then abort "Failed to start rsync" end
  end

  -- Create remote directory first, then start rsync in the callback.
  -- This prevents the race condition where rsync starts (and fails) before
  -- mkdir has completed, causing out-of-order error notifications.
  if direction == "send" and proj_config.auto_create_dirs then
    local remote_dir = remote_file:match "(.+)/[^/]+$"
    if remote_dir then
      ssh.create_remote_dir(
        host,
        remote_dir,
        proj_config.ssh_key,
        function(mkdir_ok)
          if not mkdir_ok then
            logger.warn("mkdir failed", { host = host, dir = remote_dir })
            abort("Failed to create remote directory: " .. remote_dir)
            return
          end
          start_rsync_job()
        end
      )
    else
      start_rsync_job()
    end
  else
    start_rsync_job()
  end
end

--- Try sync with address fallback
---@param direction "send"|"receive"
---@param local_path string
---@param proj_config RemoteSyncProjectConfig
---@param callback fun(success: boolean)|nil
local function sync_with_fallback(direction, local_path, proj_config, callback)
  local addresses = build_address_list(proj_config)

  if #addresses == 0 then
    utils.error "No remote addresses configured"
    if callback then callback(false) end
    return
  end

  local shown_fallback_msg = false

  local function try_address(index)
    if index > #addresses then
      utils.error(
        "Could not connect to any remote address:\n"
          .. table.concat(addresses, ", ")
      )
      if callback then callback(false) end
      return
    end

    local address = addresses[index]
    local remote_path = construct_remote_path(local_path, proj_config, address)

    if not remote_path then
      if callback then callback(false) end
      return
    end

    execute_rsync(
      direction,
      local_path,
      remote_path,
      proj_config,
      function(success)
        if success then
          -- Update last working address
          if address ~= proj_config.last_working_address then
            project.update_last_working_address(address)
          end
          if callback then callback(true) end
        else
          -- Try next address
          if index < #addresses then
            if not shown_fallback_msg then
              shown_fallback_msg = true
              logger.warn(
                "address failed, trying fallback",
                { failed = address, next = addresses[index + 1] }
              )
              utils.info(
                string.format(
                  "Primary address failed, trying %d backup(s)...",
                  #addresses - index
                )
              )
            end
            try_address(index + 1)
          else
            if callback then callback(false) end
          end
        end
      end
    )
  end

  try_address(1)
end

--- Send current file to remote
---@param callback fun(success: boolean)|nil
function M.send_file(callback)
  if M.is_busy() then
    utils.warn "Sync already in progress"
    if callback then callback(false) end
    return
  end

  local proj_config, local_path = require_config_and_file(callback)
  if not proj_config then return end

  sync_with_fallback("send", local_path, proj_config, callback)
end

--- Reload a buffer by path if it is loaded, regardless of which buffer is currently active
---@param path string Absolute path of the file to reload
local function reload_buffer(path)
  local bufnr = vim.fn.bufnr(path)
  if bufnr ~= -1 and vim.api.nvim_buf_is_valid(bufnr) then
    vim.api.nvim_buf_call(bufnr, function() vim.cmd "edit!" end)
  end
end

--- Receive current file from remote
---@param callback fun(success: boolean)|nil
function M.receive_file(callback)
  if M.is_busy() then
    utils.warn "Sync already in progress"
    if callback then callback(false) end
    return
  end

  local proj_config, local_path = require_config_and_file(callback)
  if not proj_config then return end

  -- Check if diff preview is enabled
  if proj_config.show_diff_on_receive then
    M.show_diff(function(confirmed)
      if confirmed then
        sync_with_fallback("receive", local_path, proj_config, function(success)
          if success then reload_buffer(local_path) end
          if callback then callback(success) end
        end)
      else
        utils.info "Receive cancelled"
        if callback then callback(false) end
      end
    end)
  else
    sync_with_fallback("receive", local_path, proj_config, function(success)
      if success then reload_buffer(local_path) end
      if callback then callback(success) end
    end)
  end
end

--- Show diff between local and remote file
---@param callback fun(confirmed: boolean)|nil Called after user decision
function M.show_diff(callback)
  local proj_config, local_path = require_config_and_file(callback)
  if not proj_config then return end

  -- Build address list
  local addresses = build_address_list(proj_config)

  -- Try to get remote file content
  local function try_get_remote(index)
    if index > #addresses then
      utils.error "Could not fetch remote file from any address"
      if callback then callback(false) end
      return
    end

    local address = addresses[index]
    local remote_path = construct_remote_path(local_path, proj_config, address)
    if not remote_path then
      if callback then callback(false) end
      return
    end

    local host, remote_file = utils.parse_remote_path(remote_path)

    ssh.get_remote_file(
      host,
      remote_file,
      proj_config.ssh_key,
      function(content)
        if content then
          -- Create temp buffer with remote content
          local temp_buf = vim.api.nvim_create_buf(false, true)
          vim.api.nvim_buf_set_lines(
            temp_buf,
            0,
            -1,
            false,
            vim.split(content, "\n")
          )
          vim.api.nvim_buf_set_name(temp_buf, "remote://" .. remote_path)

          -- Set filetype for syntax highlighting
          local ft = vim.filetype.match { filename = local_path }
          if ft then
            vim.api.nvim_set_option_value("filetype", ft, { buf = temp_buf })
          end

          -- Open diff view
          local local_win = vim.api.nvim_get_current_win()
          vim.cmd "vsplit"
          local remote_win = vim.api.nvim_get_current_win()
          vim.api.nvim_win_set_buf(0, temp_buf)
          vim.cmd "diffthis"
          vim.api.nvim_set_current_win(local_win)
          vim.cmd "diffthis"
          -- Leave cursor in the remote window. The BufWinLeave autocmd on
          -- temp_buf only fires when that window is closed. If the cursor
          -- were left in the local window and the user pressed :q, they
          -- would close the local window instead and the autocmd would
          -- never fire, so the receive prompt would never appear.
          vim.api.nvim_set_current_win(remote_win)

          utils.info "Showing diff. Close this window (:q) to continue."

          -- Set up autocmd to handle closing
          vim.api.nvim_create_autocmd("BufWinLeave", {
            buffer = temp_buf,
            once = true,
            callback = function()
              -- Clean up diff mode in the local window
              vim.schedule(function()
                if vim.api.nvim_win_is_valid(local_win) then
                  vim.api.nvim_set_current_win(local_win)
                  vim.cmd "diffoff"
                end

                -- Ask user
                vim.ui.select(
                  { "Yes, receive remote version", "No, keep local version" },
                  { prompt = "Receive file from remote?" },
                  function(choice)
                    if callback then
                      callback(choice and choice:match "^Yes")
                    end
                  end
                )
              end)
            end,
          })
        else
          -- Try next address
          try_get_remote(index + 1)
        end
      end
    )
  end

  try_get_remote(1)
end

--- Sync entire directory
---@param direction "send"|"receive"
---@param callback fun(success: boolean)|nil
function M.sync_directory(direction, callback)
  if M.is_busy() then
    utils.warn "Sync already in progress"
    if callback then callback(false) end
    return
  end

  local proj_config = project.load()
  if not proj_config then
    utils.error "No remote sync config found. Run :RemoteSyncConfigure"
    if callback then callback(false) end
    return
  end

  local addresses = build_address_list(proj_config)
  if #addresses == 0 then
    utils.error "No remote addresses configured"
    if callback then callback(false) end
    return
  end

  -- Use first available address for directory sync
  local address = addresses[1]
  local local_dir = proj_config.local_base_path .. "/"
  local remote_dir = address .. ":" .. proj_config.remote_base_path .. "/"

  local src, dest
  if direction == "send" then
    src = local_dir
    dest = remote_dir
  else
    src = remote_dir
    dest = local_dir
  end

  -- Build command
  local cmd = build_rsync_cmd(src, dest, proj_config, direction)

  state.current = direction == "send" and M.State.SYNCING_UP
    or M.State.SYNCING_DOWN

  utils.info(
    string.format(
      "%s directory %s %s...",
      direction == "send" and "Sending" or "Receiving",
      direction == "send" and "to" or "from",
      address
    )
  )

  logger.info(
    "directory sync start",
    { direction = direction, address = address }
  )

  state.job_id = vim.fn.jobstart(cmd, {
    on_exit = vim.schedule_wrap(function(_, code)
      state.current = M.State.IDLE
      state.last_code = code
      state.job_id = -1

      if code == 0 then
        logger.info(
          "directory sync complete",
          { direction = direction, address = address }
        )
        utils.info "Directory sync completed"
        if callback then callback(true) end
      else
        logger.error(
          "directory sync failed",
          { direction = direction, address = address, code = code }
        )
        utils.error(string.format("Directory sync failed (exit code %d)", code))
        if callback then callback(false) end
      end
    end),
  })
end

--- Cancel current sync operation
function M.cancel()
  if state.job_id > 0 then
    vim.fn.jobstop(state.job_id)
    state.current = M.State.IDLE
    state.job_id = -1
    utils.info "Sync cancelled"
  end
end

return M
