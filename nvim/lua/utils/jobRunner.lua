local M = {}

---@alias JobRunnerCmd string|string[]

---@alias JobRunner fun(cmd: JobRunnerCmd, opts?: Ros2PickerOpts)

---@alias CacheTable table<string, CacheEntry>

---@alias JobRunnerResult JobRunnerItem[]|string

---@class JobRunnerItem
---@field value string    # Raw output string item
---@field display? string # Formatted display string for picker UI
---@field filter_text? string

---@class JobRunnerOpts
---@field on_complete fun(results?: JobRunnerResult, err?: string) # Callback executed after command finishes.
---@field transform? fun(item: table): boolean               # Optional filter/transform for each output line.
---@field no_cache? boolean                                   # If true, disables result caching.
---@field stream? boolean                                     # If true, streams results to callback as they arrive.

---@class CacheEntry
---@field data JobRunnerResult        # The cached data
---@field timestamp number  # Unix timestamp when cached

-- Default configuration
local config = {
  cache_timeout = 300, -- default cache timeout (seconds)
  debounce_delay = 150, -- debounce delay for preview (ms)
}

---Internal cache storage.
---@type CacheTable
local caches = {}

-------------------------------------------------------
-- Caching Helpers
-------------------------------------------------------

-- Normalize command for cache key
---comment
---@param cmd JobRunnerCmd
---@return string
local function normalize_cmd(cmd)
  if type(cmd) == "string" then
    return cmd
  elseif type(cmd) == "table" then
    -- Join all parts with space
    return table.concat(cmd, " ")
  else
    return ""
  end
end

--Retrieve cached data if it's still valid.
---@param cmd string The shell command used as the cache key.
---@return JobRunnerResult ? The cached results, or nil if not found or expired.
local function get_from_cache(cmd)
  local entry = caches[cmd]
  if
    entry and os.difftime(os.time(), entry.timestamp) < config.cache_timeout
  then
    if type(entry.data) == "table" then
      return vim.deepcopy(entry.data) -- deep copy only if table
    else
      return entry.data -- plain string, safe to return
    end
  end
  -- Cache is invalid or doesn't exist, clear it
  caches[cmd] = nil
  return nil
end

---Update the cache for a given command.
---@param cmd string The shell command to use as the cache key.
---@param results JobRunnerResult The command output to store.
local function update_cache(cmd, results)
  caches[cmd] = {
    data = results,
    timestamp = os.time(),
  }
end

-------------------------------------------------------
-- Core Logic
-------------------------------------------------------

---Asynchronously executes a shell command with optional caching and streaming.
---@type JobRunner
function M.run(cmd, opts)
  opts = opts or {}
  opts.stream = opts.stream or false

  -- Validate cmd type
  if type(cmd) ~= "string" and type(cmd) ~= "table" then
    vim.notify(
      "JobRunner: cmd must be string or table of strings",
      vim.log.levels.ERROR
    )
    return
  end

  -- Normalize table commands (ensure all entries are strings)
  if type(cmd) == "table" then
    for _, part in ipairs(cmd) do
      if type(part) ~= "string" then
        vim.notify(
          "JobRunner: All command parts must be strings",
          vim.log.levels.ERROR
        )
        return
      end
    end
    cmd = normalize_cmd(cmd)
  end

  if not opts.on_complete or type(opts.on_complete) ~= "function" then
    vim.notify(
      "JobRunner: A valid callback function must be provided",
      vim.log.levels.ERROR
    )
    return
  end

  -- Check cache if enabled
  if not opts.no_cache then
    local cached_data = get_from_cache(cmd)
    if cached_data then
      -- Use vim.schedule to ensure callback is always executed in the main loop
      vim.schedule(function() opts.on_complete(cached_data) end)
      return
    end
  end

  local results = {}
  local stderr_lines = ""

  -- Add piped command also
  local cmd_to_run
  if cmd:find "|" then
    cmd_to_run = { "sh", "-c", cmd }
  else
    cmd_to_run = cmd
  end

  -- Execute command
  vim.fn.jobstart(cmd_to_run, {
    stdout_buffered = not opts.stream,
    on_stdout = vim.schedule_wrap(function(_, data)
      -- `data_chunk` is a table of strings. It might contain partial lines.
      if not data then return end
      -- Process all the complete lines we received in this chunk
      for _, line in ipairs(data) do
        local ok = true
        local item
        if opts.stream then
          line = vim.trim(line)
          if line ~= "" then
            item = { value = line, display = line }
            -- Apply the transform to the individual line
            if opts.transform then ok = opts.transform(item) end
          end
        else
          item = line
        end
        if ok then table.insert(results, item) end
      end
    end),
    stderr_buffered = true,
    on_stderr = function(_, data) stderr_lines = table.concat(data, "\n") end,
    on_exit = vim.schedule_wrap(function(_, code)
      if code ~= 0 then
        local err_msg =
          string.format("❌ cmd: `%s` failed (code %d)", cmd, code)
        if #stderr_lines > 0 then err_msg = err_msg .. "\n" .. stderr_lines end
        vim.notify(err_msg, vim.log.levels.ERROR)
        opts.on_complete(nil, err_msg)
        return
      end

      -- Determine which results to use
      if opts.stream then
        -- Update cache if enabled
        if not opts.no_cache then update_cache(cmd, results) end

        -- Return results via callback
        opts.on_complete(results)
      else
        -- Update cache if enabled
        if not opts.no_cache then update_cache(cmd, results) end

        -- Return results via callback
        opts.on_complete(results)
      end
    end),
  })
end

---Asynchronously executes a shell command with optional caching
---@type JobRunner
function M.stream(cmd, opts)
  opts = opts or {}
  opts.stream = true
  M.run(cmd, opts)
end

--- User-facing setup function.
--- Merges user-provided options with the defaults.
---@param opts table? User configuration options.
function M.setup(opts) config = vim.tbl_deep_extend("force", config, opts or {}) end

return M
