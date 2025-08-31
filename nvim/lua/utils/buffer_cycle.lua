local M = {}

--- Gets a list of active buffers, sorted by the most recently used.
---@param options? { include_unlisted?: boolean } An optional table to include unlisted buffers.
---@return table[] A list of buffer information tables.
function M.GetBuffersByLastUsed(options)
  options = options or {}

  local items = {}
  local current_buf = vim.api.nvim_get_current_buf()

  -- Loop through all buffer numbers in the current Neovim instance.
  for _, buf in ipairs(vim.api.nvim_list_bufs()) do
    -- We will apply some filters to get only "useful" buffers.
    -- 1. Buffer must be loaded in memory.
    -- 2. Buffer must not be a "nofile" type (e.g., terminals, help pages).
    -- 3. Buffer must be "listed" (appears in the buffer list), unless overridden by options.
    local is_valid = vim.api.nvim_buf_is_loaded(buf)
      and vim.bo[buf].buftype ~= "nofile"
      and (options.include_unlisted or vim.bo[buf].buflisted)

    if is_valid then
      local name = vim.api.nvim_buf_get_name(buf)
      if name == "" then name = "[No Name]" end

      -- getbufinfo is the key to finding the 'lastused' timestamp.
      local info = vim.fn.getbufinfo(buf)[1]

      -- Add a table with the buffer's info to our list.
      table.insert(items, {
        bufnr = buf,
        name = name,
        lastused = info.lastused, -- The timestamp we will sort by.
        is_current = (buf == current_buf),
        is_modified = info.changed == 1,
      })
    end
  end

  -- Sort the collected items table.
  -- The function `a.info.lastused > b.info.lastused` sorts the list in
  -- descending order, so the highest timestamp (most recent) comes first.
  table.sort(items, function(a, b) return a.lastused > b.lastused end)

  return items
end

--- Cycles to the next or previous buffer based on most recently used order.
---@param direction "next" | "prev" The direction to cycle in.
function M.CycleBuffer(direction)
  -- 1. Get the fresh, perfectly sorted list of buffers, every single time.
  local sorted_buffers = M.GetBuffersByLastUsed()

  -- If we have 1 or fewer buffers, there's nowhere to cycle to.
  if #sorted_buffers <= 1 then return end

  -- 2. Find the index of our current buffer in that list.
  local current_bufnr = vim.api.nvim_get_current_buf()
  local current_index = -1
  for i, buf_info in ipairs(sorted_buffers) do
    if buf_info.bufnr == current_bufnr then
      current_index = i
      break
    end
  end

  -- If the current buffer isn't in our list for some reason, do nothing.
  if current_index == -1 then return end

  -- 3. Calculate the index of the next buffer, with wrapping.
  local next_index
  if direction == "prev" then
    -- The modulo operator wraps around from the end back to the beginning.
    next_index = (current_index % #sorted_buffers) + 1
  else -- direction == "prev"
    -- This calculation correctly wraps around from the beginning to the end.
    next_index = (current_index - 2 + #sorted_buffers) % #sorted_buffers + 1
  end

  -- 4. Get the buffer number for our target buffer and switch to it.
  local next_bufnr = sorted_buffers[next_index].bufnr
  vim.cmd.buffer(next_bufnr)
end
return M
