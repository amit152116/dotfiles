local M = {}
-- Table to store buffer history
local buf_history = {}

-- Function to switch to previous buffer in history
function M.switch_prev_buffer()
  local cur = vim.api.nvim_get_current_buf()

  -- Remove current buffer if it’s already in history
  for i = #buf_history, 1, -1 do
    if buf_history[i] == cur then table.remove(buf_history, i) end
  end

  -- Add current buffer to the front
  table.insert(buf_history, 1, cur)

  -- Pick the previous buffer (2nd in the list)
  if #buf_history > 1 then
    local prev = buf_history[2]
    vim.api.nvim_set_current_buf(prev)
  end
end
return M
