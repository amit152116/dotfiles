local Snacks = require "snacks"

local M = {}
M.grep = require("myPlugins.grep").Multigrep
M.ros = require "myPlugins.ros"
M.git_worktree = require "myPlugins.git_worktree"
M.git_worktree.setup()

-- Create timer once and reuse
local debounce_timer = vim.uv.new_timer()

-- Common function for buffer selection and timer handling
local function handle_buffer_selection(picker, delay)
  if not picker or not debounce_timer then return end

  -- Stop existing timer if active
  if debounce_timer:is_active() then debounce_timer:stop() end

  debounce_timer:start(delay, 0, function()
    vim.schedule(function()
      if not (picker and picker.closed == false) then return end

      picker:close()
      local item = picker:current { fallback = true }
      if item then vim.cmd("buffer " .. item.buf) end
    end)
  end)
end

---@param direction "next"|"prev" Direction to cycle buffers
function M.buffer_cycle(direction)
  Snacks.picker.buffers {
    focus = "list",
    auto_close = true,
    auto_confirm = true,
    layout = { preset = "select" },
    on_show = function(picker)
      picker:action(direction == "prev" and "list_down" or "list_up")
      picker:action "buf_select"
    end,
    actions = {
      buf_select = function(picker) handle_buffer_selection(picker, 500) end,
      insert_mode = function(picker)
        picker:action "focus_input"
        handle_buffer_selection(picker, 2000)
      end,
    },
    on_close = function()
      if debounce_timer then debounce_timer:stop() end
    end,
    win = {
      input = { keys = {} },
      list = {
        keys = {
          ["<C-n>"] = { { "list_down", "buf_select" } },
          ["<C-p>"] = { { "list_up", "buf_select" } },
          ["j"] = { { "list_down", "buf_select" } },
          ["k"] = { { "list_up", "buf_select" } },
          ["i"] = { { "insert_mode" } },
        },
      },
    },
  }
end

return M
