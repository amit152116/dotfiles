local M = {}
local debounce_timer = vim.uv.new_timer()
local Snacks = require "snacks"

--
---@param direction "next"|"prev" Direction to cycle buffers
function M.buffer_cycle(direction)
  Snacks.picker.buffers {
    focus = "list",
    layout = { preset = "select" },
    on_show = function(picker)
      -- This correctly triggers the sequence on open
      if direction == "prev" then
        picker:action "list_down"
      else
        picker:action "list_up"
      end
      picker:action "auto_confirm"
    end,
    actions = {
      -- Add this to your auto_confirm function
      auto_confirm = function(picker, _)
        -- Cancel existing timer if running
        if debounce_timer == nil then return end
        if debounce_timer:is_active() then debounce_timer:stop() end

        -- Create new timer
        debounce_timer:start(500, 0, function()
          -- This message should ONLY appear after the delay
          vim.schedule(function()
            if picker and not picker.closed and debounce_timer then
              picker:close()
              local item = picker:current { fallback = true }
              if not item then return end
              vim.cmd("buffer " .. item.buf)
            end
          end)
        end)
      end,
    },
    on_close = function()
      -- Cleanup timer when picker closes
      if debounce_timer then debounce_timer:stop() end
    end,
    win = {
      list = {
        keys = {
          ["<C-n>"] = { { "list_down", "auto_confirm" } },
          ["<C-p>"] = { { "list_up", "auto_confirm" } },
          ["j"] = { { "list_down", "auto_confirm" } },
          ["k"] = { { "list_up", "auto_confirm" } },
        },
      },
    },
  }
end
return M
