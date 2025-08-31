local M = {}
local debounce_timer = nil
local Snacks = require "snacks"

Snacks.picker.buffers {
  focus = "list",
  layout = { preset = "select" },
  on_show = function(picker)
    -- This correctly triggers the sequence on open
    picker:action "list_down"
    picker:action "auto_confirm"
  end,
  actions = {
    -- Add this to your auto_confirm function
    auto_confirm = function(picker, _)
      vim.notify("[SNACKS] auto_confirm called", vim.log.levels.INFO)

      -- Cancel existing timer if running
      if debounce_timer then
        vim.notify("[SNACKS] > Stopping existing timer", vim.log.levels.INFO)
        debounce_timer:stop()
        debounce_timer:close()
        debounce_timer = nil
      end

      -- Create new timer
      vim.notify("[SNACKS] > Starting new 500ms timer...", vim.log.levels.INFO)
      debounce_timer = vim.uv.new_timer()
      debounce_timer:start(500, 0, function()
        -- This message should ONLY appear after the delay
        vim.notify("[SNACKS] >> TIMER FIRED!", vim.log.levels.WARN)
        vim.schedule(function()
          if picker and not picker.closed and debounce_timer then
            picker:close()
            local item = picker:current { fallback = true }
            if not item then return end
            vim.cmd("buffer " .. item.buf)
            vim.notify(
              "[SNACKS] >>> Switched to buffer " .. item.buf,
              vim.log.levels.INFO
            )
          end
        end)
      end)
    end,
  },
  on_close = function()
    -- Cleanup timer when picker closes
    if debounce_timer then
      debounce_timer:stop()
      debounce_timer:close()
      debounce_timer = nil
    end
  end,
  win = {
    list = {
      keys = {
        ["<C-n>"] = { { "list_down", "auto_confirm" }, mode = { "n", "i" } },
        ["<C-p>"] = { { "list_up", "auto_confirm" }, mode = { "n", "i" } },
        ["j"] = { "list_down", "auto_confirm" },
        ["k"] = { "list_up", "auto_confirm" },
      },
    },
  },
}
return M
