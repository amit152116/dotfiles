-- local config = require "fzf-lua.config"
-- local actions = require("trouble.sources.fzf").actions
-- config.defaults.actions.files["ctrl-t"] = actions.open
--

---@type LazySpec
return {

  "AstroNvim/astrocore",
  ---@type AstroCoreOpts
  opts = {
    autocmds = {
      -- {
      --   event = "FileType",
      --   pattern = "qf",
      --   desc = "Close quickfix window safely",
      --   callback = function()
      --     vim.defer_fn(function() pcall(vim.cmd, "cclose") end, 50)
      --   end,
      -- },
      -- -- Open Trouble instead of quickfix UI
      -- ["QuickFixCmdPost"] = {
      --   callback = function()
      --     vim.schedule(function()
      --       local trouble = require "trouble"
      --       if trouble.is_open() and trouble.get_mode() == "quickfix" then
      --         trouble.refresh()
      --       else
      --         trouble.open "quickfix"
      --       end
      --     end)
      --   end,
      -- },
    },
  },
}
