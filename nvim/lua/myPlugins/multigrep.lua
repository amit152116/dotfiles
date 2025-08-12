local M = {}
local action_state = require "telescope.actions.state"
local actions = require "telescope.actions"
local pickers = require "telescope.pickers"
local finders = require "telescope.finders"
local conf = require("telescope.config").values
local previewers = require "telescope.previewers"

function M.live_multigrep(opts)
  opts = opts or {}
  opts.cwd = opts.cwd or vim.uv.cwd()

  finders.new_async_job {
    command_generator = function(prompt)
      if not prompt or prompt == "" then return nil end
      local pieces = vim.split(prompt, "  ")
      local args = { "rg" }

      if pieces[1] then vim.list_extend(args, { "-e", pieces[1] }) end
      if pieces[2] then vim.list_extend(args, { "-g", pieces[2] }) end
      return vim.tbl_flatten {
        args,
        {
          "--color=never",
          "--no-heading",
          "--with-filename",
          "--line-number",
          "--column",
          "--smart-case",
        },
      }
    end,
  }
  pickers
    .new(opts, {
      finder = finders,
    })
    :find()
end
return M
