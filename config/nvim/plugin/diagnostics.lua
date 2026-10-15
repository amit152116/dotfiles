if vim.g.loaded_diagnostics then return end
vim.g.loaded_diagnostics = true

-- argv files are read before VimEnter, so time those reads now
require("diagnostics.collectors.bigfile").capture_early()

-- after plugin setup, so startup stays fast and notify wrapping sees final vim.notify
vim.api.nvim_create_autocmd("VimEnter", {
  once = true,
  callback = function()
    if not vim.g.diagnostics_setup_done then require("diagnostics").setup() end
  end,
})
