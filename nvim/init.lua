-- This file simply bootstraps the installation of Lazy.nvim and then calls other files for execution
-- This file doesn't necessarily need to be touched, BE CAUTIOUS editing this file and proceed at your own risk.
local lazypath = vim.env.LAZY or vim.fn.stdpath "data" .. "/lazy/lazy.nvim"
if not (vim.env.LAZY or (vim.uv or vim.loop).fs_stat(lazypath)) then
  -- stylua: ignore
  vim.fn.system({ "git", "clone", "--filter=blob:none", "https://github.com/folke/lazy.nvim.git", "--branch=stable", lazypath })
end
vim.opt.rtp:prepend(lazypath)

-- validate that lazy is available
if not pcall(require, "lazy") then
  -- stylua: ignore
  vim.api.nvim_echo({ { ("Unable to load lazy from: %s\n"):format(lazypath), "ErrorMsg" }, { "Press any key to exit...", "MoreMsg" } }, true, {})
    vim.fn.getchar()
    vim.cmd.quit()
end

vim.filetype.add {
    extension = {
        urdf = "urdf", -- Not "xml" - we want a distinct filetype
    },
}

-- Make .urdf files use XML syntax highlighting
vim.api.nvim_create_autocmd({ "BufNewFile", "BufRead" }, {
    pattern = "*.urdf",
    callback = function()
        -- Set the filetype for snippets
        vim.bo.filetype = "urdf"
        -- Use XML syntax for highlighting
        vim.bo.syntax = "xml"
        -- Optional: enable XML indentation
        vim.bo.shiftwidth = 2
        vim.bo.tabstop = 2
        vim.bo.expandtab = true
    end,
})
require "lazy_setup"
require "polish"
require "myPlugins.utils"
require "myPlugins.snippets"
