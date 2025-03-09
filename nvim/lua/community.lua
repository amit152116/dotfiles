-- if true then return {} end -- WARN: REMOVE THIS LINE TO ACTIVATE THIS FILE

-- AstroCommunity: import any community modules here
-- We import this file in `lazy_setup.lua` before the `plugins/` folder.
-- This guarantees that the specs are processed before any user plugins.

---@type LazySpec
return {
    "AstroNvim/astrocommunity",
    { import = "astrocommunity.pack.lua" },
    { import = "astrocommunity.pack.go" },
    { import = "astrocommunity.pack.cpp" },
    { import = "astrocommunity.pack.python" },
    { import = "astrocommunity.editing-support.todo-comments-nvim" },
    { import = "astrocommunity.debugging.persistent-breakpoints-nvim" },
    -- import/override with your plugins folder
}
