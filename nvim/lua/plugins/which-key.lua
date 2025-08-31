---@type LazySpec
return {
  {
    "which-key.nvim",
    config = function()
      require("which-key").setup {
        preset = "helix",
      }
    end,
  },
}
