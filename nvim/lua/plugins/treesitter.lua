-- Customize Treesitter

---@type LazySpec
return {
  { "nvim-treesitter/playground" },
  {
    "nvim-treesitter/nvim-treesitter",
    opts = {
      ensure_installed = {
        "lua",
        "vim",
        "latex",
        -- add more arguments for adding more treesitter parsers
      },
    },
  },
}
