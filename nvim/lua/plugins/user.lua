-- You can also add or configure plugins by creating files in this `plugins/` folder
-- PLEASE REMOVE THE EXAMPLES YOU HAVE NO INTEREST IN BEFORE ENABLING THIS FILE
-- Here are some examples:

---@type LazySpec
return {

  -- == Examples of Adding Plugins ==
  { "andweeb/presence.nvim" },
  {
    "ray-x/lsp_signature.nvim",
    event = "BufRead",
    enabled = false,
    config = function() require("lsp_signature").setup() end,
  },

  {
    "yutkat/confirm-quit.nvim",
    event = "CmdlineEnter",
    opts = {
      overwrite_q_command = true,
      quit_message = "Do you want to quit?",
    },
  },
  -- You can disable default plugins as follows:
  { "max397574/better-escape.nvim", enabled = true },
}
