---@type LazySpec
return {
  "christoomey/vim-tmux-navigator",
  lazy = false,
  init = function()
    vim.g.tmux_navigator_no_mappings = 1 -- mapped via astrocore instead, avoids lazy-load order race
    vim.g.tmux_navigator_preserve_zoom = 1
  end,
}
