-- Structural find/replace via ast-grep (multiline, syntax-tree aware).
-- Complements myPlugins/grep.lua (snacks line-grep + fuzzy nav); this is the
-- refactor buffer. Requires the `ast-grep` CLI (>= 0.36) on PATH.
return {
  "MagicDuck/grug-far.nvim",
  cmd = { "GrugFar", "GrugFarWithin" },
  -- `<Leader>fr` under the find namespace (`<Leader>f` = snacks pickers);
  -- `<Leader>s` is ssh/remote-sync, `<Leader>r` is ROS, `<Leader>R` is rename.
  keys = {
    {
      "<Leader>fr",
      function() require("grug-far").open() end,
      mode = "n",
      desc = "find/replace",
    },
    {
      "<Leader>fr",
      function() require("grug-far").with_visual_selection() end,
      mode = "x",
      desc = "search visual selection",
    },
    {
      "<Leader>fR",
      function()
        require("grug-far").open {
          prefills = { paths = vim.fn.expand "%" },
        }
      end,
      mode = "n",
      desc = "find/replace in current file",
    },
  },
  opts = {
    -- default to structural matching for the robotics C++/Python refactors
    engine = "astgrep",
    -- only cycle through the engines worth using on Swap Engine action
    enabledEngines = { "astgrep", "ripgrep", "astgrep-rules" },
    -- skip colcon workspace artifacts (mirrors todo-comments search globs)
    engines = {
      ripgrep = {
        extraArgs = "--glob=!build/ --glob=!install/ --glob=!log/",
      },
    },
    windowCreationCommand = "botright vsplit",
  },
}
