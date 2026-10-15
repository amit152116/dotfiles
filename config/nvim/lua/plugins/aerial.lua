return {
  "stevearc/aerial.nvim",
  -- AstroFile attached before first draw, compiling the cpp/aerial query (~50ms) into startup
  event = function() return { "VeryLazy" } end,
}
