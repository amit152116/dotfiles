---@type LazySpec
return {
  "ThePrimeagen/refactoring.nvim",
  dependencies = { "lewis6991/async.nvim", "nvim-treesitter/nvim-treesitter" },
  cmd = "Refactor",
  -- ops return "g@" so they take any motion/textobject and dot-repeat; <Leader>r/p are taken by ROS/Plugins
  keys = function()
    local function op(fn, suffix)
      return function() return require("refactoring")[fn]() .. (suffix or "") end
    end
    local function dbg(fn, opts, suffix)
      return function()
        return require("refactoring.debug")[fn](opts) .. (suffix or "")
      end
    end
    local nx = { "n", "x" }
    return {
      {
        "<Leader>le",
        op "extract_func",
        mode = nx,
        expr = true,
        desc = "Extract function",
      },
      {
        "<Leader>lE",
        op "extract_func_to_file",
        mode = nx,
        expr = true,
        desc = "Extract function to file",
      },
      {
        "<Leader>lv",
        op "extract_var",
        mode = nx,
        expr = true,
        desc = "Extract variable",
      },
      {
        "<Leader>lI",
        op "inline_var",
        mode = nx,
        expr = true,
        desc = "Inline variable",
      },
      {
        "<Leader>lF",
        op "inline_func",
        mode = nx,
        expr = true,
        desc = "Inline function",
      },
      {
        "<Leader>lx",
        function() require("refactoring").select_refactor() end,
        mode = nx,
        desc = "Select refactor",
      },
      {
        "<Leader>dv",
        dbg("print_var", { output_location = "below" }, "iw"),
        expr = true,
        desc = "Print var below",
      },
      {
        "<Leader>dv",
        dbg("print_var", { output_location = "below" }),
        mode = "x",
        expr = true,
        desc = "Print var below",
      },
      {
        "<Leader>dV",
        dbg("print_var", { output_location = "above" }, "iw"),
        expr = true,
        desc = "Print var above",
      },
      {
        "<Leader>dV",
        dbg("print_var", { output_location = "above" }),
        mode = "x",
        expr = true,
        desc = "Print var above",
      },
      {
        "<Leader>de",
        dbg("print_exp", { output_location = "below" }),
        mode = nx,
        expr = true,
        desc = "Print expression below",
      },
      {
        "<Leader>dl",
        dbg("print_loc", { output_location = "below" }),
        expr = true,
        desc = "Print location below",
      },
      -- no default range: give a motion, e.g. <Leader>dxG or <Leader>dxip
      {
        "<Leader>dx",
        dbg("cleanup", { restore_view = true }),
        mode = nx,
        expr = true,
        remap = true,
        desc = "Clean debug prints",
      },
    }
  end,
  opts = {
    refactor = {
      extract_var = {
        code_generation = {
          variable_declaration = {
            -- upstream emits a "P" type placeholder for C/C++
            cpp = function(o) return ("auto %s = %s;"):format(o.name, o.value) end,
          },
        },
      },
    },
  },
}
