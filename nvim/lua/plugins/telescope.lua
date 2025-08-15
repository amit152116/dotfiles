require("telescope").load_extension "fzf"
local utils = require "myPlugins.utils"
local multigrep = require "myPlugins.multigrep"

-- Common live_grep with <C-r> mapping
local function grep_with_replace(opts)
  opts = opts or {}
  local user_attach = opts.attach_mappings -- save any user-supplied attach_mappings

  opts.attach_mappings = function(_, map)
    if user_attach then
      -- allow the mapping caller to add more attach logic
      return user_attach(_, map)
    end
    return true
  end

  -- require("telescope.builtin").live_grep(opts)
  multigrep.live_multigrep(opts)
end

return {
  {
    "nvim-telescope/telescope.nvim",
    opts = function(_, opts)
      local trouble = require "trouble.sources.telescope"
      opts.defaults = vim.tbl_deep_extend("force", opts.defaults or {}, {
        mappings = {
          i = {
            ["<c-t>"] = trouble.open,
          },
          n = {
            ["<c-t>"] = trouble.open,
          },
        },
      })
      return opts
    end,
  },
  {
    "AstroNvim/astrocore",
    ---@type AstroCoreOpts
    opts = {
      mappings = {
        n = {
          ["<Leader>fW"] = {
            function()
              grep_with_replace {
                additional_args = { "--hidden", "--no-ignore" },
              }
            end,
            desc = "Find words in all files",
          },

          ["<Leader>fw"] = {
            function() grep_with_replace() end,
            desc = "Find words",
          },

          ["<Leader>pw"] = {
            function()
              grep_with_replace {
                cwd = vim.fs.joinpath(vim.fn.stdpath "data", "lazy"),
              }
            end,
            desc = "Find words",
          },
          ["<Leader>pf"] = {
            function()
              require("telescope.builtin").find_files {
                cwd = vim.fs.joinpath(vim.fn.stdpath "data", "lazy"),
              }
            end,
            desc = "Plugins files",
          },
        },
        x = {
          ["<Leader>fw"] = {
            function()
              local selected_text = utils.search_selected()
              grep_with_replace {
                default_text = selected_text,
                additional_args = { "--fixed-strings" },
              }
            end,
            desc = "Find words",
          },
          ["<Leader>fW"] = {
            function()
              local selected_text = utils.search_selected()
              grep_with_replace {
                default_text = selected_text,
                additional_args = {
                  "--fixed-strings",
                  "--hidden",
                  "--no-ignore",
                },
              }
            end,
            desc = "Find words in all files",
          },
        },
      },
    },
  },
}
