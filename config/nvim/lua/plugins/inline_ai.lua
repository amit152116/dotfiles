-- Test bench for free inline-suggestion plugins.
-- Switch backend in lua/ai_provider.lua -- one change, restart nvim.
local active = require("ai_provider").backend

return {
  -- pack.cpp pulls this in for nvim-dap, but it assumes nvim-cmp, which this
  -- config doesn't use (blink.cmp instead) -- breaks startup otherwise.
  -- { "rcarriga/cmp-dap", enabled = false },

  -- NeoCodeium: https://github.com/monkoose/neocodeium
  {
    "monkoose/neocodeium",
    event = "VeryLazy",
    config = function()
      local neocodeium = require "neocodeium"
      -- clear ghost text when blink's popup opens, and don't refire while it's visible
      vim.api.nvim_create_autocmd("User", {
        pattern = "BlinkCmpMenuOpen",
        callback = function() neocodeium.clear() end,
      })

      neocodeium.setup {
        show_label = true,
        silent = true, -- skip "server started"/"server stopped" noise
        debounce = true, -- wait for typing pause before requesting; cuts request spam
        max_lines = 5000, -- ROS repos have huge files; cap context scan for latency
        single_line = {
          enabled = true, -- collapse multi-line suggestions so they don't fight blink's popup for screen space
          label = "...",
        },
        filter = function() return not require("blink.cmp").is_visible() end,
        filetypes = {
          help = false,
          gitcommit = false,
          gitrebase = false,
          ["."] = false,
        },
        -- ROS workspace markers, in addition to defaults
        root_dir = {
          ".bzr",
          ".git",
          ".hg",
          ".svn",
          "_FOSSIL_",
          "package.json",
          "package.xml",
          "CMakeLists.txt",
        },
      }
    end,
    specs = {
      {
        "AstroNvim/astrocore",
        opts = {
          options = {
            g = {
              -- AstroNvim's Tab mapping checks this before snippet/indent fallback
              ai_accept = function()
                local neocodeium = require "neocodeium"
                if neocodeium.visible() then
                  neocodeium.accept()
                  return true
                end
              end,
            },
          },
        },
      },
    },
  },
}
