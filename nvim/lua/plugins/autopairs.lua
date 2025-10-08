return {
  {
    "windwp/nvim-autopairs",
    enabled = false,
    config = function(plugin, opts)
      require "astronvim.plugins.configs.nvim-autopairs"(plugin, opts) -- include the default astronvim config that calls the setup call
      -- add more custom autopairs configuration such as custom rules
      local npairs = require "nvim-autopairs"
      local Rule = require "nvim-autopairs.rule"
      local cond = require "nvim-autopairs.conds"
      npairs.add_rules(
        {
          Rule("$", "$", { "tex", "latex" })
            -- don't add a pair if the next character is %
            :with_pair(
              cond.not_after_regex "%%"
            )
            -- don't add a pair if  the previous character is xxx
            :with_pair(
              cond.not_before_regex("xxx", 3)
            )
            -- don't move right when repeat character
            :with_move(
              cond.none()
            )
            -- don't delete if the next character is xx
            :with_del(
              cond.not_after_regex "xx"
            )
            -- disable adding a newline when you press <cr>
            :with_cr(
              cond.none()
            ),
        },
        -- disable for .vim files, but it work for another filetypes
        Rule("a", "a", "-vim")
      )
    end,
  },
  {
    "altermo/ultimate-autopair.nvim",
    event = "InsertEnter",
    branch = "v0.6", --recommended as each new version will have breaking changes
    opts = {
      -- disable autopair in the command line: https://github.com/altermo/ultimate-autopair.nvim/issues/8
      cmap = false,
      extensions = {
        cond = {
          -- disable in comments
          -- https://github.com/altermo/ultimate-autopair.nvim/blob/6fd0d6aa976a97dd6f1bed4d46be1b437613a52f/Q%26A.md?plain=1#L26
          cond = {
            function(fn) return not fn.in_node "comment" end,
          },
        },
        -- get fly mode working on strings:
        -- https://github.com/altermo/ultimate-autopair.nvim/issues/33
        fly = {
          nofilter = true,
        },
      },
      config_internal_pairs = {
        { '"', '"', fly = true },
        { "'", "'", fly = true },
      },
    },
    dependencies = {
      {
        "AstroNvim/astrocore",
        opts = {
          mappings = {
            n = {
              ["<Leader>ua"] = {
                desc = "Toggle Ultimate Autopair",
                function()
                  local notify = require("astrocore").notify
                  local function bool2str(bool) return bool and "on" or "off" end
                  local ok, ultimate_autopair =
                    pcall(require, "ultimate-autopair")
                  if ok then
                    ultimate_autopair.toggle()
                    vim.g.ultimate_autopair_enabled =
                      require("ultimate-autopair.core").disable
                    notify(
                      string.format(
                        "ultimate-autopair %s",
                        bool2str(not vim.g.ultimate_autopair_enabled)
                      )
                    )
                  else
                    notify "ultimate-autopair not available"
                  end
                end,
              },
            },
          },
        },
      },
    },
    specs = {
      {
        "windwp/nvim-autopairs",
        optional = true,
        enabled = false,
      },
    },
  },
}
