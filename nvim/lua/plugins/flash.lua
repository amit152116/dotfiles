return {
  "folke/flash.nvim",
  event = "VeryLazy",
  dependencies = {
    {
      "AstroNvim/astrocore",
      opts = {
        mappings = {
          n = {
            ["s"] = {
              function() require("flash").jump {} end,
              desc = "Flash Jump",
            },
            ["S"] = {
              function()
                require("flash").jump {
                  search = { mode = "search", max_length = 0 },
                  label = { after = { 0, 0 } },
                  pattern = "^",
                }
              end,
              desc = "Flash Line",
            },
            ["<CR>"] = {
              function() require("flash").treesitter() end,
              desc = "Flash Treesitter",
            },
          },
          x = {
            ["s"] = {
              function() require("flash").jump() end,
              desc = "Flash Jump",
            },
            ["R"] = {
              function() require("flash").treesitter_search() end,
              desc = "Treesitter Search",
            },
            ["<CR>"] = {
              function() require("flash").treesitter() end,
              desc = "Flash Treesitter",
            },
          },
          o = {
            ["r"] = {
              function() require("flash").remote() end,
              desc = "Remote Flash",
            },
            ["R"] = {
              function() require("flash").treesitter_search() end,
              desc = "Treesitter Search",
            },
            ["s"] = {
              function() require("flash").treesitter() end,
              desc = "Flash Treesitter",
            },

            ["j"] = {
              function() require("flash").jump() end,
              desc = "Flash Jump",
            },
          },
          c = {
            ["<c-s>"] = {
              function() require("flash").toggle() end,
              desc = "Toggle Flash Search",
            },
          },
        },
      },
    },
  },
  opTreesitterts = {},
}
