return {
  "nvim-neo-tree/neo-tree.nvim",
  specs = {
    {
      "AstroNvim/astrocore",
      opts = function(_, opts)
        opts.mappings.n["<Leader>e"] = {
          function()
            for _, win in ipairs(vim.api.nvim_tabpage_list_wins(0)) do
              if
                vim.bo[vim.api.nvim_win_get_buf(win)].filetype == "neo-tree"
              then
                require("neo-tree.command").execute { action = "close" }
                return
              end
            end
            require("neo-tree.command").execute { source = "last" }
          end,
          desc = "Toggle Explorer",
        }
      end,
    },
  },
}
