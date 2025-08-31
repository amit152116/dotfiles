---@type LazySpec
return {
  {
    "luckasRanarison/nvim-devdocs",
    enabled = false,

    dependencies = {
      "nvim-lua/plenary.nvim",
      "nvim-telescope/telescope.nvim",
      "nvim-treesitter/nvim-treesitter",
    },
    opts = {
      default_language = "lua",
      default_browser = "firefox",
      default_search_engine = "duckduckgo",
      default_theme = "dark",

      ensure_installed = {
        "c",
        "cpp",
        "go",
        "python~3.10",
        "lua~5.1",
        "bash",
        "numpy~2.2",
        "pandas~2",
        "matplotlib",
        "tensorflow",
        "tensorflow_cpp",
        "scikit_learn",
      },
      cmd_ignore = {}, -- ignore cmd rendering for the listed docs
    },

    -- keys = {
    --   {
    --     "<Leader>fd",
    --     "<cmd>DevdocsOpenCurrent<CR>",
    --     mode = "n",
    --     desc = "Current DevDocs",
    --   },
    --   {
    --     "<Leader>fD",
    --     function() require("nvim-devdocs")._open_docs_with_telescope() end,
    --     mode = "n",
    --     desc = "FindDevDocs",
    --   },
    -- },

    config = function(_, opts)
      local devdocs = require "nvim-devdocs"
      local devdocs_list = require "nvim-devdocs.list"
      -- Setup with given options
      devdocs.setup(opts)

      function devdocs._open_docs_with_telescope()
        local docs = devdocs_list.get_installed_alias()
        require("telescope.pickers")
          .new({}, {
            prompt_title = "Select Documentation",
            finder = require("telescope.finders").new_table {
              results = docs,
            },
            sorter = require("telescope.config").values.generic_sorter {},
            attach_mappings = function(_, map)
              map("i", "<CR>", function(prompt_bufnr)
                local selection =
                  require("telescope.actions.state").get_selected_entry()
                require("telescope.actions").close(prompt_bufnr)
                vim.cmd("DevdocsOpen " .. selection.value)
              end)
              return true
            end,
            layout_strategy = "center",
            layout_config = {
              width = 0.3,
              height = 0.3,
            },
          })
          :find()
      end
    end,
  },
}
