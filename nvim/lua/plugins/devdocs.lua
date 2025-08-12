---@type LazySpec
return {
  {
    "luckasRanarison/nvim-devdocs",

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
    config = function(_, opts)
      local devdocs = require "nvim-devdocs"
      local devdocs_list = require "nvim-devdocs.list"
      -- Setup with given options
      devdocs.setup(opts)

      -- Function to open docs with Telescope
      local function get_docs_list() return devdocs_list.get_installed_alias() end

      local function open_docs_with_telescope()
        local docs = get_docs_list()
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

      -- Optional: map a key to open the picker
      vim.keymap.set(
        "n",
        "<leader>fD",
        open_docs_with_telescope,
        { desc = "Find DevDocs" }
      )
      vim.keymap.set(
        "n",
        "<leader>fd",
        "<cmd>DevdocsOpenCurrent<CR>",
        { desc = "Current DevDocs" }
      )
    end,
  },
}
