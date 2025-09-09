local Snacks = require "snacks"
local glowCmd = function(arg)
  return {
    "tmux",
    "neww",
    "-t",
    "68",
    "glow",
    arg,
  }
end
---@type LazySpec
return {
  {
    "maskudo/devdocs.nvim",
    lazy = false,
    dependencies = {
      "folke/snacks.nvim",
    },
    cmd = { "DevDocs" },
    keys = {
      {
        "<leader>pd",
        mode = "n",
        "<cmd>DevDocs install<cr>",
        desc = "Install Devdocs",
      },
      {
        "<leader>fd",
        mode = "n",
        function()
          local ft = vim.bo.filetype
          if ft == "" then
            print "No filetype detected for the current buffer"
            return
          end

          local devdocs = require "devdocs"
          local installedDocs = devdocs.GetInstalledDocs()

          -- find installed doc matching current filetype
          local docEntry
          for _, doc in ipairs(installedDocs) do
            if vim.startswith(doc, ft) then
              docEntry = doc
              break
            end
          end

          if not docEntry then
            print("DevDocs not installed for filetype: " .. ft)
            return
          end

          -- get the doc directory
          local docDir = devdocs.GetDocDir(docEntry)

          -- open Snacks picker for that doc dir
          require("snacks").picker.files {
            cwd = docDir,
            actions = {
              openGlow = function(picker, _)
                local entry = picker:current { fallback = true }
                if not entry or not entry._path then return end
                -- run tmux command to open glow in a new tmux window
                vim.fn.jobstart(glowCmd(entry._path))
                picker:close()
              end,
            },
            win = {
              input = {
                keys = {
                  ["<CR>"] = {
                    "openGlow",
                    mode = { "n", "i" },
                    desc = "Open with Glow",
                  },
                },
              },
            },
          }
        end,
        desc = "Get Devdocs",
      },
      {
        "<leader>fD",
        mode = "n",
        function()
          local devdocs = require "devdocs"
          local installedDocs = devdocs.GetInstalledDocs()
          Snacks.picker.select(installedDocs, {}, function(selected)
            if not selected then return end
            local docDir = devdocs.GetDocDir(selected)
            -- prettify the filename as you wish
            Snacks.picker.files {
              cwd = docDir,
              actions = {
                openGlow = function(picker, _)
                  local entry = picker:current { fallback = true }
                  if not entry or not entry._path then return end
                  -- run tmux command to open glow in a new tmux window
                  vim.fn.jobstart(glowCmd(entry._path))

                  picker:close()
                end,
              },
              win = {
                input = {
                  keys = {
                    ["<CR>"] = {
                      "openGlow",
                      mode = { "n", "i" },
                      desc = "Open with Glow",
                    },
                  },
                },
              },
            }
          end)
        end,
        desc = "Get All Devdocs",
      },
    },
    opts = {
      ensure_installed = {
        "c",
        "cpp",
        "go",
        "http",
        "python~3.10",
        "lua~5.1",
        "bash",
        "zsh",
      },
    },
  },
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
