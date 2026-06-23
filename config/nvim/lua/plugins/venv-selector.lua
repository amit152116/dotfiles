return {
  "linux-cultist/venv-selector.nvim",
  enabled = vim.fn.executable "fd" == 1
    or vim.fn.executable "fdfind" == 1
    or vim.fn.executable "fd-find" == 1,
  dependencies = {
    {
      "folke/snacks.nvim",
      version = "*",
      dependencies = { "nvim-lua/plenary.nvim" },
    },
    {
      "AstroNvim/astrocore",
      opts = {
        mappings = {
          n = {
            ["<Leader>lv"] = false,
          },
        },
        autocmds = {
          python_env = {
            {
              event = "LspAttach",
              desc = "Load python_extensions",
              callback = function(args)
                if
                  assert(vim.lsp.get_client_by_id(args.data.client_id)).name
                  == "basedpyright"
                then
                  require("astrocore").set_mappings({
                    n = {
                      ["<Leader>lv"] = {
                        "<Cmd>VenvSelect<CR>",
                        desc = "Select VirtualEnv",
                      },
                    },
                  }, { buffer = args.buf })
                end
              end,
            },
          },
        },
      },
    },
  },
  cmd = "VenvSelect",
  ft = "python",
  opts = {
    options = {
      on_venv_activate_callback = nil,
      enable_default_searches = true,
      enable_cached_venvs = true,
      cached_venv_automatic_activation = true,
      activate_venv_in_terminal = true,
      set_environment_variables = true,
      notify_user_on_venv_activation = false,
      override_notify = true,
      search_timeout = 5,
      log_level = "NONE", -- DEBUG, TRACE, or NONE -- inspect with :VenvSelectLog
      fd_binary_name = nil,
      require_lsp_activation = true,
      shell = { shell = vim.o.shell, shellcmdflag = vim.o.shellcmdflag },
      on_telescope_result_callback = nil,
      picker_filter_type = "substring",
      selected_venv_marker_color = "#00FF00",
      selected_venv_marker_icon = "✔ ",
      picker_icons = {
        default = "",
      },
      picker_columns = {
        "marker",
        "search_icon",
        "search_name",
        "search_result",
      },
      picker_options = {},
      picker = "auto", -- "telescope", "fzf-lua", "snacks", "native", "mini-pick", or "auto"
      statusline_func = { nvchad = nil, lualine = nil },
    },
    search = {},
  },
}
