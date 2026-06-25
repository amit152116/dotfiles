return {
  {
    "Civitasv/cmake-tools.nvim",
    ft = { "cmake", "c", "cpp" },
    dependencies = {
      {
        "AstroNvim/astrocore",
        opts = {
          autocmds = {
            cmake_command = {
              {
                event = "LspAttach",
                desc = "Load cmake-tools mappings",
                callback = function(args)
                  if
                    assert(vim.lsp.get_client_by_id(args.data.client_id)).name
                    == "clangd"
                  then
                    require("astrocore").set_mappings({
                      n = {
                        ["<Leader>le"] = {
                          "<Cmd>CMakeRunCurrentFile<CR>",
                          desc = "Run Current File",
                        },
                        ["<Leader>lE"] = {
                          "<Cmd>CMakeQuickRun<CR>",
                          desc = "Run Executable",
                        },
                        ["<Leader>lt"] = {
                          "<Cmd>CMakeRunTest<CR>",
                          desc = "Run Tests",
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
    opts = {
      cmake_command = "cmake",
      ctest_command = "ctest",
      cmake_use_preset = true,
      cmake_regenerate_on_save = true, -- only triggers from CMakeLists.txt saves
      cmake_generate_options = {
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=1",
      },
      cmake_build_options = {
        "-DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache",
      },
      -- supports macro expansion: ${kit}, ${kitGenerator}, ${variant:xx}
      cmake_build_directory = function()
        if require("cmake-tools.osys").iswin32 then return "out\\${variant:buildType}" end
        return "out/${variant:buildType}" -- relative to cwd
      end,
      cmake_compile_commands_options = {
        -- soft_link: symlink compile_commands.json to target; copy: copy it; lsp: point lsp at it directly; none: skip
        action = "soft_link",
        target = vim.loop.cwd(), -- only used by soft_link/copy
      },
      cmake_kits_path = nil,
      cmake_variants_message = {
        short = { show = true },
        long = { show = true, max_length = 40 },
      },
      cmake_dap_configuration = {
        name = "cpp",
        type = "codelldb",
        request = "launch",
        stopOnEntry = false,
        runInTerminal = true,
        console = "integratedTerminal",
      },
      cmake_executor = {
        name = "quickfix",
        opts = {}, -- merged into the matching default_opts[name] entry below
        default_opts = {
          quickfix = {
            show = "always", -- "always", "only_on_error"
            position = "belowright", -- see `:h vertical` for valid split positions
            size = 10,
            encoding = "utf-8", -- non-utf-8 output gets converted via vim.fn.iconv
            auto_close_when_success = true,
          },
          toggleterm = {
            direction = "float", -- "vertical" | "horizontal" | "tab" | "float"
            close_on_exit = false,
            auto_scroll = true,
            singleton = true, -- reuses/auto-closes any existing instance instead of stacking
          },
          overseer = {
            new_task_opts = {
              strategy = {
                "toggleterm",
                direction = "horizontal",
                auto_scroll = true,
                quit_on_exit = "success",
              },
            }, -- passed into overseer.new_task
            on_new_task = function(task)
              require("overseer").open { enter = false, direction = "right" }
            end, -- called with the new overseer.Task before task:start
          },
          terminal = {
            -- must be unique and non-blank, or the plugin can't find/reuse the terminal
            name = "Main Terminal",
            prefix_name = "[CMakeTools]: ",
            split_direction = "horizontal", -- "horizontal", "vertical"
            split_size = 11,

            single_terminal_per_instance = true, -- one viewport, multiple windows
            single_terminal_per_tab = true, -- one viewport per tab
            keep_terminal_static_location = true,
            auto_resize = true,

            start_insert = false, -- enter insert mode on :CMakeRun
            focus = false,
            do_not_add_newline = false, -- leave the command uncommitted so it can be edited before pressing enter
          },
        },
      },
      cmake_runner = {
        name = "terminal",
        opts = {},
        default_opts = {
          quickfix = {
            show = "always", -- "always", "only_on_error"
            position = "belowright",
            size = 10,
            encoding = "utf-8",
            auto_close_when_success = true,
          },
          toggleterm = {
            direction = "float",
            close_on_exit = false,
            auto_scroll = true,
            singleton = true,
          },
          overseer = {
            new_task_opts = {
              strategy = {
                "toggleterm",
                direction = "horizontal",
                auto_scroll = true,
                quit_on_exit = "success",
              },
            },
            on_new_task = function(task) end,
          },
          terminal = {
            name = "Main Terminal",
            prefix_name = "[CMakeTools]: ",
            split_direction = "horizontal",
            split_size = 11,

            single_terminal_per_instance = true,
            single_terminal_per_tab = true,
            keep_terminal_static_location = true,
            auto_resize = true,

            start_insert = false,
            focus = false,
            do_not_add_newline = false,
          },
        },
      },
      cmake_notifications = {
        runner = { enabled = true },
        executor = { enabled = true },
        spinner = {
          "⠋",
          "⠙",
          "⠹",
          "⠸",
          "⠼",
          "⠴",
          "⠦",
          "⠧",
          "⠇",
          "⠏",
        },
        refresh_rate_ms = 100,
      },
      cmake_virtual_text_support = true, -- shows the target tied to the current file, right-aligned
      cmake_use_scratch_buffer = false,
    },
  },
}
