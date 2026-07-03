return {
  {
    "stevearc/overseer.nvim",
    cmd = { "OverseerRun", "OverseerToggle", "OverseerBuild" },
    event = "VeryLazy",
    config = function(_, opts)
      local overseer = require "overseer"
      overseer.setup(opts)
      local function is_ros_ws()
        local cwd = vim.fn.getcwd()
        return vim.fn.isdirectory(cwd .. "/src") == 1
          and vim.fn.isdirectory(cwd .. "/install") == 1
      end

      overseer.register_template {
        name = "ros2 run",
        condition = { callback = is_ros_ws },
        params = {
          pkg = { type = "string", name = "Package", order = 1 },
          node = { type = "string", name = "Node", order = 2 },
          args = {
            type = "string",
            name = "Args (optional)",
            optional = true,
            order = 3,
          },
        },
        builder = function(p)
          local cmd = { "ros2", "run", p.pkg, p.node }
          if p.args and p.args ~= "" then
            vim.list_extend(cmd, vim.split(p.args, " ", { plain = true }))
          end
          return { cmd = cmd, components = { "default", "on_complete_notify" } }
        end,
      }
      overseer.register_template {
        name = "ros2 launch",
        condition = { callback = is_ros_ws },
        params = {
          pkg = { type = "string", name = "Package", order = 1 },
          launch = { type = "string", name = "Launch file", order = 2 },
          args = {
            type = "string",
            name = "Args (optional)",
            optional = true,
            order = 3,
          },
        },
        builder = function(p)
          local cmd = { "ros2", "launch", p.pkg, p.launch }
          if p.args and p.args ~= "" then
            vim.list_extend(cmd, vim.split(p.args, " ", { plain = true }))
          end
          return { cmd = cmd, components = { "default", "on_complete_notify" } }
        end,
      }
    end,
    opts = {
      -- populates quickfix so ]q/[q jump to errors
      task_list = {
        direction = "bottom",
        min_height = 10,
        max_height = 15,
        bindings = {
          ["<CR>"] = "RunAction",
          ["<C-e>"] = "Edit",
          ["o"] = "Open",
          ["q"] = "close",
        },
      },
    },
    specs = {
      {
        "AstroNvim/astrocore",
        opts = {
          mappings = {
            n = {
              ["<Leader>lo"] = { "<cmd>OverseerToggle<cr>", desc = "Task list" },
              ["<Leader>lO"] = { "<cmd>OverseerRun<cr>", desc = "Run task" },
              ["<Leader>lm"] = {
                function()
                  vim.ui.input({ prompt = "make args: " }, function(args)
                    if not args then return end
                    local overseer = require "overseer"
                    local task = overseer.new_task {
                      name = "make " .. args,
                      cmd = "make",
                      args = vim.split(args, " ", { plain = true }),
                      components = { { "on_output_quickfix", open = false, items_only = true }, "default" },
                    }
                    task:subscribe("on_complete", function(_, status)
                      vim.schedule(function()
                        if status == overseer.STATUS.SUCCESS then
                          overseer.close()
                        else
                          local items = vim.fn.getqflist()
                          if #items > 0 then vim.cmd "copen" end
                        end
                      end)
                    end)
                    task:start()
                  end)
                end,
                desc = "Async make with args",
              },
            },
          },
        },
      },
    },
  },
  -- switch cmake-tools executor to overseer for async builds
  {
    "Civitasv/cmake-tools.nvim",
    opts = {
      cmake_executor = { name = "overseer" },
      cmake_runner = { name = "overseer" },
    },
  },
}
