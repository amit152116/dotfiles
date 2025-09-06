local picker = require "myPickers"
local Snacks = require "snacks"
---@type LazySpec
return {
  {
    dir = vim.fn.stdpath "config" .. "/lua/myPickers",
    name = "ros2-nvim",
    dependencies = {
      "AstroNvim/astrocore", -- if it really depends on astrocore
    },
    specs = {
      "astroNvim/astrocore",
      opts = {
        mappings = {
          n = {
            ["<Leader>r"] = { desc = "ROS" },
            ["<Leader>ri"] = {
              desc = "ROS Interfaces",
            },

            -- Find all ROS Distro files
            ["<Leader>rf"] = {
              function()
                Snacks.picker.files {
                  cwd = "/opt/ros/humble",
                  matcher = {
                    frecency = true,
                  },
                }
              end,
              desc = "ROS files",
            },

            -- Find words in ROS distro files
            ["<Leader>rw"] = {
              function()
                picker.multigrep {
                  cwd = "/opt/ros/humble",
                  matcher = {
                    frecency = true,
                  },
                }
              end,
              desc = "ROS Live Grep",
            },
            ["<Leader>ra"] = {
              function() picker.rosActions() end,
              desc = "ROS Active Actions",
            },
            ["<Leader>rs"] = {
              desc = "ROS Active Services",
              function() picker.rosServics() end,
            },
            ["<Leader>rt"] = {
              desc = "ROS Active Topics",
              function() picker.rosTopics() end,
            },
            ["<Leader>rn"] = {
              desc = "ROS Active Nodes",
              function() picker.rosNodes() end,
            },
            ["<Leader>rp"] = {
              desc = "ROS Params",
              function() picker.rosParams() end,
            },
            ["<Leader>rim"] = {
              function() picker.rosInterfaceMsgs() end,
              desc = "ROS Msgs",
            },
            ["<Leader>ris"] = {
              function() picker.rosInterfaceSrvs() end,
              desc = "ROS Srvs",
            },
            ["<Leader>ria"] = {
              function() picker.rosInterfaceActions() end,
              desc = "ROS Actions",
            },
          },
        },
      },
    },
  },
}
