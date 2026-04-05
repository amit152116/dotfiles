local picker = require "myPlugins"
local Snacks = require "snacks"
---@type LazySpec
return {
  {
    dir = vim.fn.stdpath "config" .. "/lua/myPlugins",
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
                picker.grep {
                  cwd = "/opt/ros/humble",
                  matcher = {
                    frecency = true,
                  },
                }
              end,
              desc = "ROS Live Grep",
            },
            ["<Leader>ra"] = {
              function() picker.ros.Actions() end,
              desc = "ROS Active Actions",
            },
            ["<Leader>rs"] = {
              desc = "ROS Active Services",
              function() picker.ros.Servics() end,
            },
            ["<Leader>rt"] = {
              desc = "ROS Active Topics",
              function() picker.ros.Topics() end,
            },
            ["<Leader>rn"] = {
              desc = "ROS Active Nodes",
              function() picker.ros.Nodes() end,
            },
            ["<Leader>rp"] = {
              desc = "ROS Params",
              function() picker.ros.Params() end,
            },
            ["<Leader>rim"] = {
              function() picker.ros.InterfaceMsgs() end,
              desc = "ROS Msgs",
            },
            ["<Leader>ris"] = {
              function() picker.ros.InterfaceSrvs() end,
              desc = "ROS Srvs",
            },
            ["<Leader>ria"] = {
              function() picker.ros.InterfaceActions() end,
              desc = "ROS Actions",
            },
          },
        },
      },
    },
  },
}
