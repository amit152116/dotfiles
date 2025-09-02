---@type LazySpec
return {
  {
    dir = vim.fn.stdpath "config" .. "/lua/ros2-nvim",
    name = "ros2-nvim",
    enabled = true,
    specs = {
      "astroNvim/astrocore",
      opts = {
        mappings = {
          n = {
            ["<Leader>r"] = { desc = "ROS" },
            ["<Leader>ri"] = {
              desc = "ROS Interfaces",
            },
            ["<Leader>ra"] = {
              function() require("ros2-nvim").Actions() end,
              desc = "ROS Active Actions",
            },
            ["<Leader>rs"] = {
              desc = "ROS Active Services",
              function() require("ros2-nvim").Services() end,
            },
            ["<Leader>rt"] = {
              desc = "ROS Active Topics",
              function() require("ros2-nvim").Topics() end,
            },
            ["<Leader>rn"] = {
              desc = "ROS Active Nodes",
              function() require("ros2-nvim").Nodes() end,
            },
            ["<Leader>rp"] = {
              desc = "ROS Params",
              function() require("ros2-nvim").Params() end,
            },
            ["<Leader>rim"] = {
              function() require("ros2-nvim").InterfaceMsgs() end,
              desc = "ROS Msgs",
            },
            ["<Leader>ris"] = {
              function() require("ros2-nvim").InterfaceSrvs() end,
              desc = "ROS Srvs",
            },
            ["<Leader>ria"] = {
              function() require("ros2-nvim").InterfaceActions() end,
              desc = "ROS Actions",
            },
          },
        },
      },
    },
  },
}
