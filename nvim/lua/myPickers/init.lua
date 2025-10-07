local grep = require "myPickers.grep"
local Snacks = require "snacks"
local rosPicker = require("myPickers.ros").RosPicker

local M = {}
---@param opts snacks.picker.multigrep.Config
function M.multigrep(opts)
  opts = opts or {}
  opts.regex = (opts.regex == nil) and true or opts.regex
  opts.cwd = opts.cwd or vim.uv.cwd()
  if opts.title then
    opts.title = opts.title .. " (Multi-Grep)"
  else
    opts.title = "Multi-Grep (Smart)"
  end

  Snacks.picker.pick("grep", {
    search = opts.search,
    cwd = opts.cwd,
    title = opts.title,
    hidden = opts.hidden,
    ignored = opts.ignored,
    notify = false,
    finder = grep.createFinder(opts),
    matcher = {
      frecency = true,
      smartcase = true,
    },
    preview = grep.createPreview,
    actions = {
      replace = function(picker, _)
        if opts.replace_pattern then
          local sel = picker:selected()
          local results = #sel > 0 and sel or picker:items()
          if not results or #results == 0 then
            vim.notify(
              "No results to replace.",
              vim.log.levels.WARN,
              { title = "Multi Grep" }
            )
            return
          end
          grep.execute_replace(results, opts)
        end
      end,
    },
    win = {
      input = {
        keys = {
          ["<c-y>"] = {
            "replace",
            mode = { "n", "i" },
            desc = "Replace All",
          },
        },
      },
    },
  })
end

function M.rosTopics()
  rosPicker {
    title = "Active Topics",
    args = { "topic", "list" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      local pattern = "^/([^/]+)/?(.*)"
      local pkg, name = item.text:match(pattern)

      if pkg then
        if name ~= "" then
          item.display = string.format("%s [%s]", name, pkg)
        else
          item.display = pkg
        end
        return true
      end
      return false
    end,
    preview = {
      args = "topic info --verbose", -- Command to generate the preview
      ft = "yaml", -- Filetype for the preview window
    },
  }
end
function M.rosActions()
  rosPicker {
    title = "Active Actions",
    args = { "action", "list" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      local pattern = "^/([^/]+)/?(.*)"
      local pkg, name = item.text:match(pattern)

      if pkg then
        if name ~= "" then
          item.display = string.format("%s [%s]", name, pkg)
        else
          item.display = pkg
        end
        return true
      end
      return false
    end,
    preview = {
      args = "action info",
      ft = "yaml", -- Filetype for the preview window
    },
  }
end
function M.rosServics()
  rosPicker {
    title = "Active Services",
    args = { "service", "list" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      local pattern = "^/([^/]+)/?(.*)"
      local pkg, name = item.text:match(pattern)

      if pkg then
        if name ~= "" then
          item.display = string.format("%s [%s]", name, pkg)
        else
          item.display = pkg
        end
        return true
      end
      return false
    end,
    preview = {
      args = function(item)
        -- 1. Get the service type
        local service_type =
          vim.fn.system(string.format("ros2 service type %s", item.text))
        service_type = service_type:gsub('"', "") -- remove quotes
        service_type = service_type:gsub("%s+$", "") -- trim trailing newline
        item.preview_title = service_type

        return string.format("ros2 interface show %s", service_type)
      end,
      ft = "yaml", -- Filetype for the preview window
    },
  }
end
-- A function to open a picker for ROS2 nodes
function M.rosNodes()
  rosPicker {
    title = "Active Nodes",
    args = { "node", "list" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      local pattern = "^/([^/]+)/?(.*)"
      local pkg, name = item.text:match(pattern)

      if pkg then
        if name ~= "" then
          item.display = string.format("%s [%s]", name, pkg)
        else
          item.display = pkg
        end
        return true
      end
      return false
    end,
    preview = {
      args = "node info",
      ft = "yaml",
    },
  }
end

function M.rosParams()
  rosPicker {
    title = "Params",
    args = { "param", "list" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      item.text = item.text:gsub(":$", "") -- Remove leading slash
      local pattern = "^/([^/]+)/?(.*)"
      local pkg, name = item.text:match(pattern)

      if pkg then
        if name ~= "" then
          item.display = string.format("%s [%s]", name, pkg)
        else
          item.display = pkg
        end
        return true
      end
      return false
    end,
    preview = {
      ft = "yaml",
      args = "param dump",
    },
  }
end

function M.rosInterfaceMsgs()
  rosPicker {
    title = "Msgs",
    args = { "interface", "list", "--only-msgs" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      local pattern = "(.+)/msg/(.+)"
      local pkg, name = item.text:match(pattern)

      if pkg and name then
        item.display = string.format("%s [%s]", name, pkg)
        return true
      end
      return false
    end,
    preview = {
      ft = "yaml",
      args = "interface show",
    },
  }
end

function M.rosInterfaceSrvs()
  rosPicker {
    title = "Srvs",
    args = { "interface", "list", "--only-srvs" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      local pattern = "(.+)/srv/(.+)"
      local pkg, name = item.text:match(pattern)

      if pkg and name then
        item.display = string.format("%s [%s]", name, pkg)
        return true
      end
      return false
    end,
    preview = {
      ft = "yaml",
      args = "interface show",
    },
  }
end

function M.rosInterfaceActions()
  rosPicker {
    title = "Actions",
    args = { "interface", "list", "--only-actions" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      local pattern = "(.+)/action/(.+)"
      local pkg, name = item.text:match(pattern)

      if pkg and name then
        item.display = string.format("%s [%s]", name, pkg)
        return true
      end
      return false
    end,
    preview = {
      ft = "yaml",
      args = "interface show",
    },
  }
end
return M
