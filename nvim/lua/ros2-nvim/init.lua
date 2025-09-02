---@meta

--- This file provides a factory for creating snacks.nvim finders for ROS2 commands.
-- It uses a lazy-loading pattern for efficient preview generation.

local M = {}

---@alias ros2.picker.PreviewArgsFunc fun(item: snacks.picker.finder.Item): table

---@class ros2.picker.PreviewOptions
---@field args string|string[]|ros2.picker.PreviewArgsFunc The command to generate a preview.
---   - string: e.g., "topic info". The selected item's text will be appended.
---   - table: e.g., {"topic", "info"}. The selected item's text will be appended.
---   - function: Takes the picker item and must return a command table (e.g., {"ros2", ...}).
---@field ft? string The filetype for the preview buffer (e.g., "yaml").

---@class ros2.picker.FinderOptions
---@field cmd string The base command to get the list of items (e.g., "topic list").
---@field args? string[]
---@field preview ros2.picker.PreviewOptions Options for generating the preview.
---@field formatter? fun(item: snacks.picker.finder.Item): boolean A function to format the display text.

---@alias ros2.picker.Finder snacks.picker.finder

--- Creates a snacks.nvim finder for ROS2 commands.
--- This finder uses snacks.picker.source.proc to asynchronously
--- run a ROS2 command and populate the picker. It also uses
--- metatables for lazy-loading preview content.
---@param opts ros2.picker.FinderOptions The configuration options for the finder.
---@return ros2.picker.Finder
local function create_finder(opts)
  opts = opts or {}
  opts.preview = opts.preview or {}

  ---@param picker_opts snacks.picker.proc.Config
  ---@type snacks.picker.finder
  return function(picker_opts, ctx)
    return require("snacks.picker.source.proc").proc({
      picker_opts,
      {
        cmd = opts.cmd,
        args = opts.args,
        ---@param item snacks.picker.finder.Item
        transform = function(item)
          local content = vim.trim(item.text)
          if content == "" or content:match "^---$" then
            return false -- Skip empty or separator lines often found in ROS2 output
          end

          item.text = content
          item.display = content -- This is the text displayed in the picker list
          if opts.formatter then
            if opts.formatter(item) == false then return false end
          end

          setmetatable(item, {
            __index = function(_, k)
              -- Lazy-load preview data only when 'data' or 'preview' is accessed for an item.
              if k == "data" then
                local preview_cmd_config = opts.preview.args
                if not preview_cmd_config then
                  rawset(item, "data", "No preview command configured.")
                  return rawget(item, "data")
                end

                local preview_cmd_parts
                local config_type = type(preview_cmd_config)

                if config_type == "string" then
                  -- e.g., cmd="topic info", content="/chatter" -> {"ros2", "topic", "info", "/chatter"}
                  preview_cmd_parts = vim.split(
                    opts.cmd .. " " .. preview_cmd_config .. " " .. item.text,
                    "%s+"
                  )
                elseif config_type == "function" then
                  preview_cmd_parts = preview_cmd_config(item)
                elseif config_type == "table" then
                  -- e.g., cmd={"topic", "info"}, content="/chatter" -> {"ros2", "topic", "info", "/chatter"}
                  preview_cmd_parts = vim.deepcopy(preview_cmd_config)
                  table.insert(preview_cmd_parts, 1, opts.cmd)
                  table.insert(preview_cmd_parts, item.text)
                end
                if not preview_cmd_parts then
                  rawset(
                    item,
                    "data",
                    "Error: Could not construct preview command."
                  )
                  return rawget(item, "data")
                end

                -- Execute the command synchronously to get the preview data.
                -- This blocks, but only for the preview of a single item, which is acceptable.
                local data = vim.fn.system(preview_cmd_parts)
                if vim.v.shell_error ~= 0 then
                  local err_msg = "Error running preview command:\n"
                    .. table.concat(preview_cmd_parts, " ")
                    .. "\n\n"
                    .. data
                  rawset(item, "data", err_msg)
                  return err_msg
                end
                -- Cache the result in the item table itself to avoid re-running the command
                rawset(item, "data", data)
                return data
              elseif k == "preview" then
                -- This is called by the snacks previewer for the highlighted item.
                -- It implicitly accesses `item.data`, triggering the 'data' logic above if needed.
                return {
                  text = item.data,
                  ft = opts.preview.ft or "markdown",
                }
              end
            end,
          })
          return true -- Keep the item
        end,
      },
    }, ctx)
  end
end

local snacks = require "snacks"

---@class ros2.picker.Picker: ros2.picker.FinderOptions
---@field title string  -- optional title for the picker
---
---@param opts ros2.picker.Picker The configuration options for the finder.
function M.RosPicker(opts)
  opts = opts or {}

  local title = string.format("   %s", opts.title or opts.cmd)
  local finder = create_finder {
    cmd = "ros2",
    args = opts.args,
    formatter = opts.formatter,
    preview = opts.preview,
  }
  snacks.picker.pick {
    title = title,
    finder = finder,
    format = function(item, _)
      if not item or not item.text then return { { "" } } end
      return {
        { "  ", "Comment" }, -- Using a standard highlight group like Comment
        { " " .. item.display, "Normal" },
      }
    end,
    preview = function(ctx)
      local item = ctx.item
      if not item then return true end

      local preview_obj = item.preview

      if preview_obj and preview_obj.text then
        -- THE FIX IS HERE: Split the text string into a table of lines
        local lines = vim.split(preview_obj.text, "\n")
        table.remove(lines, 1)
        ctx.preview:set_lines(lines)
        ctx.preview:highlight { ft = preview_obj.ft }
        if item.preview_title then
          ctx.preview:set_title(item.preview_title)
        else
          ctx.preview:set_title(item.display)
        end
      else
        ctx.preview:set_lines {
          "Error: Could not generate preview.",
          "The 'ros2' command may have failed or returned no output.",
        }
      end
      return true
    end,
    actions = {
      notify = function(self, item)
        self:close()
        local text = item.preview_title or item.text
        -- The preview data is available in item.data
        snacks.notify.notify(item.data, { title = " ( " .. text .. " )" })
      end,
    },
    win = {
      input = {
        keys = {
          ["<CR>"] = { "notify", mode = { "n", "i" } },
        },
      },
      list = {
        keys = {
          ["<CR>"] = { "notify", mode = { "n", "i" } },
        },
      },
    },
  }
end

-- A function to open a picker for ROS2 topics
function M.Topics()
  M.RosPicker {
    title = "Active Topics",
    args = { "topic", "list" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      local pattern = "^/([^/]+)/(.+)"
      local pkg, name = item.text:match(pattern)

      if pkg and name then
        item.display = string.format("%s [%s]", name, pkg)
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
function M.Actions()
  M.RosPicker {
    title = "Active Actions",
    args = { "action", "list" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      local pattern = "^/([^/]+)/(.+)"
      local pkg, name = item.text:match(pattern)

      if pkg and name then
        item.display = string.format("%s [%s]", name, pkg)
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
function M.Services()
  M.RosPicker {
    title = "Active Services",
    args = { "service", "list" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      local pattern = "^/([^/]+)/(.+)"
      local pkg, name = item.text:match(pattern)

      if pkg and name then
        item.display = string.format("%s [%s]", name, pkg)
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
function M.Nodes()
  M.RosPicker {
    title = "Active Nodes",
    args = { "node", "list" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      local pattern = "^/([^/]+)/(.+)"
      local pkg, name = item.text:match(pattern)

      if pkg and name then
        item.display = string.format("%s [%s]", name, pkg)
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

function M.Params()
  M.RosPicker {
    title = "Params",
    args = { "param", "list" },
    ---@param item snacks.picker.finder.Item
    formatter = function(item)
      item.text = item.text:gsub(":$", "") -- Remove leading slash
      local pattern = "^/([^/]+)/(.+)"
      local pkg, name = item.text:match(pattern)

      if pkg and name then
        item.display = string.format("%s [%s]", name, pkg)
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

function M.InterfaceMsgs()
  M.RosPicker {
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

function M.InterfaceSrvs()
  M.RosPicker {
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

function M.InterfaceActions()
  M.RosPicker {
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
