---@meta

--- This file provides a factory for creating snacks.nvim finders for ROS2 commands.
-- It uses a lazy-loading pattern for efficient preview generation.

local M = {}

---@class ros2.picker.PreviewOptions
---@field args string|table|function The command to generate a preview.
---   - string: e.g., "topic info". The selected item's text will be appended.
---   - table: e.g., {"topic", "info"}. The selected item's text will be appended.
---   - function: Takes the picker item and must return a command table (e.g., {"ros2", ...}).
---@field ft? string The filetype for the preview buffer (e.g., "yaml").

---@class ros2.picker.FinderOptions
---@field cmd string The base command to get the list of items (e.g., "topic list").
---@field args? string[]
---@field preview ros2.picker.PreviewOptions Options for generating the preview.
---@field formatter? fun(text: string): string A function to format the display text.

---@alias ros2.picker.Finder snacks.picker.finder

--- Creates a snacks.nvim finder for ROS2 commands.
--- This finder uses snacks.picker.source.proc to asynchronously
--- run a ROS2 command and populate the picker. It also uses
--- metatables for lazy-loading preview content.
---@param opts ros2.picker.FinderOptions The configuration options for the finder.
---@return ros2.picker.Finder
function M.create_finder(opts)
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
          if opts.formatter then
            item.display = opts.formatter(content)
          else
            item.display = content -- This is the text displayed in the picker list
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
                    opts.cmd .. " " .. preview_cmd_config .. " " .. content,
                    "%s+"
                  )
                elseif config_type == "function" then
                  preview_cmd_parts = preview_cmd_config(item)
                elseif config_type == "table" then
                  -- e.g., cmd={"topic", "info"}, content="/chatter" -> {"ros2", "topic", "info", "/chatter"}
                  preview_cmd_parts = vim.deepcopy(preview_cmd_config)
                  table.insert(preview_cmd_parts, 1, opts.cmd)
                  table.insert(preview_cmd_parts, content)
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

function M.RosPicker(opts)
  opts = opts or {}

  local title = string.format("  ROS2 %s", opts.title or opts.cmd)
  local finder = M.create_finder {
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
      else
        ctx.preview:set_lines {
          "Error: Could not generate preview.",
          "The 'ros2' command may have failed or returned no output.",
        }
      end
      return true
    end,
  }
end
-- A function to open a picker for ROS2 topics
function M.RosTopics()
  local finder = M.create_finder {
    cmd = "ros2",
    args = { "topic", "list" },
    formatter = function(text)
      -- local pkg, node = text:match "^/(.+)/(.+)"
      -- inspect(pkg, node)
      -- if pkg and node then return string.format("%s [%s]", node, pkg) end
      return string.sub(text, 2)
    end,
    preview = {
      args = "topic info --verbose", -- Command to generate the preview
      ft = "yaml", -- Filetype for the preview window
    },
  }

  snacks.picker.pick {
    finder = finder,
    title = "  ROS2 Topics",
    -- Re-creating your custom format from the original code
    format = function(item, _)
      if not item or not item.text then return { { "" } } end
      return {
        { "  ", "Comment" }, -- Using a standard highlight group like Comment
        { " " .. item.text, "Normal" },
      }
    end,
    -- Example of a custom action

    preview = function(ctx)
      local item = ctx.item
      if not item then return true end

      local preview_obj = item.preview

      if preview_obj and preview_obj.text then
        -- THE FIX IS HERE: Split the text string into a table of lines
        local lines = vim.split(preview_obj.text, "\n")
        ctx.preview:set_lines(lines)
        ctx.preview:highlight { ft = preview_obj.ft }
      else
        ctx.preview:set_lines {
          "Error: Could not generate preview.",
          "The 'ros2' command may have failed or returned no output.",
        }
      end

      return true
    end,
    actions = {
      ["<CR>"] = function(self, item)
        self:close()
        -- The preview data is available in item.data
        snacks.notify.notify(
          "Selected: " .. item.text,
          { title = "ROS2 Topics" }
        )
      end,
    },
  }
end

-- A function to open a picker for ROS2 nodes
function M.RosNodes()
  local finder = M.create_finder {
    cmd = "ros2",
    args = { "node", "list" },
    preview = {
      args = "node info",
      ft = "yaml",
    },
    formatter = function(text) return string.sub(text, 2) end,
  }

  snacks.picker.pick {
    title = "  ROS2 Nodes",
    finder = finder,
    format = function(item, _)
      if not item or not item.display then return { { "" } } end

      -- Return TWO text segments, each with a different highlight group
      return {
        { "  ", "Icon" },
        { item.display, "Normal" }, -- Highlight the topic name
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
      else
        ctx.preview:set_lines {
          "Error: Could not generate preview.",
          "The 'ros2' command may have failed or returned no output.",
        }
      end

      return true
    end,
    -- You can add other picker options here as needed
  }
end
function M.RosParams()
  M.RosPicker {
    title = "Params",
    args = { "param", "list" },
    formatter = function(text) return text end,
    preview = {
      ft = "yaml",
      args = "param dump",
    },
  }
end

function M.RosMsgs()
  M.RosPicker {
    title = "Msgs",
    args = { "interface", "list", "--only-msgs" },
    formatter = function(text)
      local pattern = "(.+)/msg/(.+)"
      local pkg, name = text:match(pattern)

      if pkg and name then return string.format("%s [%s]", name, pkg) end
      return text
    end,
    preview = {
      ft = "yaml",
      args = "interface show",
    },
  }
end

function M.RosSrvs()
  M.RosPicker {
    title = "Srvs",
    args = { "interface", "list", "--only-srvs" },
    formatter = function(text)
      local pattern = "(.+)/srv/(.+)"
      local pkg, name = text:match(pattern)

      if pkg and name then return string.format("%s [%s]", name, pkg) end
      return text
    end,
    preview = {
      ft = "yaml",
      args = "interface show",
    },
  }
end

function M.RosActions()
  M.RosPicker {
    title = "Actions",
    args = { "interface", "list", "--only-actions" },
    formatter = function(text)
      local pattern = "(.+)/action/(.+)"
      local pkg, name = text:match(pattern)

      if pkg and name then return string.format("%s [%s]", name, pkg) end
      return text
    end,
    preview = {
      ft = "yaml",
      args = "interface show",
    },
  }
end
M.RosActions()

return M
