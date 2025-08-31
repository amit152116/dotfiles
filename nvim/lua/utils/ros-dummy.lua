local JobRunner = require "utils.jobRunner"
local Snacks = require "snacks"
local M = {}

---@alias Ros2Picker fun(cmd: string, opts?: Ros2PickerOpts)

---@alias Ros2Transform fun(pattern: string, item: JobRunnerItem): boolean

---@alias Ros2InterfaceTransform fun(type: Ros2InterfaceType, item: JobRunnerItem): boolean

---@alias Ros2PreviewCmd fun(choice:JobRunnerItem):JobRunnerCmd
---@alias Ros2Confirm fun(item: snacks.picker.Item)

---@alias Ros2InterfaceType
---| "msg"     # Message definition
---| "srv"     # Service definition
---| "action"  # Action definition

---@class Ros2PickerOpts: JobRunnerOpts
---@field title? string
---@field confirm? Ros2Confirm
---@field preview_cmd? JobRunnerCmd|Ros2PreviewCmd
-- NOTE: we deliberately remove JobRunnerOpts.on_complete
--       since ros2_picker defines it internally.
---@field on_complete nil

---ROS2 picker wrapper
---@type Ros2Picker
local function ros2_picker(cmd, opts)
  opts = opts or {}

  local title = string.format("  ROS2 %s", opts.title or cmd)

  opts.on_complete = function(results)
    if not results or #results == 0 then return end

    Snacks.picker.pick {
      items = results,
      title = title,
      format = function(item, _)
        if not item or not item.display then return { { "" } } end

        -- Return TWO text segments, each with a different highlight group
        return {
          { "  ", "Icon" },
          { item.display, "PickWin" }, -- Highlight the topic name
        }
      end,
      layout = {
        preset = "default", -- or your preferred layout
        cycle = true,
        layout = {
          title = title,
        },
      },
      confirm = function(self, item, _)
        self:close()
        if opts.confirm and type(opts.confirm) == "function" then
          opts.confirm(item)
        else
          Snacks.notify.notify(item.content, {
            title = title,
          })
        end
      end,
      preview = function(ctx)
        ctx.preview:set_lines { "Loading.." }
        if not ctx.item then return true end
        local item = ctx.item

        local prev_cmd = opts.preview_cmd

        local type = type(prev_cmd)
        if type == "string" then
          prev_cmd = { "ros2", prev_cmd, item.value }
        elseif type == "function" then
          prev_cmd = prev_cmd(item) or ""
        elseif type == "table" then
          prev_cmd = table.insert(prev_cmd, 1, "ros")
        end

        if prev_cmd == nil then return end

        JobRunner.run(prev_cmd, {
          on_complete = function(preview)
            if preview then
              ctx.item.content = preview
              ctx.preview:set_lines(preview)
              ctx.preview:highlight { ft = "markdown" }
            else
              ctx.preview:notify("No preview", "error")
            end
          end,
        })
      end,
    }
  end

  JobRunner.stream("ros2 " .. cmd, opts)
end

---Transforms a JobRunnerItem by extracting display info from its value.
---@type Ros2Transform
local function applyTransform(pattern, item)
  local pkg, name = item.value:match(pattern)

  if pkg and name then
    item.display = string.format("%s [%s]", name, pkg)
    return true
  end
  return false
end

---Transforms a ROS2 interface string into a formatted display.
---@type Ros2InterfaceTransform
local function interfaceTransform(type, item)
  -- Build the pattern dynamically
  local pattern = "(.+)/" .. type .. "/(.+)"
  return applyTransform(pattern, item)
end

function M.InterfaceAction()
  ros2_picker("interface list --only-actions", {
    preview_cmd = "interface show",
    title = "Interface(msg)",
    transform = function(item) return interfaceTransform("action", item) end,
  })
end
function M.InterfaceMsg()
  ros2_picker("interface list --only-msgs", {
    preview_cmd = "interface show",
    title = "Interface(msg)",
    transform = function(item) return interfaceTransform("msg", item) end,
  })
end

function M.InterfaceSrv()
  ros2_picker("interface list --only-srvs", {
    preview_cmd = "interface show",
    title = "Interface(msg)",
    transform = function(item) return interfaceTransform("srv", item) end,
  })
end

function M.Topics()
  ros2_picker("topic list", {
    preview_cmd = "topic info --verbose",
    title = "Active Topic",
  })
end

function M.Services()
  ros2_picker("service list", {
    preview_cmd = function(choice)
      return string.format(
        "ros2 service tyendpe %s | xargs -I {} sh -c 'echo \"{}\\n\"; ros2 interface show {}'",
        choice.value
      )
    end,
    title = "Active Service",
  })
end

function M.Params()
  ros2_picker("param list", {
    preview_cmd = "param dump",
    title = "Active Param",
    transform = function(item)
      item.value = item.value:gsub(":%s*$", "") -- remove trailing colon
      return applyTransform("/(.+)/(.+)", item)
    end,
  })
end

function M.Nodes()
  ros2_picker("pkg list", {
    preview_cmd = "pkg executables",
    title = "Package",
  })
end

M.Params()
return M
