-- File: lua/ros2-telescope/init.lua

local pickers = require "telescope.pickers"
local finders = require "telescope.finders"
local conf = require("telescope.config").values
local actions = require "telescope.actions"
local action_state = require "telescope.actions.state"
local previewers = require "telescope.previewers"

local M = {}

local config = {
    ros2_command = "ros2",
    -- Add other telescope-specific config if needed
}

-- Helper function to execute ROS2 commands
local function get_ros2_data(command, callback)
    -- vim.notify("Running command: " .. config.ros2_command .. " " .. command, vim.log.levels.INFO)
    vim.fn.jobstart(config.ros2_command .. " " .. command, {
        stdout_buffered = true,
        on_stdout = function(_, data)
            if data then
                local results = {}
                -- vim.notify("ROS2 Output: " .. table.concat(data, "\n"), vim.log.levels.INFO)
                table.remove(data, 1) -- Remove the first line which is the command itself
                for _, line in ipairs(data) do
                    if line ~= "" then table.insert(results, line) end
                end
                callback(results)
            end
        end,
        -- on_stderr = function(_, err) vim.notify("ROS2 Error: " .. table.concat(err, "\n"), vim.log.levels.ERROR) end,
    })
end

-- Previewer for ROS2 message definitions
local ros_previewer = previewers.new_buffer_previewer {
    title = "ROS2 Definition Preview",
    get_buffer_by_name = function(_, entry) return entry.value end,

    define_preview = function(self, entry)
        local bufnr = self.state.bufnr
        vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, { "Loading..." })

        get_ros2_data("interface show" .. entry.value, function(data)
            vim.schedule(function()
                if vim.api.nvim_buf_is_valid(bufnr) then
                    vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, data)
                    vim.api.nvim_buf_set_option(bufnr, "filetype", "rosmsg")
                end
            end)
        end)
    end,
}

-- Generic picker creator
local function create_ros_picker(opts)
    get_ros2_data(opts.command, function(results)
        pickers
            .new(opts.picker_opts or {}, {
                prompt_title = opts.prompt_title,
                finder = finders.new_table {
                    results = results,
                    entry_maker = function(entry)
                        return {
                            value = entry,
                            display = entry,
                            ordinal = entry,
                        }
                    end,
                },
                previewer = opts.previewer,
                sorter = conf.generic_sorter(opts.picker_opts),
                attach_mappings = opts.attach_mappings,
            })
            :find()
    end)
end

function M.topics()
    create_ros_picker {
        command = "topic list",
        prompt_title = "ROS2 Topics",
        previewer = ros_previewer,
        attach_mappings = function(prompt_bufnr)
            actions.select_default:replace(function()
                actions.close(prompt_bufnr)
                local selection = action_state.get_selected_entry()
                vim.notify("Selected topic: " .. selection.value)
                -- Add custom action here
            end)
            return true
        end,
    }
end

function M.services()
    create_ros_picker {
        command = "interface list --only-srvs",
        prompt_title = "ROS2 Services",
        previewer = ros_previewer,
    }
end

function M.actions()
    create_ros_picker {
        command = "interface list --only-actions",
        prompt_title = "ROS2 Actions",
        previewer = ros_previewer,
    }
end

function M.nodes()
    create_ros_picker {
        command = "node list",
        prompt_title = "ROS2 Nodes",
        attach_mappings = function(prompt_bufnr)
            actions.select_default:replace(function()
                actions.close(prompt_bufnr)
                local selection = action_state.get_selected_entry()
                require("ros2-nvim").node_info(selection.value)
            end)
            return true
        end,
    }
end

function M.messages()
    create_ros_picker {
        command = "interface list --only-msgs",
        prompt_title = "ROS2 Message Types",
        previewer = ros_previewer,
        attach_mappings = function(prompt_bufnr)
            actions.select_default:replace(function()
                actions.close(prompt_bufnr)
                local selection = action_state.get_selected_entry()
                require("ros2-nvim").show_message_definition(selection.value)
            end)
            return true
        end,
    }
end

function M.setup(user_config)
    config = vim.tbl_deep_extend("force", config, user_config or {})

    require("telescope").register_extension {
        exports = {
            ros2_topics = M.topics,
            ros2_services = M.services,
            ros2_actions = M.actions,
            ros2_nodes = M.nodes,
            ros2_messages = M.messages,
        },
    }
end

return M
