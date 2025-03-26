-- File: lua/ros2-telescope/init.lua

local pickers = require "telescope.pickers"
local finders = require "telescope.finders"
local conf = require("telescope.config").values
local actions = require "telescope.actions"
local action_state = require "telescope.actions.state"
local previewers = require "telescope.previewers"
local debugger = require "myPlugins.debugger"
local M = {}

local config = {
    ros2_command = "ros2",
    -- Add other telescope-specific config if needed
}

-- Helper function to execute ROS2 commands
local function get_ros2_data(command, callback, top_pop, filter)
    -- vim.notify("Running command: " .. config.ros2_command .. " " .. command, vim.log.levels.INFO)
    vim.fn.jobstart(config.ros2_command .. " " .. command, {
        stdout_buffered = true,
        on_stdout = function(_, data)
            if filter then
                local results = filter(data)
                callback(results)
            elseif data then
                local results = {}
                -- vim.notify("ROS2 Output: " .. table.concat(data, "\n"), vim.log.levels.INFO)
                if top_pop then
                    table.remove(data, 1) -- Remove the first line which is the command itself
                end
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
local function create_ros_previewer(command)
    return previewers.new_buffer_previewer {
        title = "ROS2 Definition Preview",
        get_buffer_by_name = function(_, entry) return entry.value end,

        define_preview = function(self, entry)
            local bufnr = self.state.bufnr
            vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, { "Loading..." })

            local cmd = command .. " " .. entry.value
            get_ros2_data(cmd, function(data)
                vim.schedule(function()
                    if vim.api.nvim_buf_is_valid(bufnr) then
                        vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, data)
                        vim.api.nvim_buf_set_option(bufnr, "filetype", "rosmsg")
                    end
                end)
            end)
        end,
    }
end

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
    end, opts.top_pop, opts.filter)
end

local interface_ros_previewer = create_ros_previewer "interface show"
function M.services()
    create_ros_picker {
        command = "interface list --only-srvs",
        prompt_title = "ROS2 Services",
        previewer = interface_ros_previewer,
        top_pop = true,
    }
end

function M.actions()
    create_ros_picker {
        command = "interface list --only-actions",
        prompt_title = "ROS2 Actions",
        previewer = interface_ros_previewer,
        top_pop = true,
    }
end

function M.messages()
    create_ros_picker {
        command = "interface list --only-msgs",
        prompt_title = "ROS2 Message Types",
        previewer = interface_ros_previewer,
        attach_mappings = function(prompt_bufnr)
            actions.select_default:replace(function()
                actions.close(prompt_bufnr)
                local selection = action_state.get_selected_entry()
                vim.notify("Selected Msg: " .. selection.value)
            end)
            return true
        end,
        top_pop = true,
    }
end

function M.active_nodes()
    create_ros_picker {
        command = "node list",
        prompt_title = "ROS2 Nodes",
        previewer = create_ros_previewer "node info",
        attach_mappings = function(prompt_bufnr)
            actions.select_default:replace(function()
                actions.close(prompt_bufnr)
                local selection = action_state.get_selected_entry()
                vim.notify("Selected Node: " .. selection.value)
            end)
            return true
        end,
    }
end

function M.active_topics()
    create_ros_picker {
        command = "topic list",
        prompt_title = "ROS2 Topics",
        previewer = create_ros_previewer "topic info",
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

function M.exec_nodes()
    create_ros_picker {
        command = "pkg executables",
        prompt_title = "ROS2 Executables Nodes",
        attach_mappings = function(prompt_bufnr)
            actions.close(prompt_bufnr)
            local selection = action_state.get_selected_entry()
            vim.notify("Selected topic: " .. selection.value)
        end,
    }
end

function M.exec_launch_file()
    create_ros_picker {
        command = "pkg executables",
        prompt_title = "ROS2 Executables Nodes",
        attach_mappings = function(prompt_bufnr)
            actions.close(prompt_bufnr)
            local selection = action_state.get_selected_entry()
            vim.notify("Selected topic: " .. selection.value)
        end,
    }
end
local function get_params_list(node)
    create_ros_picker {
        command = "param list " .. node,
        prompt_title = "ROS2 Params For " .. node,
        previewer = create_ros_previewer "param get" .. node,
        attach_mappings = function(prompt_bufnr)
            actions.select_default:replace(function()
                actions.close(prompt_bufnr)
                local selection = action_state.get_selected_entry()
                vim.notify("Selected Param: " .. selection.value)
                -- Add custom action here
            end)
            return true
        end,
    }
end
function M.param()
    local nodes = {}
    local current_node = nil
    local nodes_list = {}
    create_ros_picker {
        command = "param list",
        prompt_title = "ROS2 Params",

        filter = function(raw_output)
            -- @param raw_output string[] List of Params with Nodes
            -- @return nodes_list string[] List of Nodes which have params
            -- Parse node hierarchy from raw output
            debugger.show_debug(raw_output)
            for _, line in ipairs(raw_output) do
                if line:match "^/" then table.insert(nodes_list, line) end
            end
            debugger.show_debug(nodes_list)
            return nodes_list
        end,
        attach_mappings = function(prompt_bufnr)
            actions.select_default:replace(function()
                actions.close(prompt_bufnr)
                local selection = action_state.get_selected_entry()
                get_params_list(selection.value)
                -- M.show_node_parameters(selection.value, nodes[selection.value])
            end)
            return true
        end,
    }
end

-- Update the M.param function and add helper functions
function M.param2()
    get_ros2_data("param list", function(raw_output)
        -- First level picker - nodes
        pickers
            .new({}, {
                prompt_title = "ROS2 Parameter Nodes",
                finder = finders.new_table {
                    results = vim.tbl_keys(nodes),
                    entry_maker = function(entry)
                        return {
                            value = entry,
                            display = entry,
                            ordinal = entry,
                        }
                    end,
                },
                sorter = conf.generic_sorter {},
                previewer = previewers.new_buffer_previewer {
                    title = "Node Parameters Preview",
                    define_preview = function(self, entry)
                        local bufnr = self.state.bufnr
                        local params = nodes[entry.value]
                        vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, {
                            "Node: " .. entry.value,
                            "Parameters (" .. #params .. "):",
                            unpack(params),
                        })
                    end,
                },
            })
            :find()
    end)
end

function M.show_node_parameters(node_name, parameters)
    -- Second level picker - parameters
    pickers
        .new({}, {
            prompt_title = "Parameters for " .. node_name,
            finder = finders.new_table {
                results = parameters,
                entry_maker = function(entry)
                    return {
                        value = entry,
                        display = entry,
                        ordinal = entry,
                        node = node_name,
                    }
                end,
            },
            sorter = conf.generic_sorter {},
            previewer = previewers.new_buffer_previewer {
                title = "Parameter Value Preview",
                define_preview = function(self, entry)
                    local bufnr = self.state.bufnr
                    vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, { "Loading..." })

                    get_ros2_data(string.format("param get %s %s", entry.node, entry.value), function(data)
                        vim.schedule(function()
                            if vim.api.nvim_buf_is_valid(bufnr) then
                                vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, {
                                    string.format("Node: %s", entry.node),
                                    string.format("Parameter: %s", entry.value),
                                    "Value:",
                                    unpack(data),
                                })
                            end
                        end)
                    end)
                end,
            },
            attach_mappings = function(prompt_bufnr)
                actions.select_default:replace(function()
                    actions.close(prompt_bufnr)
                    local selection = action_state.get_selected_entry()
                    vim.notify(string.format("Selected parameter: %s/%s", selection.node, selection.value))
                end)
                return true
            end,
        })
        :find()
end

function M.setup(user_config)
    config = vim.tbl_deep_extend("force", config, user_config or {})

    require("telescope").register_extension {
        exports = {
            ros2_topics = M.active_topics,
            ros2_services = M.services,
            ros2_actions = M.actions,
            ros2_nodes = M.active_nodes,
            ros2_messages = M.messages,
            ros2_params = M.param,
            ros2_exec_node = M.exec_nodes,
            ros2_exec_launch = M.exec_launch_file,
        },
    }
end

return M
