-- Required modules and telescope components
local pickers = require "telescope.pickers"
local finders = require "telescope.finders"
local conf = require("telescope.config").values
local actions = require "telescope.actions"
local action_state = require "telescope.actions.state"
local previewers = require "telescope.previewers"
local utils = require "myPlugins.utils"
local terminal = require "myPlugins.ros2-terminal"

-- Determine ROS distro and set cache directory
local cache_dir = vim.fn.stdpath "cache" .. "/ros2_nvim/"
vim.fn.mkdir(cache_dir, "p")

local M = {}

-- Default configuration
local config = {
    ros2_command = "ros2 ",
    cache_timeout = 300, -- default cache timeout (seconds)
    debounce_delay = 150, -- debounce delay for preview (ms)
}

-- Internal cache storage
local cache = {
    data = {},
    timestamps = {},
    cache_time = {},
}

-------------------------------------------------------
-- Caching Helpers
-------------------------------------------------------

-- Retrieve cached data if valid
local function get_cached(command, cache_timeout)
    cache.cache_time[command] = cache.cache_time[command] or cache_timeout

    if cache.data[command] and os.difftime(os.time(), cache.timestamps[command]) < cache.cache_time[command] then
        return cache.data[command]
    end
end

-- Update cache for a given command
local function set_cache(command, results)
    cache.data[command] = results
    cache.timestamps[command] = os.time()
end

-------------------------------------------------------
-- ROS2 Command Execution with Caching
-------------------------------------------------------

-- Asynchronous command execution with cache
local function get_ros2_data(command, callback, opts)
    -- Validate inputs
    if not command or command == "" then
        vim.notify("ROS2 Error: Empty command provided", vim.log.levels.ERROR)
        if callback then callback {} end
        return
    end

    if not callback then
        vim.notify("ROS2 Error: Null callback provided", vim.log.levels.ERROR)
        return
    end

    -- Initialize options
    opts = opts or {}
    local cache_timeout = opts.cache_timeout or config.cache_timeout

    -- Check cache if enabled
    if not opts.no_cache then
        local results = get_cached(command, cache_timeout)
        if results then
            -- Apply optional filter
            if opts.filter_callback then results = opts.filter_callback(results) end
            callback(results)
            return
        end
    end

    -- Execute ROS2 command
    utils.run_command(
        config.ros2_command .. command,
        vim.schedule_wrap(function(_, data)
            -- Process command output
            local results = {}

            if data then
                for i = 1, #data do
                    local line = utils.trim(data[i])
                    if line ~= "" then table.insert(results, line) end
                end
            end

            -- Update cache if enabled
            if not opts.no_cache and cache_timeout > 0 then set_cache(command, results) end

            -- Apply optional filter
            if opts.filter_callback then results = opts.filter_callback(results) end

            -- Return results via callback
            callback(results)
        end),
        nil
    )
end

-------------------------------------------------------
-- Previewer Creation with Debounce
-------------------------------------------------------

-- Cached previewer with debounce
local function create_ros_previewer(command, opts)
    local previewer = {
        timer = nil,
        current_bufnr = nil,
        current_cmd = nil,
        cache = {},
    }
    opts = opts or {}
    return previewers.new_buffer_previewer {
        title = "ROS2 Preview",
        get_buffer_by_name = function(_, entry) return command .. ":" .. entry.value end,

        define_preview = function(self, entry)
            -- Safely cancel previous timer
            if previewer.timer and vim.loop.is_closing(previewer.timer) == false then
                previewer.timer:stop()
                previewer.timer:close()
                previewer.timer = nil
            end

            local bufnr = self.state.bufnr
            previewer.current_bufnr = bufnr
            local cache_key = command .. ":" .. entry.value

            -- Show cached content immediately if available
            if previewer.cache[cache_key] then
                vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, previewer.cache[cache_key])
                return
            end

            vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, { "Loading..." })

            previewer.timer = vim.defer_fn(function()
                local cmd = command .. " " .. entry.value
                previewer.current_cmd = cmd

                get_ros2_data(cmd, function(data)
                    -- Only update if still relevant
                    if previewer.current_cmd ~= cmd then return end
                    if not vim.api.nvim_buf_is_valid(bufnr) then return end

                    -- Cache results for future use
                    previewer.cache[cache_key] = data

                    pcall(function()
                        vim.api.nvim_buf_set_lines(bufnr, 0, -1, false, data)
                        vim.api.nvim_buf_set_option(bufnr, "filetype", "rosmsg")
                    end)
                end, { no_cache = true })
            end, config.debounce_delay)
        end,

        teardown = function(self)
            -- Safer cleanup
            if previewer.timer and vim.loop.is_closing(previewer.timer) == false then
                previewer.timer:stop()
                previewer.timer:close()
                previewer.timer = nil
            end
            previewer.current_bufnr = nil
            previewer.current_cmd = nil
        end,

        clear_cache = function() previewer.cache = {} end,
    }
end

-------------------------------------------------------
-- Picker Creation Helper
-------------------------------------------------------

local function call_picker(opts)
    if vim.tbl_isempty(opts.results) then
        vim.notify("No " .. opts.prompt_title, vim.log.levels.INFO)
        return
    end
    pickers
        .new(opts.picker_opts or {}, {
            prompt_title = opts.prompt_title,
            finder = finders.new_table {
                results = opts.results,
                entry_maker = opts.entry_maker or function(entry)
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
    return true
end

-- Generic picker creator
local function create_ros_picker(opts)
    if opts.results then
        call_picker(opts)
        return true
    end

    get_ros2_data(opts.command, function(results)
        opts.results = results
        call_picker(opts)
    end, {
        filter_callback = opts.filter_callback,
        cache_timeout = opts.cache_timeout,
    })
end

-------------------------------------------------------
-- ROS Functions
-------------------------------------------------------

local function roslaunch_files()
    local home = vim.fn.expand "~"
    local system_cmd = [[fdfind '.*\.(launch\.py|launch\.xml|launch)$' /opt/ros/$ROS_DISTRO ]]
        .. home
        .. [[/Documents/aim_ros2 --max-depth 5 -t f]]

    local results = {}
    local path_patterns = {
        "/share/([^/]+)/launch/(.+)",
        "/src/([^/]+)/launch/(.+)",
        -- Add more patterns here if needed
    }
    -- Run system command
    local system = vim.fn.systemlist(system_cmd) or {}
    for _, file in ipairs(system) do
        local found = false
        for _, pattern in ipairs(path_patterns) do
            local pkg, launch = file:match(pattern)
            if pkg and launch then
                found = true
                results[pkg] = results[pkg] or {}
                table.insert(results[pkg], launch)
                break -- Found a match, move to next file
            end
        end
        if not found then
            results[file] = results[file] or {}
            table.insert(results[file], file)
        end
    end
    return results
end

local interface_ros_previewer = create_ros_previewer "interface show"

function M.services()
    create_ros_picker {
        command = "interface list --only-srvs",
        filter_callback = function(results)
            table.remove(results, 1)
            return results
        end,
        prompt_title = "ROS2 Services Interfaces",
        previewer = interface_ros_previewer,
        cache_timeout = 600,
    }
end

function M.actions()
    create_ros_picker {
        command = "interface list --only-actions",
        filter_callback = function(results)
            table.remove(results, 1)
            return results
        end,
        prompt_title = "ROS2 Actions Interfaces",
        cache_timeout = 600,
        previewer = interface_ros_previewer,
    }
end

function M.messages()
    create_ros_picker {
        command = "interface list --only-msgs",
        filter_callback = function(results)
            table.remove(results, 1)
            return results
        end,
        prompt_title = "ROS2 Message Interfaces",
        previewer = interface_ros_previewer,
        cache_timeout = 600,
        attach_mappings = function(prompt_bufnr)
            actions.select_default:replace(function()
                actions.close(prompt_bufnr)
                local selection = action_state.get_selected_entry()
                vim.notify("Selected Msg: " .. selection.value)
            end)
            return true
        end,
    }
end

function M.active_nodes()
    create_ros_picker {
        command = "node list",
        prompt_title = "ROS2 Active Nodes",
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
        prompt_title = "ROS2 Active Topics",
        previewer = create_ros_previewer "topic info",
        attach_mappings = function(prompt_bufnr)
            actions.select_default:replace(function()
                actions.close(prompt_bufnr)
                local selection = action_state.get_selected_entry()
                terminal.create_terminal("ros2 topic echo " .. selection.value)
            end)
            return true
        end,
    }
end

function M.exec_nodes()
    local pkgs = {}
    create_ros_picker {
        command = "pkg executables",
        filter_callback = function(results)
            for _, line in ipairs(results) do
                line = utils.trim(line)
                local pkg, node = line:match "^(%S+)%s+(%S+)$"
                if pkg and node then
                    pkgs[pkg] = pkgs[pkg] or {}
                    table.insert(pkgs[pkg], node)
                end
            end
            return vim.tbl_keys(pkgs)
        end,
        prompt_title = "ROS2 Executables Nodes",
        entry_maker = function(pkg)
            return {
                value = pkg,
                display = pkg,
                ordinal = pkg,
                nodes = pkgs[pkg],
            }
        end,
        attach_mappings = function(prompt_bufnr)
            actions.select_default:replace(function()
                actions.close(prompt_bufnr)
                local pkg_selected = action_state.get_selected_entry()

                create_ros_picker {
                    results = pkg_selected.nodes,
                    prompt_title = "Nodes inside " .. pkg_selected.value,
                    attach_mappings = function(prompt_buf)
                        actions.select_default:replace(function()
                            actions.close(prompt_buf)
                            local node_selected = action_state.get_selected_entry()
                            terminal.create_terminal("ros2 run " .. pkg_selected.value .. " " .. node_selected.value)
                        end)
                        return true
                    end,
                }
            end)
            return true
        end,
    }
end

function M.exec_launch_file()
    local pkgs = roslaunch_files()

    return create_ros_picker {
        results = vim.tbl_keys(pkgs),
        prompt_title = "ROS2 Launch files",
        entry_maker = function(pkg)
            return {
                value = pkg,
                display = pkg,
                ordinal = pkg,
                launch = pkgs[pkg],
            }
        end,
        attach_mappings = function(prompt_bufnr)
            actions.select_default:replace(function()
                actions.close(prompt_bufnr)
                local pkg_selected = action_state.get_selected_entry()

                create_ros_picker {
                    results = pkg_selected.launch,
                    prompt_title = "Launch Files",
                    attach_mappings = function(prompt_buf)
                        actions.select_default:replace(function()
                            actions.close(prompt_buf)
                            local launch_file = action_state.get_selected_entry()

                            local launch = launch_file.value:match "([^/]+%.launch%.[a-zA-Z0-9]+)$"
                            vim.notify(launch)
                            terminal.create_terminal("ros2 launch " .. pkg_selected.value .. " " .. launch)
                        end)
                        return true
                    end,
                }
            end)
            return true
        end,
    }
end

-- Parameter handling with cached hierarchy
function M.param()
    local nodes = {}

    create_ros_picker {
        command = "param list",
        filter_callback = function(results)
            local current_node = nil

            -- Single-pass processing
            for _, line in ipairs(results) do
                line = utils.trim(line)
                if line:match "^/" then
                    current_node = line:gsub(":$", "")
                    nodes[current_node] = {}
                elseif current_node then
                    table.insert(nodes[current_node], line)
                end
            end
            return vim.tbl_keys(nodes)
        end,
        prompt_title = "ROS2 Parameter Nodes",
        entry_maker = function(node)
            return {
                value = node,
                display = node,
                ordinal = node,
                params = nodes[node],
            }
        end,
        attach_mappings = function(prompt_bufnr)
            actions.select_default:replace(function()
                actions.close(prompt_bufnr)
                local selection = action_state.get_selected_entry()

                create_ros_picker {
                    results = selection.params,
                    prompt_title = "Parameters for " .. selection.value,
                    attach_mappings = function(prompt_buf)
                        actions.select_default:replace(function()
                            actions.close(prompt_buf)
                            local param = action_state.get_selected_entry().value
                            utils.run_command(
                                config.ros2_command .. "param get " .. selection.value .. " " .. param,
                                function(_, data) vim.notify(table.concat(data, "\n")) end
                            )
                        end)
                        return true
                    end,
                }
            end)
            return true
        end,
    }
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
