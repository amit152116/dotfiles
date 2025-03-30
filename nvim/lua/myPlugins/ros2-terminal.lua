local pickers = require "telescope.pickers"
local finders = require "telescope.finders"
local conf = require("telescope.config").values
local actions = require "telescope.actions"
local action_state = require "telescope.actions.state"
local Terminal = require("toggleterm.terminal").Terminal

local M = {
    terminals = {}, -- Stores all created terminals
    next_id = 1, -- Unique ID counter for terminals
}

-- Default terminal configuration
local default_config = {
    direction = "float",
    hidden = true,
    close_on_exit = false,
    auto_scroll = true,
    float_opts = {
        border = "curved",
        width = 150,
        height = 50,
    },
    on_open = function(term)
        vim.cmd "startinsert!"
        vim.api.nvim_buf_set_keymap(term.bufnr, "t", "<Esc>", "<C-\\><C-n>", { noremap = true })
    end,
}

local function toggle_terminal(term_id)
    local term_info = M.terminals[term_id]
    if not term_info then return end

    term_info.instance:toggle()
    term_info.is_open = not term_info.is_open
    term_info.is_running = term_info.is_open
end

function M.create_terminal(command)
    local term_id = M.next_id
    M.next_id = M.next_id + 1

    local term = Terminal:new(vim.tbl_extend("force", default_config, {
        cmd = command,
        on_exit = function()
            M.terminals[term_id] = nil -- Cleanup when terminal closes
        end,
    }))

    M.terminals[term_id] = {
        instance = term,
        command = command,
        is_running = false,
        is_open = false,
    }
    toggle_terminal(term_id)
    return term_id
end

function M.execute_command(term_id, new_command)
    local term_info = M.terminals[term_id]
    if not term_info then return end

    -- Stop current command if running
    if term_info.is_running then
        vim.api.nvim_chan_send(term_info.instance.job_id, "\003") -- Send Ctrl+C
        vim.schedule_wrap(function() term_info.instance:send(new_command, false) end)()
    else
        if not term_info.instance:is_open() then term_info.instance:open() end
        term_info.instance:send(new_command, false)
    end

    term_info.command = new_command
    term_info.is_running = true
end

-- Telescope Picker Implementation
function M.terminal_picker()
    local terminals = M.terminals
    if vim.tbl_isempty(terminals) then
        vim.notify("No ROS terminals available", vim.log.levels.INFO)
        return
    end

    local entries = {}
    for id, term in pairs(terminals) do
        table.insert(entries, {
            value = id,
            display = string.format("[%d] %s %s", id, term.is_open and "🟢" or "🔴", term.command),
            ordinal = term.command,
        })
    end

    pickers
        .new({}, {
            prompt_title = "ROS Terminals",
            finder = finders.new_table {
                results = entries,
                entry_maker = function(entry)
                    return {
                        value = entry.value,
                        display = entry.display,
                        ordinal = entry.ordinal,
                    }
                end,
            },
            sorter = conf.generic_sorter {},
            attach_mappings = function(prompt_bufnr)
                actions.select_default:replace(function()
                    actions.close(prompt_bufnr)
                    local selection = action_state.get_selected_entry()
                    toggle_terminal(selection.value)
                end)
                return true
            end,
        })
        :find()
end
return M

-- Example usage:
-- Create and manage ROS nodes
-- local term1 = M.create_terminal("ros2 run package node1")
-- M.toggle_terminal(term1)
--
-- Later...
-- M.execute_command(term1, "ros2 run package node2")
