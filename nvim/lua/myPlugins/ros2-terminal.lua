local Terminal = require("toggleterm.terminal").Terminal
local M = {}
-- Single ROS terminal instance
local ros_terminal = Terminal:new {
    cmd = "", -- Start with empty command
    direction = "float",
    hidden = true,
    float_opts = {
        border = "curved",
        width = 150,
        height = 50,
    },
    on_open = function(term) vim.cmd "startinsert!" end,
}

-- Toggle function with command execution
function M.ToggleRosTerminal(command)
    if ros_terminal:is_open() then
        if command then
            -- Send Ctrl+C to stop previous command
            vim.api.nvim_chan_send(ros_terminal.job_id, "\003")
            vim.wait(100, function() end) -- Brief delay
            -- Update and keep open
            ros_terminal:send(command, false) -- false = don't open new terminal
        else
            ros_terminal:toggle() -- Close if no new command
        end
    elseif command then
        -- Open with new command
        ros_terminal.cmd = command
        ros_terminal:toggle()
    else
        ros_terminal:toggle()
    end
end

return M
