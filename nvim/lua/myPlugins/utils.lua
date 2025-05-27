local M = {}
function M.show_debug(...)
    local args = { ... }
    if #args == 0 then return end
    local buf = vim.api.nvim_create_buf(false, true)
    local win = vim.api.nvim_open_win(buf, true, {
        relative = "cursor",
        width = 60,
        height = 10,
        col = 2,
        row = 1,
        style = "minimal",
        border = "rounded",
    })
    vim.api.nvim_buf_set_lines(buf, 0, -1, false, vim.split(vim.inspect(args), "\n"))
end

function M.run_command(command, stdout, stderr)
    -- vim.notify("Running command: " .. command, vim.log.levels.INFO)
    vim.fn.jobstart(command, {
        stdout_buffered = true,
        on_stdout = stdout,
        on_stderr = stderr,
    })
end

function table.empty(self)
    for _, _ in pairs(self) do
        return false
    end
    return true
end

function M.trim(str) return str:match "^%s*(.-)%s*$" end

function M.source_nvim_settings()
    local config_file = vim.fn.stdpath "config" .. "/init.lua"
    -- vim.notify('Sourcing '..config_file, vim.log.levels.INFO)
    dofile(config_file)
    vim.notify("Neovim config reloaded!", vim.log.levels.INFO)
end

-- Create a command and keymap
vim.api.nvim_create_user_command("SourceConfig", M.source_nvim_settings, {})
return M
