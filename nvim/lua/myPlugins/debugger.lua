local M = {}
function M.show_debug(value)
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
    vim.api.nvim_buf_set_lines(buf, 0, -1, false, vim.split(vim.inspect(value), "\n"))
end
return M
