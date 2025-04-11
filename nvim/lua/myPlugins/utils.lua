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

function M.create_form(config)
    -- Default configuration
    local defaults = {
        fields = { "Name", "Email", "Phone", "Address" },
        form_title = "FORM",
        title_width = 15,
        input_width = 20,
        border_style = "rounded",
    }
    config = vim.tbl_extend("force", defaults, config or {})

    -- Calculate dimensions
    local total_width = config.title_width + config.input_width + 3 -- +3 for borders
    local form_content = {}

    -- Helper function to create horizontal lines
    local function create_line(parts)
        return "┌" .. string.rep("─", config.title_width) .. "┬" .. string.rep("─", config.input_width) .. "┐"
    end

    -- Build form content
    table.insert(
        form_content,
        string.format(
            "┌%s%s%s┐",
            string.rep("─", config.title_width + 1),
            string.format(" %s ", config.form_title),
            string.rep("─", config.input_width - string.len(config.form_title) - 1)
        )
    )

    table.insert(
        form_content,
        string.format("├%s┬%s┤", string.rep("─", config.title_width), string.rep("─", config.input_width))
    )

    for _, field in ipairs(config.fields) do
        local padded_field = field .. string.rep(" ", config.title_width - #field)
        table.insert(
            form_content,
            string.format("│ %s │ %s │", padded_field, string.rep("░", config.input_width))
        )
    end

    table.insert(
        form_content,
        string.format("└%s┴%s┘", string.rep("─", config.title_width), string.rep("─", config.input_width))
    )
    table.insert(form_content, "")
    table.insert(form_content, " Press <ESC> to close")

    -- Create buffer and window
    local buf = vim.api.nvim_create_buf(false, true)
    vim.api.nvim_buf_set_lines(buf, 0, -1, false, form_content)
    vim.api.nvim_buf_set_option(buf, "modifiable", false)
    vim.api.nvim_buf_set_option(buf, "filetype", "form")

    local win = vim.api.nvim_open_win(buf, true, {
        relative = "editor",
        width = total_width,
        height = #form_content,
        col = (vim.o.columns - total_width) / 2,
        row = (vim.o.lines - #form_content) / 2 - 1,
        style = "minimal",
        border = config.border_style,
    })

    -- Set syntax highlighting
    vim.cmd [[
    syntax match FormBorder /┌\|─\|┬\|┐\|├\|┤\|┘\|└\|│\|┴/
    highlight FormBorder guifg=#87CEEB ctermfg=117
    
    syntax match FormField /░/
    highlight FormField guifg=#FFFFFF guibg=#4A4A4A ctermfg=255 ctermbg=238
    
    syntax match FormTitle /\v│\s\S+.*\s│/ contains=FormBorder
    highlight FormTitle guifg=#98FB98 ctermfg=120
    
    syntax match FormHeader /FORM/
    highlight FormHeader gui=bold guifg=#87CEEB cterm=bold ctermfg=117
  ]]

    -- Key mapping to close window
    vim.api.nvim_buf_set_keymap(buf, "n", "<Esc>", "<cmd>q!<CR>", { noremap = true, silent = true })
end

-- Create a command and keymap
vim.api.nvim_create_user_command("SourceConfig", M.source_nvim_settings, {})
return M
