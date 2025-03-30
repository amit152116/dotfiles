local M = {}
local actions = require "telescope.actions"
local action_state = require "telescope.actions.state"
local pickers = require "telescope.pickers"
local finders = require "telescope.finders"
local conf = require("telescope.config").values

-- Cache for last used directory
local last_used_dir = nil

-- Check if command exists
local function has_command(cmd) return vim.fn.executable(cmd) == 1 end

local function get_current_buffer_content()
    local buf = vim.api.nvim_get_current_buf()
    local lines = vim.api.nvim_buf_get_lines(buf, 0, -1, false)
    return table.concat(lines, "\n"), buf
end

local function ensure_directory_exists(path)
    local dir = path:match "(.*[/\\])"
    if dir and not vim.fn.isdirectory(dir) then vim.fn.mkdir(dir, "p") end
end

local function save_to_file(file_path, content, buf)
    local ok, err = pcall(function()
        ensure_directory_exists(file_path)
        vim.fn.writefile(vim.split(content, "\n"), file_path)
        vim.api.nvim_buf_set_name(buf, file_path)
        vim.cmd("silent! edit " .. vim.fn.fnameescape(file_path))
        last_used_dir = file_path:match "(.*[/\\])" or vim.fn.getcwd()
    end)

    if ok then
        vim.notify("File saved to: " .. file_path, vim.log.levels.INFO)
    else
        vim.notify("Failed to save file: " .. err, vim.log.levels.ERROR)
    end
end

local function get_fd_command()
    if has_command "fdfind" then return "fdfind" end
    if has_command "fd" then return "fd" end
    return nil
end

local function create_telescope_picker(file_name)
    local cwd = vim.fn.getcwd()
    local fd_cmd = get_fd_command()
    local finder

    -- Common function to create directory entries
    local function make_entry(entry)
        return {
            value = entry:gsub("/$", ""), -- Remove trailing slash
            display = entry,
            ordinal = entry,
        }
    end

    if fd_cmd then
        -- Get directories using fd
        local dirs = vim.fn.systemlist(fd_cmd .. " --type d --hidden --exclude .git .")
        -- Add navigation options
        table.insert(dirs, 1, "./") -- Current directory

        finder = finders.new_table {
            results = dirs,
            entry_maker = make_entry,
        }
    else
        -- Fallback to vim glob
        local dirs = vim.fn.globpath(cwd, "**/", false, true)
        local results = {}
        for _, dir in ipairs(dirs) do
            table.insert(results, vim.fn.fnamemodify(dir, ":."))
        end
        table.insert(results, "./")
        finder = finders.new_table {
            results = results,
            entry_maker = function(entry)
                return {
                    value = entry,
                    display = entry,
                    ordinal = entry,
                }
            end,
        }
    end

    local opts = {
        prompt_title = "Save File: " .. file_name,
        cwd = cwd,
        finder = finder,
        sorter = conf.generic_sorter(),
        attach_mappings = function(prompt_bufnr, map)
            map("i", "<CR>", function()
                local entry = action_state.get_selected_entry()
                actions.close(prompt_bufnr)
                if entry then
                    local content, buf = get_current_buffer_content()
                    local dir_path = vim.fn.fnamemodify(entry.value, ":p")
                    local file_path = dir_path .. file_name
                    save_to_file(file_path, content, buf)
                end
            end)
            map("i", "<C-o>", function() -- Open in file browser
                local entry = action_state.get_selected_entry()
                if entry then vim.cmd("edit " .. vim.fn.fnameescape(vim.fn.fnamemodify(entry.value, ":p"))) end
            end)
            return true
        end,
    }

    pickers.new(opts, opts):find()
end

function M.save_file()
    vim.ui.input({
        prompt = "Enter file name: ",
        default = last_used_dir and vim.fn.fnamemodify(last_used_dir, ":t") or "",
        completion = "file",
    }, function(input)
        if not input or input == "" then return end

        -- Add default extension if none provided
        if not input:match "%..+$" then
            local buf_ft = vim.api.nvim_buf_get_option(0, "filetype")
            if buf_ft ~= "" then input = input .. "." .. buf_ft end
        end

        create_telescope_picker(input)
    end)
end

-- Create command for easy access
vim.api.nvim_create_user_command("SaveFile", function() M.save_file() end, {})

return M
