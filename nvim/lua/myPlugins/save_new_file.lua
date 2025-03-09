local M = {}

local function get_current_buffer()
    -- Get the current buffer ID
    local buf = vim.api.nvim_get_current_buf()

    -- Get all lines of the current buffer
    local lines = vim.api.nvim_buf_get_lines(buf, 0, -1, false)

    -- Combine the lines into a single string if needed
    local content = table.concat(lines, "\n")

    -- Output the content of the buffer
    return content, buf
end

local function custom_ui_select(fileName)
    local cwd = vim.fn.getcwd() .. "/"
    local dirs = vim.fn.globpath(vim.fn.getcwd(), "**/", false, true)
    local options = {}

    for idx, dir in ipairs(dirs) do
        -- Strip the root part of the path, leaving only the relative part
        local relative_dir = dir:sub(#cwd + 1) -- Remove the root path part

        options[idx] = { dir = relative_dir, path = dir }
    end
    local root = { dir = "./", path = cwd }
    table.insert(options, 1, root)

    vim.ui.select(options, {
        prompt = "Select the Directory to Save File: {" .. fileName .. "}",
        format_item = function(item) return item.dir end,
    }, function(choice, _)
        if choice then
            local filePath = choice.path .. fileName
            local content, buf = get_current_buffer()
            vim.fn.writefile({ content }, filePath)
            vim.api.nvim_buf_set_name(buf, filePath)
            vim.cmd("edit " .. filePath)
            print("File created at: " .. filePath)
        else
            print "No selection made."
        end
    end)
end

function M.SaveFile()
    vim.ui.input({ prompt = "Enter File Name: " }, function(input)
        if input then
            custom_ui_select(input)
        else
            return
        end
    end)
end

return M
