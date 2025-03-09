local M = {}

local http = require "plenary.http"

-- Function to query Ollama API
M.query_codellama = function(prompt, callback)
    local url = "http://localhost:11434/api/chat"
    local payload = {
        model = "codellama",
        messages = { { role = "user", content = prompt } },
    }

    http.request("POST", url, {
        body = vim.json.encode(payload),
        headers = { ["Content-Type"] = "application/json" },
        callback = function(response)
            if response.status == 200 then
                local data = vim.json.decode(response.body)
                if data and data.content then
                    callback(data.content)
                else
                    callback "Error: No response content"
                end
            else
                callback "Error: Failed to query CodeLlama"
            end
        end,
    })
end

-- Example mapping to test CodeLlama integration
M.setup = function()
    vim.api.nvim_set_keymap(
        "n",
        "<leader>cc",
        ":lua require('codellama').ask_codellama()<CR>",
        { noremap = true, silent = true }
    )
end

-- Example function to ask CodeLlama
M.ask_codellama = function()
    local prompt = vim.fn.input "CodeLlama Prompt: "
    M.query_codellama(prompt, function(response)
        vim.schedule(function() print("CodeLlama: " .. response) end)
    end)
end

return M
