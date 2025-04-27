local devdocs = require "nvim-devdocs"
local devdocs_list = require "nvim-devdocs.list"

local M = {}

function M.setup()
    -- Set up DevDocs
    devdocs.setup {
        -- Set the default language to use for DevDocs
        default_language = "lua",
        -- Set the default browser to use for DevDocs
        default_browser = "firefox",
        -- Set the default search engine to use for DevDocs
        default_search_engine = "duckduckgo",
        -- Set the default theme to use for DevDocs
        default_theme = "dark",
    }
end

local function get_docs_list()
    -- List all available DevDocs documentation
    local docs = devdocs_list.get_installed_alias()
    return docs
end

function M.open_docs_with_telescope()
    local docs = get_docs_list()
    require("telescope.pickers")
        .new({}, {
            prompt_title = "Select Documentation",
            finder = require("telescope.finders").new_table {
                results = docs,
            },
            sorter = require("telescope.config").values.generic_sorter {},
            attach_mappings = function(_, map)
                map("i", "<CR>", function(prompt_bufnr)
                    local selection = require("telescope.actions.state").get_selected_entry()
                    require("telescope.actions").close(prompt_bufnr)
                    vim.cmd("DevdocsOpen " .. selection.value)
                end)
                return true
            end,
            layout_strategy = "center",
            layout_config = {
                width = 0.3,
                height = 0.3,
            },
        })
        :find()
end

return M
