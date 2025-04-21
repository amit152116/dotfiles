local neovim_config_dir = vim.fn.stdpath "config_dirs"
local ros2_terminal = require "myPlugins.ros2-terminal"
local ros2_nvim = require "myPlugins.ros2-nvim"
return {
    {
        -- AstroCore provides a central place to modify mappings, vim options, autocommands, and more!
        "AstroNvim/astrocore",
        ---@type AstroCoreOpts
        opts = {
            -- Mappings can be configured through AstroCore as well.
            -- NOTE: keycodes follow the casing in the vimdocs. For example, `<Leader>` must be capitalized
            mappings = {
                -- first key is the mode
                n = {
                    -- second key is the lefthand side of the map

                    -- mappings seen under group name "Buffer"
                    ["<C-Q>"] = false,
                    ["<C-R>"] = false,
                    ["<Leader>C"] = false,
                    ["<Leader>bd"] = {
                        function()
                            require("astroui.status.heirline").buffer_picker(
                                function(bufnr) require("astrocore.buffer").close(bufnr) end
                            )
                        end,
                        desc = "Close buffer from tabline",
                    },

                    -- Move cursor between visual lines
                    ["k"] = { "v:count == 0 ? 'gk' : 'k'", expr = true, silent = true },
                    ["j"] = { "v:count == 0 ? 'gj' : 'j'", expr = true, silent = true },

                    -- Keeps the cursor in the middle when search next or prev term
                    ["n"] = { "nzzzv", silent = true },
                    ["N"] = { "Nzzzv", silent = true },

                    -- TreeSitter Node navigation
                    ["<Tab>"] = {
                        function() require("myPlugins.custom_treesitter").go_to_next_node() end,
                        desc = "Go to next TreeSitter node",
                    },
                    ["<S-Tab>"] = {
                        function() require("myPlugins.custom_treesitter").go_to_prev_node() end,
                        desc = "Go to previous TreeSitter node",
                    },

                    -- Save new file
                    ["<Leader>sf"] = {
                        function() require("myPlugins.save_new_file").save_file() end,
                        desc = "Save File",
                    },

                    ["<Leader>fd"] = {
                        desc = "Find DevDocs ",
                        "<cmd>DevdocsOpenCurrent<CR>",
                    },
                    ["<Leader>o"] = {
                        "<C-w>w",
                        desc = "Switch window",
                    },
                    ["<Leader>r"] = {
                        desc = "ROS2 commands",
                    },
                    ["<Leader>ri"] = {
                        desc = "ROS Interfaces",
                    },
                    ["<Leader>rim"] = {
                        desc = "ROS Messages",
                        function() ros2_nvim.messages() end,
                    },
                    ["<Leader>ria"] = {
                        desc = "ROS Actions",
                        function() ros2_nvim.actions() end,
                    },
                    ["<Leader>ris"] = {
                        desc = "ROS Services",
                        function() ros2_nvim.services() end,
                    },
                    ["<Leader>rs"] = {
                        desc = "ROS Active Services",
                        function() ros2_nvim.active_services() end,
                    },
                    ["<Leader>rt"] = {
                        desc = "ROS Active Topics",
                        function() ros2_nvim.active_topics() end,
                    },
                    ["<Leader>rn"] = {
                        desc = "ROS Active Nodes",
                        function() ros2_nvim.active_nodes() end,
                    },
                    ["<Leader>rp"] = {
                        desc = "ROS Params",
                        function() ros2_nvim.param() end,
                    },
                    ["<Leader>re"] = {
                        desc = "ROS Execute Command",
                    },
                    ["<Leader>ren"] = {
                        desc = "Execute Node",
                        function() ros2_nvim.exec_nodes() end,
                    },
                    ["<Leader>rel"] = {
                        desc = "Execute Launch File",
                        function() ros2_nvim.exec_launch_file() end,
                    },
                    ["<Leader>rc"] = {
                        function() ros2_terminal.terminal_picker() end,
                        desc = "Toggle ROS Terminal",
                    },

                    -- CopilotChat mapppings
                    ["<Leader>a"] = {
                        desc = "Copilot Assistant",
                    },
                    -- Chat commands
                    ["<Leader>ac"] = {
                        desc = "CopilotChat - Open chat",
                        "<cmd>CopilotChat<CR>",
                    },
                    -- Save/Load history
                    ["<Leader>aw"] = {
                        desc = "CopilotChat - Save history",
                        "<cmd>CopilotChatSave<CR>",
                    },
                    ["<Leader>al"] = {
                        desc = "CopilotChat - Load history",
                        "<cmd>CopilotChatLoad<CR>",
                    },
                    -- Quick actions
                    ["<Leader>ap"] = {
                        desc = "CopilotChat - Show prompts",
                        "<cmd>CopilotChatPrompts<CR>",
                    },
                    ["<Leader>am"] = {
                        desc = "CopilotChat - Show models",
                        "<cmd>CopilotChatModels<CR>",
                    },
                    ["<Leader>aa"] = {
                        desc = "CopilotChat - Show agents",
                        "<cmd>CopilotChatAgents<CR>",
                    },
                    -- Predefined prompts
                    ["<Leader>ae"] = {
                        desc = "CopilotChat - Explain code",
                        "<cmd>CopilotChatExplain<CR>",
                    },
                    ["<Leader>av"] = {
                        desc = "CopilotChat - Review code",
                        "<cmd>CopilotChatReview<CR>",
                    },
                    ["<Leader>af"] = {
                        desc = "CopilotChat - Fix code",
                        "<cmd>CopilotChatFix<CR>",
                    },
                    ["<Leader>ao"] = {
                        desc = "CopilotChat - Optimize code",
                        "<cmd>CopilotChatOptimize<CR>",
                    },
                    ["<Leader>ad"] = {
                        desc = "CopilotChat - Add docs",
                        "<cmd>CopilotChatDocs<CR>",
                    },
                    ["<Leader>au"] = {
                        desc = "CopilotChat - Generate tests",
                        "<cmd>CopilotChatTests<CR>",
                    },
                    ["<Leader>ai"] = {
                        desc = "CopilotChat - Write commit message",
                        "<cmd>CopilotChatCommit<CR>",
                    },
                    ["<Leader>ax"] = {
                        desc = "CopilotChat - Reset Chat",
                        "<cmd>CopilotChatReset<CR>",
                    },
                    -- Rename the current file
                    -- ["<leader>rn"] = { ":saveas ", desc = "Rename current file" },

                    -- Save all files
                    ["<leader>W"] = { "<cmd>w<CR>", desc = "Save All Files" },

                    -- setting a mapping to false will disable it
                    -- ["<C-S>"] = false,
                },
                -- All Visual Mode
                x = {
                    -- Replace and keep yank
                    ["<C-Q>"] = false,
                    ["<C-R>"] = false,
                    ["<Leader>C"] = false,
                    ["<Leader>p"] = { '"_dP', desc = "Replace and keep yank" },
                    ["<Leader>ac"] = {
                        desc = "CopilotChat - Open chat",
                        "<cmd>CopilotChat<CR>",
                    },
                    ["<Leader>a"] = {
                        desc = "Copilot Assistant",
                    },
                    ["<Leader>ap"] = {
                        desc = "CopilotChat - Show prompts",
                        "<cmd>CopilotChatPrompts<CR>",
                    },
                    ["<Leader>ae"] = {
                        desc = "CopilotChat - Explain code",
                        "<cmd>CopilotChatExplain<CR>",
                    },
                    ["<Leader>av"] = {
                        desc = "CopilotChat - Review code",
                        "<cmd>CopilotChatReview<CR>",
                    },
                    ["<Leader>af"] = {
                        desc = "CopilotChat - Fix code",
                        "<cmd>CopilotChatFix<CR>",
                    },
                    ["<Leader>ao"] = {
                        desc = "CopilotChat - Optimize code",
                        "<cmd>CopilotChatOptimize<CR>",
                    },
                    ["<Leader>ad"] = {
                        desc = "CopilotChat - Add docs",
                        "<cmd>CopilotChatDocs<CR>",
                    },
                    ["<Leader>au"] = {
                        desc = "CopilotChat - Generate tests",
                        "<cmd>CopilotChatTests<CR>",
                    },
                },
                t = {

                    ["<Esc>"] = { "<C-\\><C-n>", desc = "Normal Mode Terminal" },
                },
                -- Visual mode mappings
                v = {
                    ["<C-Q>"] = false,
                    ["<C-R>"] = false,
                    ["<Leader>C"] = false,
                    -- Move selected lines up or down
                    ["J"] = { ":m '>+1<CR>gv=gv", desc = "Move Current Line Down", noremap = true, silent = true },
                    ["K"] = { ":m '<-2<CR>gv=gv", desc = "Move Current Line Up", noremap = true, silent = true },

                    -- Search for visually selected text literally
                    ["/"] = {
                        function()
                            -- Yank the visually selected text into the default register
                            vim.cmd 'normal! "vy'
                            -- Get the yanked text from the default register
                            local selected_text = vim.fn.getreg '"'
                            -- Escape special characters for literal searching
                            local escaped_text = vim.fn.escape(selected_text, "/\\")
                            -- Set the search register for literal searching
                            vim.fn.setreg("/", "\\V" .. escaped_text)
                            -- Trigger the search for the next occurrence
                            vim.cmd "normal! n"
                        end,
                        desc = "Search for visually selected text literally",
                    },
                },
            },
        },
    },

    {
        "AstroNvim/astrolsp",
        ---@type AstroLSPOpts
        opts = {
            -- mappings to be set up on attaching of a language server
            mappings = {
                n = {
                    -- a `cond` key can provided as the string of a server capability to be required to attach, or a function with `client` and `bufnr` parameters from the `on_attach` that returns a boolean
                    gD = {
                        function() vim.lsp.buf.declaration() end,
                        desc = "Declaration of current symbol",
                        cond = "textDocument/declaration",
                    },
                    ["<Leader>uY"] = {
                        function() require("astrolsp.toggles").buffer_semantic_tokens() end,
                        desc = "Toggle LSP semantic highlight (buffer)",
                        cond = function(client)
                            return client.supports_method "textDocument/semanticTokens/full"
                                and vim.lsp.semantic_tokens ~= nil
                        end,
                    },
                    grr = {
                        function() require("vim.lsp.buf").references() end,
                        desc = "vim.lsp.buf.references()",
                    },
                    gs = {
                        function() require("telescope.builtin").lsp_document_symbols() end,
                        desc = "Search Document Symbols",
                    },
                    grw = {
                        function() require("telescope.builtin").lsp_workspace_symbols() end,
                        desc = "Search WorkSpace Symbols",
                    },
                },
                x = {
                    -- ["<Tab>"] = {
                    --   function() require("plugins.custom_treesitter").go_to_next_node() end,
                    -- },
                    -- ["<S-Tab>"] = {
                    --   function() require("plugins.custom_treesitter").go_to_prev_node() end,
                    -- },
                },
            },
        },
    },
}
