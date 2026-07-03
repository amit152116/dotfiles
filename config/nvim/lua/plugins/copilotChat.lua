---@type LazySpec
return {
  {
    "CopilotC-Nvim/CopilotChat.nvim",
    dependencies = {
      { "zbirenbaum/copilot.lua" },
      { "nvim-lua/plenary.nvim", branch = "master" }, -- for curl, log and async functions
    },
    build = "make tiktoken", -- Only on MacOS or Linux

    -- See Commands section for default commands if you want to lazy load on them
    specs = {
      {
        "AstroNvim/astrocore",
        ---@type AstroCoreOpts
        opts = {
          mappings = {
            n = {
              ["<Leader>a"] = { name = "+AI" },
              ["<Leader>aa"] = {
                "<cmd>CopilotChatToggle<CR>",
                desc = "Toggle chat",
              },
              ["<Leader>aw"] = {
                "<cmd>CopilotChatSave<CR>",
                desc = "Write history",
              },
              ["<Leader>al"] = {
                "<cmd>CopilotChatLoad<CR>",
                desc = "Load history",
              },
              -- ["<Leader>ap"] = {
              --   "<cmd>CopilotChatPrompts<CR>",
              --   desc = "Show prompts",
              -- },

              ["<Leader>am"] = {
                "<cmd>CopilotChatModels<CR>",
                desc = "Show models",
              },

              ["<Leader>ae"] = {
                "<cmd>CopilotChatExplain<CR>",
                desc = "Explain code",
              },

              ["<Leader>af"] = { "<cmd>CopilotChatFix<CR>", desc = "Fix code" },

              ["<Leader>ao"] = {
                "<cmd>CopilotChatOptimize<CR>",
                desc = "Optimize code",
              },
              -- ["<Leader>ad"] = {
              --   "<cmd>CopilotChatDocs<CR>",
              --   desc = "Generate docs",
              -- },
              -- ["<Leader>at"] = {
              --   "<cmd>CopilotChatTests<CR>",
              --   desc = "Generate tests",
              -- },
              -- ["<Leader>ac"] = {
              --   "<cmd>CopilotChatCommit<CR>",
              --   desc = "Generate commit msg",
              -- },
              ["<Leader>ax"] = {
                "<cmd>CopilotChatReset<CR>",
                desc = "Clear chat",
              },
            },
            x = {
              ["<Leader>a"] = { name = "AI" },
              ["<Leader>aa"] = {
                "<cmd>CopilotChatToggle<CR>",
                desc = "Toggle chat",
              },
              ["<Leader>ap"] = {
                "<cmd>CopilotChatPrompts<CR>",
                desc = "Show prompts",
              },
              ["<Leader>ae"] = {
                "<cmd>CopilotChatExplain<CR>",
                desc = "Explain code",
              },

              -- ["<Leader>af"] = { "<cmd>CopilotChatFix<CR>", desc = "Fix code" },

              ["<Leader>ao"] = {
                "<cmd>CopilotChatOptimize<CR>",
                desc = "Optimize code",
              },
              ["<Leader>ad"] = {
                "<cmd>CopilotChatDocs<CR>",
                desc = "Generate docs",
              },
              ["<Leader>at"] = {
                "<cmd>CopilotChatTests<CR>",
                desc = "Generate tests",
              },
            },
          },
        },
      },
    },
  },
}
