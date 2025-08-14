---@type LazySpec
return {

  {
    "github/copilot.vim",
    enabled = false,
  },
  {
    "CopilotC-Nvim/CopilotChat.nvim",
    enabled = false,
    dependencies = {
      { "github/copilot.vim" }, -- or zbirenbaum/copilot.lua
      { "nvim-lua/plenary.nvim", branch = "master" }, -- for curl, log and async functions
    },
    build = "make tiktoken", -- Only on MacOS or Linux
    opts = {
      -- See Configuration section for options
    },

    -- See Commands section for default commands if you want to lazy load on them
    config = function()
      require("CopilotChat").setup {
        model = "claude-3.5-sonnet",
        -- model = "gpt-4o",
        prompts = {
          Resuable = {
            prompt = "Extract reusable functions or methods from this code to improve modularity and testability.",
            system_prompt = "COPILOT_REVIEW", -- System prompt to use (can be specified manually in prompt via /).
            description = "Reusable prompt description",
          },
          Refactor = {
            prompt = "Refactor this code to improve readability and maintainability.",
            system_prompt = "COPILOT_REVIEW", -- System prompt to use (can be specified manually in prompt via /).
            description = "Refactor prompt description",
          },
          DailySummary = {
            prompt = "Summarize the key changes and contributions made today across all code files. Highlight major features, bug fixes, refactoring, and configuration updates. Keep the summary concise and suitable for inclusion in a daily work log or team stand-up note.",
            system_prompt = "COPILOT_INSTRUCTIONS", -- System prompt to use (can be specified manually in prompt via /).
            description = "Daily summary prompt description",
          },
          ROSRefactor = {
            prompt = "Refactor the selected code with ROS 2 best practices in mind. Preserve functionality but improve readability, modularity, and performance.",
            system_prompt = "COPILOT_REVIEW", -- System prompt to use (can be specified manually in prompt via /).
          },

          ROSReview = {
            prompt = "Review this ROS 2 C++ code for memory management, topic/service usage, and adherence to ROS 2 conventions.",
            system_prompt = "COPILOT_REVIEW",
            description = "ROS 2 code review prompt description",
          },
        },
      }
    end,

    keys = {
      -- Normal mode
      {
        "<Leader>a",
        mode = "n",
        desc = "Copilot Assistant",
      },
      {
        "<Leader>ac",
        "<cmd>CopilotChat<CR>",
        mode = "n",
        desc = "CopilotChat - Open chat",
      },
      {
        "<Leader>aw",
        "<cmd>CopilotChatSave<CR>",
        mode = "n",
        desc = "CopilotChat - Save history",
      },
      {
        "<Leader>al",
        "<cmd>CopilotChatLoad<CR>",
        mode = "n",
        desc = "CopilotChat - Load history",
      },
      {
        "<Leader>ap",
        "<cmd>CopilotChatPrompts<CR>",
        mode = "n",
        desc = "CopilotChat - Show prompts",
      },
      {
        "<Leader>am",
        "<cmd>CopilotChatModels<CR>",
        mode = "n",
        desc = "CopilotChat - Show models",
      },
      {
        "<Leader>aa",
        "<cmd>CopilotChatAgents<CR>",
        mode = "n",
        desc = "CopilotChat - Show agents",
      },
      {
        "<Leader>ae",
        "<cmd>CopilotChatExplain<CR>",
        mode = "n",
        desc = "CopilotChat - Explain code",
      },
      {
        "<Leader>ar",
        "<cmd>CopilotChatReview<CR>",
        mode = "n",
        desc = "CopilotChat - Review code",
      },
      {
        "<Leader>af",
        "<cmd>CopilotChatFix<CR>",
        mode = "n",
        desc = "CopilotChat - Fix code",
      },
      {
        "<Leader>ao",
        "<cmd>CopilotChatOptimize<CR>",
        mode = "n",
        desc = "CopilotChat - Optimize code",
      },
      {
        "<Leader>ad",
        "<cmd>CopilotChatDocs<CR>",
        mode = "n",
        desc = "CopilotChat - Add docs",
      },
      {
        "<Leader>au",
        "<cmd>CopilotChatTests<CR>",
        mode = "n",
        desc = "CopilotChat - Generate tests",
      },
      {
        "<Leader>ai",
        "<cmd>CopilotChatCommit<CR>",
        mode = "n",
        desc = "CopilotChat - Write commit message",
      },
      {
        "<Leader>ax",
        "<cmd>CopilotChatReset<CR>",
        mode = "n",
        desc = "CopilotChat - Reset Chat",
      },

      -- Visual / select mode
      {
        "<Leader>a",
        mode = "x",
        desc = "Copilot Assistant",
      },
      {
        "<Leader>ac",
        "<cmd>CopilotChat<CR>",
        mode = "x",
        desc = "CopilotChat - Open chat",
      },
      {
        "<Leader>ap",
        "<cmd>CopilotChatPrompts<CR>",
        mode = "x",
        desc = "CopilotChat - Show prompts",
      },
      {
        "<Leader>ae",
        "<cmd>CopilotChatExplain<CR>",
        mode = "x",
        desc = "CopilotChat - Explain code",
      },
      {
        "<Leader>ar",
        "<cmd>CopilotChatReview<CR>",
        mode = "x",
        desc = "CopilotChat - Review code",
      },
      {
        "<Leader>af",
        "<cmd>CopilotChatFix<CR>",
        mode = "x",
        desc = "CopilotChat - Fix code",
      },
      {
        "<Leader>ao",
        "<cmd>CopilotChatOptimize<CR>",
        mode = "x",
        desc = "CopilotChat - Optimize code",
      },
      {
        "<Leader>ad",
        "<cmd>CopilotChatDocs<CR>",
        mode = "x",
        desc = "CopilotChat - Add docs",
      },
      {
        "<Leader>au",
        "<cmd>CopilotChatTests<CR>",
        mode = "x",
        desc = "CopilotChat - Generate tests",
      },
    },
  },
}
