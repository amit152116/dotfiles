---@type LazySpec
return {

  {
    "github/copilot.vim",
  },
  {
    "CopilotC-Nvim/CopilotChat.nvim",
    dependencies = {
      { "github/copilot.vim", enabled = false }, -- or zbirenbaum/copilot.lua
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
  },
}
