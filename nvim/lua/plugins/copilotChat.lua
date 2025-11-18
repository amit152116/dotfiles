---@type LazySpec
return {
  {
    "CopilotC-Nvim/CopilotChat.nvim",
    dependencies = {
      { "zbirenbaum/copilot.lua" },
      { "nvim-lua/plenary.nvim", branch = "master" }, -- for curl, log and async functions
    },
    build = "make tiktoken", -- Only on MacOS or Linux
    opts = {
      -- See Configuration section for options
    },

    -- See Commands section for default commands if you want to lazy load on them
    config = function()
      require("CopilotChat").setup {
        model = "claude-haiku-4.5",
        -- model = "gpt-4o",
        prompts = {
          AlgoDoc = {
            prompt = [[
            You are documenting this algorithm/data structure implementation for coding interview preparation.
            Generate a detailed structured comment block in the following format:

            1. Definition:
              - One-liner about what the algorithm/data structure does.

            2. Allowed Operations / Rules:
              - Describe what operations or rules are permitted (if applicable).

            3. Recurrence Relation / Formula:
              - Include DP recurrence, formula, or core idea (if applicable).
              - For data structures, describe the logic of core operations.

            4. Edge Cases / Pitfalls:
              - Common Tricky Cases (if applicable).

            5. Complexity:
              - Time and space complexity (worst/average/best if relevant).

            6. Example:
              - A small, concrete input/output transformation example.

            7. Use Cases:
              - Real-world applications and why this algorithm matters.

            8. Comparison:
              - How it relates to or differs from similar algorithms or data structures.

            9. Implementation Notes:
              - Specific implementation details (if applicable).

            Make it concise, structured, and uniform so it’s easy to revise for interviews.
            ]],
            system_prompt = "COPILOT_REVIEW",
            description = "Generate structured DSA algorithm comments (algo_doc style)",
          },
        },
      }
    end,
    specs = {
      {
        "AstroNvim/astrocore",
        ---@type AstroCoreOpts
        opts = {
          mappings = {
            n = {
              ["<Leader>a"] = { name = "AI" },
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
              ["<Leader>ap"] = {
                "<cmd>CopilotChatPrompts<CR>",
                desc = "Show prompts",
              },

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
              ["<Leader>ad"] = {
                "<cmd>CopilotChatDocs<CR>",
                desc = "Generate docs",
              },
              ["<Leader>at"] = {
                "<cmd>CopilotChatTests<CR>",
                desc = "Generate tests",
              },
              ["<Leader>ac"] = {
                "<cmd>CopilotChatCommit<CR>",
                desc = "Generate commit msg",
              },
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

              ["<Leader>af"] = { "<cmd>CopilotChatFix<CR>", desc = "Fix code" },

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
