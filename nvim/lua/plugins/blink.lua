return {
  {
    "saghen/blink.cmp",
    ---@module 'blink.cmp'
    ---@type blink.cmp.Config

    optional = true,
    dependencies = {
      "fang2hou/blink-copilot",
      "zbirenbaum/copilot.lua",
      -- "supermaven-inc/supermaven-nvim",
      -- "saghen/blink.compat",
    },
    opts = {
      sources = {
        default = { "copilot" },
        providers = {
          supermaven = {
            name = "supermaven",
            module = "blink-compat.source",
            score_offset = 200,
            async = true,
          },
          copilot = {
            name = "copilot",
            module = "blink-copilot",
            score_offset = 100,
            async = true,
            opts = {
              -- Local options override global ones
              max_completions = 5, -- Override global max_completions
              max_attempts = 2,
            },
          },
        },
      },
      keymap = {
        -- ["<Tab>"] = {
        --   "snippet_forward",
        --   function()
        --     if vim.g.ai_accept then return vim.g.ai_accept() end
        --   end,
        --   "fallback",
        -- },

        -- Or use Alt+Space instead
        -- ["<A-Space>"] = { "show", "show_documentation", "hide_documentation" },
        -- Or use Ctrl+N for a more vim-like approach
        -- ["<C-n>"] = { "show", "show_documentation", "hide_documentation" },
      },
    },
  },
}
