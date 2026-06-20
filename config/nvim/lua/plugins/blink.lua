local active_ai = require "ai_provider"
-- shared rank for all AI sources: below lsp/snippets so known symbols still win ties
local ai_score_offset = 20
return {
  {
    "saghen/blink.cmp",
    ---@module 'blink.cmp'
    ---@type blink.cmp.Config

    optional = true,
    dependencies = {
      -- Copilot Source for Blink
      "zbirenbaum/copilot.lua",
      "fang2hou/blink-copilot",
      -- Ripgrep source for Blink
      "mikavilpas/blink-ripgrep.nvim",
      {
        "supermaven-inc/supermaven-nvim",
        cond = active_ai == "supermaven", -- stays installed, just doesn't load
        opts = {
          disable_inline_completion = true, -- disables inline completion for use with cmp
          disable_keymaps = true, -- disables built in keymaps for more manual control
        },
      },
      "huijiro/blink-cmp-supermaven",
      {
        "Exafunction/windsurf.nvim",
        cond = active_ai == "windsurf", -- stays installed, just doesn't load
        event = "InsertEnter",
        dependencies = { "nvim-lua/plenary.nvim" },
        config = function()
          require("codeium").setup {
            bin_path = vim.fn.stdpath "data" .. "/codeium-server", -- shared with neocodeium's `bin`
            enable_cmp_source = true,
            virtual_text = { enabled = false },
          }
        end,
      },
      {
        "milanglacier/minuet-ai.nvim",
        cond = active_ai == "minuet", -- stays installed, just doesn't load
        event = "InsertEnter",
        config = function()
          -- llm_provider.lua picks which cloud backend minuet talks to
          local llm_configs = {
            ollama_cloud = {
              api_key = "OLLAMA_API_KEY",
              name = "Ollama Cloud",
              end_point = "https://ollama.com/v1/chat/completions",
              model = "qwen3-coder:480b-cloud",
            },
            openrouter = {
              api_key = "OPENROUTER_API_KEY",
              name = "Openrouter",
              end_point = "https://openrouter.ai/api/v1/chat/completions",
              model = "qwen/qwen-2.5-coder-32b-instruct:free",
            },
            nvidia = {
              api_key = "NVIDIA_API_KEY",
              name = "Nvidia NIM",
              end_point = "https://integrate.api.nvidia.com/v1/chat/completions",
              model = "qwen/qwen3-coder-480b-a35b-instruct",
            },
          }
          local llm = llm_configs[require "llm_provider"]
          llm.optional = { max_tokens = 56, top_p = 0.9 }

          require("minuet").setup {
            provider = "openai_compatible",
            n_completions = 1,
            context_window = 512,
            request_timeout = 2.5,
            throttle = 1500, -- avoid burning free-tier rate limits
            debounce = 600,
            provider_options = { openai_compatible = llm },
            -- virtualtext block left out on purpose: blink source below replaces it.
          }
        end,
      },
    },
    ---@module 'blink.cmp'
    ---@type blink.cmp.Config
    opts = {
      completion = {
        menu = {
          -- only auto-pop in cmdline mode while neocodeium active, so its ghost
          -- text doesn't fight blink's popup -- https://github.com/monkoose/neocodeium#using-alongside-blinkcmp
          auto_show = active_ai == "neocodeium" and function(ctx)
            return ctx.mode ~= "default"
          end or nil,
        },
      },
      sources = {
        -- blink still requires() every provider module listed here just to
        -- check trigger characters, even disabled ones -- so only the active
        -- backend's source name goes in, not all four unconditionally.
        default = (function()
          local base = { "lsp", "path", "snippets", "buffer", "ripgrep" }
          local backend_source = ({
            copilot = "copilot",
            supermaven = "supermaven",
            windsurf = "codeium",
            minuet = "minuet",
          })[active_ai]
          if backend_source then table.insert(base, backend_source) end
          return base
        end)(),
        providers = {
          ripgrep = {
            module = "blink-ripgrep",
            name = "Ripgrep",
            score_offset = -10, -- weakest signal: repo-wide fuzzy text, fallback only
            async = true,
            ---@module "blink-ripgrep"
            ---@type blink-ripgrep.Options
            opts = {
              prefix_min_len = 7,
              backend = {
                use = "gitgrep-or-ripgrep",
              },
            },
          },
          copilot = {
            name = "copilot",
            module = "blink-copilot",
            score_offset = ai_score_offset,
            enabled = function()
              if active_ai == "copilot" then return true end
              return false
            end,
            async = true,
            opts = {
              -- Local options override global ones
              max_completions = 3, -- Override global max_completions
              max_attempts = 2,
            },
          },
          supermaven = {
            name = "supermaven",
            module = "blink-cmp-supermaven",
            score_offset = ai_score_offset,
            async = true,
            enabled = function()
              if active_ai == "supermaven" then return true end
              return false
            end,
          },
          codeium = {
            name = "Codeium",
            module = "codeium.blink",
            score_offset = ai_score_offset,
            async = true,
            enabled = function() return active_ai == "windsurf" end,
          },
          minuet = {
            name = "minuet",
            module = "minuet.blink",
            async = true,
            timeout_ms = 3000,
            score_offset = ai_score_offset,
            enabled = function() return active_ai == "minuet" end,
          },
        },
      },
      keymap = {
        -- snippet jump, then ai_accept (ghost text), then default fallback
        -- https://github.com/AstroNvim/astrocommunity/blob/main/lua/astrocommunity/recipes/ai/init.lua
        ["<Tab>"] = {
          "snippet_forward",
          "select_next", -- cycle blink popup items, if menu open
          function()
            if vim.g.ai_accept then return vim.g.ai_accept() end
          end,
          "fallback",
        },
        ["<S-Tab>"] = {
          "snippet_backward",
          "select_prev", -- cycle blink popup items backward, if menu open
          "fallback",
        },
      },
    },
  },
}
