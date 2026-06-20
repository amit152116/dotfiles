local active = require("ai_provider").backend
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
        cond = active == "supermaven", -- stays installed, just doesn't load
        opts = {
          disable_inline_completion = true, -- disables inline completion for use with cmp
          disable_keymaps = true, -- disables built in keymaps for more manual control
          log_level = "off", -- pure blink source, no need for its own status messages
          ignore_filetypes = { help = true, gitcommit = true, gitrebase = true },
        },
      },
      "huijiro/blink-cmp-supermaven",
      {
        "Exafunction/windsurf.nvim",
        cond = active == "windsurf", -- stays installed, just doesn't load
        event = "InsertEnter",
        dependencies = { "nvim-lua/plenary.nvim" },
        config = function()
          require("codeium").setup {
            bin_path = vim.fn.stdpath "data" .. "/codeium-server", -- shared with neocodeium's `bin`
            enable_cmp_source = true,
            enable_chat = false, -- unused here, skip the overhead
            detect_proxy = false, -- skip proxy probing on startup, speeds up InsertEnter
            virtual_text = { enabled = false }, -- ghost text off; codeium.blink feeds blink's popup instead
            workspace_root = {
              use_lsp = true,
              paths = { ".git", "package.xml", "CMakeLists.txt" }, -- ROS/catkin workspace markers
            },
          }
        end,
      },
      {
        "milanglacier/minuet-ai.nvim",
        cond = active == "minuet", -- stays installed, just doesn't load
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
          local llm = llm_configs[require("ai_provider").llm]
          llm.optional = { max_tokens = 56, top_p = 0.9 }

          require("minuet").setup {
            provider = "openai_compatible",
            n_completions = 1, -- single completion: keeps free-tier request count down
            context_window = 4000, -- 512 was too thin for chat-model completions on a multi-file ROS codebase
            request_timeout = 2.5,
            throttle = 1500, -- avoid burning free-tier rate limits
            debounce = 600,
            notify = "warn", -- only surface real errors, not every request
            -- skip auto-completion in huge files (generated msg/srv headers, build logs)
            -- to avoid burning free-tier requests on files the LLM can't meaningfully help with
            enable_predicates = {
              function() return vim.api.nvim_buf_line_count(0) < 3000 end,
            },
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
          auto_show = active == "neocodeium" and function(ctx)
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
          })[active]
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
            enabled = function() return active == "copilot" end,
            async = true,
            opts = {
              max_completions = 3,
              max_attempts = 4, -- recommended max_completions+1, accounts for empty Copilot responses
              debounce = 150, -- snappier than the 200ms default, ROS files trigger LSP often anyway
            },
          },
          supermaven = {
            name = "supermaven",
            module = "blink-cmp-supermaven",
            score_offset = ai_score_offset,
            async = true,
            enabled = function() return active == "supermaven" end,
          },
          codeium = {
            name = "Codeium",
            module = "codeium.blink",
            score_offset = ai_score_offset,
            async = true,
            enabled = function() return active == "windsurf" end,
          },
          minuet = {
            name = "minuet",
            module = "minuet.blink",
            async = true,
            timeout_ms = 3000,
            score_offset = ai_score_offset,
            enabled = function() return active == "minuet" end,
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
