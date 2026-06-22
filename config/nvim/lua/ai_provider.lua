-- Single switch for all AI completion backends.
return {
  -- One of: "copilot", "supermaven", "neocodeium", "windsurf", "minuet"
  -- Change this value, then `:Lazy sync` + restart nvim.
  backend = "copilot",

  -- Cloud LLM used when backend == "minuet". One of: "ollama_cloud", "openrouter", "nvidia"
  llm = "openrouter",

  -- Per-provider endpoint + free model catalog for minuet.
  -- To switch model: change the `model` key below to one of that provider's `models` keys.
  -- Verify exact tags still exist before relying on them -- catalogs change.
  llm_configs = {
    ollama_cloud = {
      api_key = "OLLAMA_API_KEY", -- ollama.com -> Settings -> API keys
      name = "Ollama Cloud",
      end_point = "https://ollama.com/v1/chat/completions",
      models = {
        light = "gpt-oss:20b-cloud", -- light, stretches free-tier quota
        coder = "qwen3-coder:480b-cloud", -- heavy, best coding quality, burns quota fast
      },
      model = "coder",
    },
    openrouter = {
      api_key = "OPENROUTER_API_KEY", -- openrouter.ai/keys -- only ":free"-suffixed models are free
      name = "Openrouter",
      end_point = "https://openrouter.ai/api/v1/chat/completions",
      models = {
        coder = "qwen/qwen3-coder:free", -- best free coding model, 256k-1M ctx
        reasoning = "deepseek/deepseek-v4-flash:free", -- strong reasoning, 1M ctx
        agentic = "nvidia/nemotron-3-ultra:free", -- 1M ctx agentic tasks
      },
      model = "coder",
    },
    nvidia = {
      api_key = "NVIDIA_API_KEY", -- build.nvidia.com, free tier: 1000 credits, 40 req/min
      name = "Nvidia NIM",
      end_point = "https://integrate.api.nvidia.com/v1/chat/completions",
      models = {
        coder = "qwen/qwen3-coder-480b-a35b-instruct", -- purpose-built coding, 256k ctx
        general = "meta/llama-4-maverick-instruct", -- general purpose, most-used on platform
        agentic = "mistralai/mistral-nemotron-instruct", -- tuned for function calling/agentic flows
      },
      model = "coder",
    },
  },
}
