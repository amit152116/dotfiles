-- Single switch for all AI completion backends.
return {
  -- One of: "copilot", "supermaven", "neocodeium", "windsurf", "minuet"
  -- Change this value, then `:Lazy sync` + restart nvim.
  backend = "copilot",

  -- Cloud LLM used when backend == "minuet". One of: "ollama_cloud", "openrouter", "nvidia"
  -- Each needs its own API key env var set in your shell:
  --   ollama_cloud -> OLLAMA_API_KEY  (ollama.com -> Settings -> API keys)
  --   openrouter   -> OPENROUTER_API_KEY (openrouter.ai/keys, use a :free model)
  --   nvidia       -> NVIDIA_API_KEY  (build.nvidia.com, free tier: 1000 credits, 40 req/min)
  llm = "openrouter",
}
