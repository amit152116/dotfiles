-- Cloud LLM backend for minuet-ai.nvim (used only when ai_provider.lua == "minuet").
-- One of: "ollama_cloud", "openrouter", "nvidia"
-- Each needs its own API key env var set in your shell (see table below):
--   ollama_cloud -> OLLAMA_API_KEY  (ollama.com -> Settings -> API keys)
--   openrouter   -> OPENROUTER_API_KEY (openrouter.ai/keys, use a :free model)
--   nvidia       -> NVIDIA_API_KEY  (build.nvidia.com, free tier: 1000 credits, 40 req/min)
return "ollama_cloud"
