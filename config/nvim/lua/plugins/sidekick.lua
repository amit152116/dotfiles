local helper = require "utils.helper"

return {
  "folke/sidekick.nvim",
  opts = {
    nes = {
      enabled = function(buf)
        if helper.is_secret_buf(buf) then return false end
        return vim.g.sidekick_nes ~= false and vim.b.sidekick_nes ~= false
      end,
    },
    cli = {
      mux = {
        backend = "tmux",
        enabled = true,
      },
      prompts = {
        commit = table.concat({
          "Run `git diff --staged --stat`. If it is empty, tell me nothing is staged and stop — do not write a commit message.",
          "Otherwise run `git diff --staged`. If it is too large to read directly, instead read it in per-file chunks (e.g. `git diff --staged -- <path>` for each file listed in the stat).",
          "Write a commit message for the staged change based on that.",
          "Follow Commitizen/Conventional Commits: `type(scope): subject`, types = feat|fix|refactor|docs|test|chore|perf|style.",
          "Subject: imperative mood, no period, max 50 chars. Body: wrap at 72 chars, explain why not what, bullet points for multiple changes.",
          "Write body in caveman-compact style: drop articles/filler/pleasantries, short fragments, keep all technical substance and exact terms.",
          "Add a `BREAKING CHANGE:` footer only if applicable. Output only the message in a gitcommit code block, no extra commentary.",
        }, "\n"),
      },
    },
    picker = "snacks",
  },
  keys = {
    {
      "<Leader>uN",
      function()
        local nes = require "sidekick.nes"
        nes.enable(not nes.enabled)
      end,
      desc = "Toggle Sidekick NES",
    },
    {
      "<c-.>",
      function() require("sidekick.cli").toggle() end,
      desc = "Sidekick Toggle",
      mode = { "n", "t", "i", "x" },
    },
    {
      "<Leader>az",
      function() require("sidekick.cli").toggle() end,
      desc = "Sidekick Toggle",
      mode = { "n", "t", "i", "x" },
    },
    {
      "<Leader>aa",
      function() require("sidekick.cli").toggle() end,
      desc = "Sidekick Toggle CLI",
    },
    {
      "<Leader>as",
      function()
        require("sidekick.cli").select { filter = { installed = true } }
      end,
      desc = "Select CLI",
    },
    {
      "<Leader>ad",
      function() require("sidekick.cli").close() end,
      desc = "Detach a CLI Session",
    },
    {
      "<Leader>at",
      function() require("sidekick.cli").send { msg = "{this}" } end,
      mode = { "x", "n" },
      desc = "Send This",
    },
    {
      "<Leader>af",
      function() require("sidekick.cli").send { msg = "{file}" } end,
      desc = "Send File",
    },
    {
      "<Leader>av",
      function() require("sidekick.cli").send { msg = "{selection}" } end,
      mode = { "x" },
      desc = "Send Visual Selection",
    },
    {
      "<Leader>ap",
      function() require("sidekick.cli").prompt() end,
      mode = { "n", "x" },
      desc = "Sidekick Select Prompt",
    },
    -- {
    --   "<Leader>ac",
    --   function()
    --     local cli = require "sidekick.cli"
    --     -- freshly spawned opencode needs time to boot before it reads pasted stdin, or the prompt is lost
    --     local attached = require("sidekick.cli.state").get { attached = true, name = "opencode" }
    --     vim.defer_fn(function()
    --       cli.send { name = "opencode", prompt = "commit", focus = true }
    --     end, #attached == 0 and 600 or 0)
    --   end,
    --   desc = "Git Commit Message",
    -- },
  },
}
