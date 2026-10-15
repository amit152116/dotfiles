return {
  "Bekaboo/dropbar.nvim",
  keys = {
    -- upstream <Leader>; is taken; [; ]; were same-finger, so thumb+right-hand leader key
    {
      "<Leader>,",
      function() require("dropbar.api").pick() end,
      desc = "Pick breadcrumb (dropbar)",
    },
    {
      "<Leader>k",
      function() require("dropbar.api").goto_context_start() end,
      desc = "Go to start of current context",
    },
  },
}
