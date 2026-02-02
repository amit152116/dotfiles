local Snacks = require "snacks"

return {
  "lewis6991/gitsigns.nvim",
  opts = function(_, opts)
    local old_attach = opts.on_attach
    opts.on_attach = function(bufnr)
      if old_attach then old_attach(bufnr) end -- keep defaults

      -- ❌ remove the blame mapping
      vim.keymap.del("n", "<Leader>gl", { buffer = bufnr })
      vim.keymap.del("n", "<Leader>gL", { buffer = bufnr })
      local map = function(lhs, rhs, desc)
        vim.keymap.set("n", lhs, rhs, { buffer = bufnr, desc = desc })
      end

      map(
        "<Leader>gy",
        function() Snacks.git.blame_line() end,
        "View Git Blame"
      )
      map(
        "<Leader>gY",
        function() require("gitsigns").blame_line { full = true } end,
        "View full Git blame"
      )

      map("<Leader>go", function() Snacks.gitbrowse() end, "Git Browse (open)")
      map("<Leader>gf", function()
        local file = vim.fn.expand "%"
        if file == "" then
          vim.notify("No file in current buffer", vim.log.levels.WARN)
          return
        end

        vim.cmd(
          "silent !tmux-sessionizer -c lazygit -- -f "
            .. vim.fn.shellescape(vim.fn.expand "%:p")
        )
      end, "Git Logs (current file)")
    end
    return opts
  end,
}
