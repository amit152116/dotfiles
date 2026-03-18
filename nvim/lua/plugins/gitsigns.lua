local Snacks = require "snacks"

return {
  "lewis6991/gitsigns.nvim",
  opts = function(_, opts)
    local old_attach = opts.on_attach
    opts.attach_to_untracked = true -- show signs in untracked files (e.g. new files in git status)
    opts.on_attach = function(bufnr)
      if old_attach then old_attach(bufnr) end -- keep defaults

      local map = function(lhs, rhs, desc)
        vim.keymap.set("n", lhs, rhs, { buffer = bufnr, desc = desc })
      end

      -- Blame
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

      -- Browse / logs
      map("<Leader>go", function() Snacks.gitbrowse() end, "Git Browse (open)")
      map(
        "<Leader>gf",
        function() Snacks.picker.git_log_file { focus = "list" } end,
        "Git Logs (current file)"
      )
      map(
        "<Leader>gL",
        function() Snacks.picker.git_log_line { focus = "list" } end,
        "Git Logs (current line)"
      )
      map(
        "<Leader>gl",
        function() Snacks.picker.git_log { focus = "list" } end,
        "Git Logs"
      )

      -- Branches / status / stash
      map(
        "<Leader>gb",
        function() Snacks.picker.git_branches { focus = "list" } end,
        "Git branches"
      )
      map("<Leader>gt", function() Snacks.picker.git_status() end, "Git status")
      map("<Leader>gT", function() Snacks.picker.git_stash() end, "Git stash")

      -- Gitignore
      map("<Leader>gi", function()
        local bufdir =
          vim.fn.fnamemodify(vim.api.nvim_buf_get_name(bufnr), ":h")
        local root = vim.trim(
          vim.fn.system(
            "git -C "
              .. vim.fn.shellescape(bufdir)
              .. " rev-parse --show-toplevel"
          )
        )
        vim.cmd("edit " .. root .. "/.gitignore")
      end, "Open .gitignore")

      -- Backup
      map(
        "<Leader>gB",
        function() require("git_flow.backup").show_backups() end,
        "View Git Backup"
      )

      -- Worktree (only if this repo uses worktrees)
      local worktree = require "git_flow.worktree"
      if worktree.is_worktree_repo() then
        map(
          "<Leader>gw",
          function() worktree.switch_worktree() end,
          "Git Worktree: Switch/Create"
        )
      end
    end
    return opts
  end,
}
