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

      -- Preview
      -- HACK(gitsigns): must go through the public async-wrapped API — calling
      -- the private preview.preview_hunk_inline() directly fails an assertion in
      -- gitsigns/async.lua (needs coroutine context), so it silently did nothing.
      -- That API's own close-on-{CursorMoved,InsertEnter,BufLeave} watcher is
      -- armed synchronously as part of opening (the float for removed lines, or
      -- the <C-y> scroll compensation for top-of-buffer hunks), and can catch the
      -- redraw that opening itself causes -> preview flashes open then closes
      -- instantly. Happens for any hunk with removed lines or straddling screen
      -- top/bottom, regardless of size. Fix: kill that watcher and re-arm an
      -- equivalent one one tick later, once the open-induced redraw has settled.
      -- Relies on the internal autocmd desc string "Clear gitsigns inline
      -- preview" — may break on gitsigns updates.
      map("<Leader>gp", function()
        local gitsigns = require "gitsigns"
        local bufnr = vim.api.nvim_get_current_buf()
        local host_win = vim.api.nvim_get_current_win()
        local ns = vim.api.nvim_create_namespace "gitsigns_preview_inline"

        gitsigns.preview_hunk_inline(function(err)
          if err then return end
          vim.schedule(function()
            if not vim.api.nvim_buf_is_valid(bufnr) then return end
            if
              #vim.api.nvim_buf_get_extmarks(bufnr, ns, 0, -1, { limit = 1 })
              == 0
            then
              return -- no hunk under cursor, nothing opened
            end

            for _, ev in ipairs { "CursorMoved", "InsertEnter", "BufLeave" } do
              for _, au in
                ipairs(vim.api.nvim_get_autocmds { event = ev, buffer = bufnr })
              do
                if au.desc == "Clear gitsigns inline preview" then
                  pcall(vim.api.nvim_del_autocmd, au.id)
                end
              end
            end

            local winid
            for _, win in ipairs(vim.api.nvim_tabpage_list_wins(0)) do
              local cfg = vim.api.nvim_win_get_config(win)
              if cfg.relative == "win" and cfg.win == host_win then
                winid = win
                break
              end
            end

            vim.defer_fn(function()
              if not vim.api.nvim_buf_is_valid(bufnr) then return end
              vim.api.nvim_create_autocmd(
                { "CursorMoved", "InsertEnter", "BufLeave" },
                {
                  buffer = bufnr,
                  once = true,
                  desc = "Clear gitsigns inline preview (deferred)",
                  callback = function()
                    if winid and vim.api.nvim_win_is_valid(winid) then
                      pcall(vim.api.nvim_win_close, winid, true)
                    end
                    vim.api.nvim_buf_clear_namespace(bufnr, ns, 0, -1)
                  end,
                }
              )
            end, 20)
          end)
        end)
      end, "Preview Hunk (inline)")

      -- Blame
      map(
        "<Leader>gy",
        function() require("gitsigns").blame_line() end,
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
