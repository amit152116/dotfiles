return {
  {
    "AstroNvim/astrocore",
    opts = {
      autocmds = {
        dotfiles_auto = {
          {
            event = "BufWritePost",
            pattern = "*tmux.conf",
            command = "silent !tmux source-file % && tmux display-message 'Tmux Config Reloaded'",
          },
        },
        executable = {
          {
            event = "BufWritePost",
            pattern = { "*" },
            callback = function()
              -- 1. Check for Shebang first (fastest check)
              local first_line = vim.api.nvim_buf_get_lines(0, 0, 1, false)[1]
              if not first_line or not first_line:match "^#!" then return end

              -- 2. Check current file permissions
              local filename = vim.api.nvim_buf_get_name(0)
              local stat = vim.loop.fs_stat(filename)

              -- 3. Only chmod if the owner execute bit (64) is missing
              --    bit.band(stat.mode, 64) == 0 means "not executable by owner"
              if stat and bit.band(stat.mode, 64) == 0 then
                vim.loop.fs_chmod(filename, 493) -- 493 decimal is 755 octal
                vim.notify(
                  "Made script executable: "
                    .. vim.fn.fnamemodify(filename, ":t"),
                  vim.log.levels.INFO
                )
              end
            end,
          },
        },
      },
    },
  },
}
