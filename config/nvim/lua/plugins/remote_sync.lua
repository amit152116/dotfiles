---@type LazySpec
return {
  -- local plugin, dir points straight at the nvim config tree (lua/remote_sync/)
  {
    dir = vim.fn.stdpath "config",
    name = "remote_sync",
    lazy = false,
    config = function()
      require("remote_sync").setup {
        -- Sync behavior
        sync_on_save = false, -- Set to true to auto-sync on save
        show_diff_on_receive = true,
        auto_create_dirs = true,
        log_sync_ops = true,
        log_level = "DEBUG",

        -- Project config location
        project_config_path = ".nvim/remote_sync.toml",

        -- Rsync options (includes .gitignore filtering for ROS2/C++ projects)
        rsync_opts = "-avzP --filter=':- .gitignore' --exclude='.git/'",

        -- SSH settings
        ssh_control_persist = "10m",
        connect_timeout = 5,
      }
    end,
  },
}
