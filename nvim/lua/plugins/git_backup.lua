-- ~/.config/nvim/lua/plugins/git-backup.lua
local Snacks = require "snacks"
return {
  "AstroNvim/astrocore",
  ---@param opts AstroCoreOpts
  opts = function(_, opts)
    local M = {}
    local backup_dir = vim.fn.stdpath "data" .. "/git-backups"
    local manifest_file = backup_dir .. "/manifest.json"
    local max_backups = 20
    local timer = nil
    local backup_interval = 60 * 60 * 1000 -- 30 minutes in milliseconds

    -- Cache to store the Tree SHA of the last backup for each repo
    -- { ["repo_name"] = "tree_sha_string" }
    local last_backup_trees = {}

    -- Ensure backup directory exists
    if vim.fn.isdirectory(backup_dir) == 0 then
      vim.fn.mkdir(backup_dir, "p")
    end

    -- HELPER: Load Manifest (The Cache)
    local function load_manifest()
      local f = io.open(manifest_file, "r")
      if not f then return {} end
      local content = f:read "*a"
      f:close()

      -- Safety: decode json, return empty table if fails
      local ok, data = pcall(vim.json.decode, content)
      return ok and data or {}
    end

    -- HELPER: Save Manifest
    local function save_manifest(data)
      local f = io.open(manifest_file, "w")
      if f then
        f:write(vim.json.encode(data))
        f:close()
      end
    end

    -- Helper: Get git root
    local function get_git_root()
      local cmd = "git rev-parse --show-toplevel 2>/dev/null"
      local handle = io.popen(cmd)
      if not handle then return nil end
      local result = handle:read("*a"):gsub("%s+$", "")
      handle:close()
      return result ~= "" and result or nil
    end

    local function list_backups()
      local backups = {}
      local files = vim.fn.glob(backup_dir .. "/*.patch", false, true)

      for _, file in ipairs(files) do
        local name = vim.fn.fnamemodify(file, ":t")
        table.insert(backups, {
          name = name,
          file = file,
        })
      end

      -- Sort by newest first
      table.sort(backups, function(a, b) return a.name > b.name end)

      return backups
    end

    -- Apply a backup patch
    local function apply_backup(backup_file)
      local git_root = get_git_root()
      if not git_root then
        vim.notify("Not in a git repository", vim.log.levels.ERROR)
        return
      end

      -- Confirm before applying
      local confirm = vim.fn.confirm(
        "Apply backup patch? This will modify your working directory.",
        "&Yes\n&No",
        2
      )

      if confirm ~= 1 then
        vim.notify("Backup not applied", vim.log.levels.INFO)
        return
      end

      local cmd =
        string.format("cd %s && git apply %s 2>&1", git_root, backup_file)
      local handle = io.popen(cmd)
      if not handle then
        vim.notify("Failed to apply patch", vim.log.levels.ERROR)
        return
      end

      local result = handle:read "*a"
      local success = handle:close()

      if success then
        vim.notify("Backup applied successfully", vim.log.levels.INFO)
      else
        vim.notify("Failed to apply patch: " .. result, vim.log.levels.ERROR)
      end
    end

    -- CORE LOGIC: Create Backup
    function M.create_backup()
      local git_root = get_git_root()
      if not git_root then return end

      local repo_name = vim.fn.fnamemodify(git_root, ":t")
      -- Sanitize repo name (fix from previous turn)
      repo_name = repo_name:gsub("[^%w%-]", "_")

      -- 1. Create a "Ghost Stash"
      -- -u includes untracked files
      local cmd_create = string.format("cd %s && git stash create -u", git_root)
      local stash_sha = vim.fn.system(cmd_create):gsub("%s+", "")

      -- If output is empty, there are no changes at all relative to HEAD
      if stash_sha == "" then return end

      -- 2. DEDUPLICATION CHECK  -----------------------------------
      -- We extract the "Tree Object" SHA from the stash commit.
      -- The Tree SHA depends ONLY on file contents, not timestamps.
      local cmd_tree = string.format("git rev-parse %s^{tree}", stash_sha)
      local current_tree = vim.fn.system(cmd_tree):gsub("%s+", "")

      if #last_backup_trees == 0 then last_backup_trees = load_manifest() end

      if last_backup_trees[repo_name] == current_tree then
        -- Content is identical to the last backup we made this session.
        -- Skip to avoid spamming duplicates.
        return
      else
        last_backup_trees[repo_name] = current_tree
      end

      local timestamp = os.date "%Y%m%d_%H%M%S"
      local filename =
        string.format("%s/%s_%s.patch", backup_dir, repo_name, timestamp)

      -- 3. Export Ghost Stash to Patch File (Binary Safe)
      -- --binary ensures images/libs are backed up correctly
      local cmd_export = string.format(
        "cd %s && git stash show --include-untracked --binary --patch %s > %s",
        git_root,
        stash_sha,
        filename
      )

      vim.fn.system(cmd_export)

      -- 4. Rotate old backups
      local glob_pattern = string.format("%s/%s_*.patch", backup_dir, repo_name)
      local backups = vim.fn.glob(glob_pattern, false, true) -- returns list

      -- Sort reverse to keep newest
      table.sort(backups, function(a, b) return a > b end)

      if #backups > max_backups then
        for i = max_backups + 1, #backups do
          os.remove(backups[i])
        end
      end

      -- Optional: Silent success (uncomment to see notifications)
      vim.notify("Backup saved: " .. timestamp, vim.log.levels.INFO)
    end

    -- Start auto-backup timer
    function M.start_auto_backup()
      if timer then timer:stop() end

      timer = vim.loop.new_timer()
      timer:start(
        backup_interval,
        backup_interval,
        vim.schedule_wrap(function() M.create_backup() end)
      )
    end

    -- Stop auto-backup timer
    function M.stop_auto_backup()
      if timer then
        timer:stop()
        timer = nil
        vim.notify("Git auto-backup stopped", vim.log.levels.INFO)
      end
    end

    -- RESTORE LOGIC: Pick a backup
    function M.show_backups()
      local backups = list_backups()

      Snacks.picker.pick {
        title = "Git Backups",
        prompt = "Patch > ",
        ui_select = true,
        format = function(item)
          if not item or not item.name then return { { "" } } end

          -- 1. PARSE THE FILENAME
          -- Expected format: "repo_name_YYYYMMDD_HHMMSS.patch"
          -- We capture: (Name)_(Date)_(Time).patch
          local name, date_str, time_str =
            item.name:match "^(.*)_(%d%d%d%d%d%d%d%d)_(%d%d%d%d%d%d)%.patch$"

          -- Fallback if filename format is unexpected
          if not name then
            return { { "  ", "DiffAdd" }, { item.name, "Normal" } }
          end

          -- 2. FORMAT DATE & TIME (Make it readable)
          local year = date_str:sub(1, 4)
          local month = date_str:sub(5, 6)
          local day = date_str:sub(7, 8)
          local hour = time_str:sub(1, 2)
          local min = time_str:sub(3, 4)
          local sec = time_str:sub(5, 6)

          local fmt_date = string.format("%s-%s-%s", year, month, day)
          local fmt_time = string.format("%s:%s:%s", hour, min, sec)

          -- 3. RETURN STYLED COLUMNS
          return {
            -- Icon: Use DiagnosticOk (Green text) or String, which rarely have backgrounds
            { "   ", "DiagnosticOk" },

            -- Name: Use Function (Blue) or Normal
            { name .. "  ", "Function" },

            -- Separator: NonText is safe (dim)
            { "│ ", "NonText" },

            -- Date: Comment is usually safe (dim grey)
            { " " .. fmt_date, "Comment" },

            { "  " },

            -- Time: Use Constant or Special (Orange/Purple) instead of Number
            { " " .. fmt_time, "Constant" },
          }
        end,
        items = backups,
        confirm = function(picker, item)
          picker:close()
          if item then
            -- Create a scratch buffer to show the diff
            local buf = vim.api.nvim_create_buf(false, true)

            vim.api.nvim_set_option_value("bufhidden", "wipe", { buf = buf })
            vim.api.nvim_set_option_value("filetype", "diff", { buf = buf })

            -- Read the patch file content
            local lines = vim.fn.readfile(item.file)
            vim.api.nvim_buf_set_lines(buf, 0, -1, false, lines)

            vim.api.nvim_set_option_value("modifiable", false, { buf = buf })

            -- Set buffer name
            vim.api.nvim_buf_set_name(
              buf,
              "Git Backup: " .. vim.fn.fnamemodify(item.file, ":t")
            )

            -- Open in a new split
            vim.cmd "vsplit"
            vim.api.nvim_win_set_buf(0, buf)

            -- Add keymaps for the scratch buffer
            local opts = { buffer = buf, noremap = true, silent = true }
            vim.keymap.set("n", "q", "<cmd>close<cr>", opts)
            vim.keymap.set("n", "<leader>ga", function()
              vim.cmd "close"
              apply_backup(item.file)
            end, vim.tbl_extend(
              "force",
              opts,
              { desc = "Apply this backup" }
            ))

            -- Show help message
            vim.notify(
              "Press 'q' to close, '<leader>ga' to apply this backup",
              vim.log.levels.INFO
            )
          end
        end,
        actions = {
          applyPatch = function(picker, _)
            local sel = picker:selected()
            apply_backup(sel.file)
          end,
        },
        win = {
          input = {
            keys = {
              ["<c-y>"] = {
                "applyPatch",
                mode = { "n", "i" },
                desc = "Apply Patch",
              },
            },
          },
        },
      }
    end

    -- Register Commands
    vim.api.nvim_create_user_command("GitBackupNow", M.create_backup, {})
    vim.api.nvim_create_user_command("GitBackupShow", M.show_backups, {})

    -- Auto-start backup on VimEnter if in a git repo
    vim.api.nvim_create_autocmd("VimEnter", {
      callback = function()
        vim.defer_fn(function()
          if get_git_root() then
            vim.keymap.set("n", "<leader>gB", M.show_backups, {
              desc = "View Git Backup",
            })
            M.start_auto_backup()
          end
        end, 1000)
      end,
    })

    -- Correct: Run immediately, blocking exit for the few ms it takes to write
    vim.api.nvim_create_autocmd("VimLeavePre", {
      callback = function() save_manifest(last_backup_trees) end,
    })

    return opts
  end,
}
