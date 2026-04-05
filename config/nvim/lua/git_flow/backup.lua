---
--- git-flow.nvim - backup.lua
--- Stash-based patch backup system with deduplication and rotation.
--- Uses vim.fn.system (sync) and io.popen; no external dependencies.
---

local M = {}

local common = require "git_flow.common"
local config = require "git_flow.config"
local Snacks = require "snacks"

-- Module-level timer handle
local timer = nil

-- In-memory cache: { [repo_name] = "tree_sha" }
-- Persisted to / loaded from manifest.json between sessions
local last_backup_trees = {}

-- ---------------------------------------------------------------------------
-- Internal helpers
-- ---------------------------------------------------------------------------

--- Return the path of the manifest file (reads from config at call time so
--- that config changes are always respected).
---@return string
local function manifest_path()
  return config.options.backup.backup_dir .. "/manifest.json"
end

--- Load the on-disk manifest into last_backup_trees (once per session).
local function load_manifest()
  local f = io.open(manifest_path(), "r")
  if not f then return {} end
  local content = f:read "*a"
  f:close()
  local ok, data = pcall(vim.json.decode, content)
  return ok and data or {}
end

--- Resolve the git root and a sanitised repo name for the current working
--- context.  Returns (nil, nil) when not inside a git repository.
---@return string|nil git_root
---@return string|nil repo_name
local function get_context()
  local root = common.get_git_root()
  if not root then return nil, nil end
  return root, common.get_repo_name(root)
end

--- Collect all patch files for *repo_name*, sorted newest-first.
---@param repo_name string
---@return table[]  { name, file, value }
local function list_patches(repo_name)
  local pattern =
    string.format("%s/%s_*.patch", config.options.backup.backup_dir, repo_name)
  local files = vim.fn.glob(pattern, false, true)

  local backups = {}
  for _, file in ipairs(files) do
    local name = vim.fn.fnamemodify(file, ":t")
    table.insert(backups, { name = name, file = file, value = name })
  end

  table.sort(backups, function(a, b) return a.name > b.name end)
  return backups
end

--- Apply a patch file to the current repo after user confirmation.
---@param backup_file string Absolute path to the .patch file
local function apply_patch(backup_file)
  local git_root = common.get_git_root()
  if not git_root then
    vim.notify("Not in a git repository", vim.log.levels.ERROR)
    return
  end

  local choice = vim.fn.confirm(
    "Apply backup patch?\nThis will modify your working directory.",
    "&Yes\n&No",
    2
  )
  if choice ~= 1 then
    vim.notify("Backup not applied", vim.log.levels.INFO)
    return
  end

  local result, code = common.git_sync({ "apply", backup_file }, git_root)

  if code == 0 then
    vim.notify("Backup applied successfully", vim.log.levels.INFO)
  else
    vim.notify("Failed to apply patch: " .. result, vim.log.levels.ERROR)
  end
end

-- ---------------------------------------------------------------------------
-- Public API
-- ---------------------------------------------------------------------------

--- Persist the in-memory tree-SHA cache to disk.
function M.save_manifest()
  local f = io.open(manifest_path(), "w")
  if f then
    f:write(vim.json.encode(last_backup_trees))
    f:close()
  end
end

--- Create a patch backup of all uncommitted changes (staged + unstaged +
--- untracked).  Skips silently when there are no changes or when the content
--- is identical to the previous backup (deduplication via tree SHA).
function M.create_backup()
  local cfg = config.options.backup
  if not cfg.enabled then return end

  local git_root, repo_name = get_context()
  if not git_root then return end

  -- Create a ghost stash object without touching the stash stack
  local stash_sha, code = common.git_sync({ "stash", "create", "-u" }, git_root)
  if code ~= 0 or stash_sha == "" then return end -- nothing to back up

  -- Deduplication: compare tree SHA of this stash vs last saved one
  local current_tree =
    vim.trim(vim.fn.system { "git", "rev-parse", stash_sha .. "^{tree}" })
  if #last_backup_trees == 0 then last_backup_trees = load_manifest() end
  if last_backup_trees[repo_name] == current_tree then return end
  last_backup_trees[repo_name] = current_tree

  -- Write patch file
  local timestamp = os.date "%Y%m%d_%H%M%S"
  local filename =
    string.format("%s/%s_%s.patch", cfg.backup_dir, repo_name, timestamp)
  common.git_sync({
    "stash",
    "show",
    "--include-untracked",
    "--binary",
    "--patch",
    stash_sha,
  }, git_root)
  -- Use system directly so we can redirect to a file
  vim.fn.system(
    string.format(
      "git -C %s stash show --include-untracked --binary --patch %s > %s",
      vim.fn.shellescape(git_root),
      stash_sha,
      vim.fn.shellescape(filename)
    )
  )

  -- Rotate: keep only the newest max_backups patches
  local all = vim.fn.glob(
    string.format("%s/%s_*.patch", cfg.backup_dir, repo_name),
    false,
    true
  )
  table.sort(all, function(a, b) return a > b end)
  for i = cfg.max_backups + 1, #all do
    os.remove(all[i])
  end

  vim.notify(
    "Git backup saved: " .. timestamp,
    vim.log.levels.INFO,
    { title = "git-flow" }
  )
end

--- Start the periodic auto-backup timer.
function M.start_auto_backup()
  if timer then timer:stop() end
  local interval = config.options.backup.backup_interval
  timer = vim.loop.new_timer()
  timer:start(interval, interval, vim.schedule_wrap(M.create_backup))
end

--- Stop the periodic auto-backup timer.
function M.stop_auto_backup()
  if timer then
    timer:stop()
    timer = nil
    vim.notify(
      "Git auto-backup stopped",
      vim.log.levels.INFO,
      { title = "git-flow" }
    )
  end
end

--- Open a Snacks picker to browse and apply backup patches for the current repo.
function M.show_backups()
  local git_root, repo_name = get_context()
  if not git_root then
    vim.notify("Not in a git repository", vim.log.levels.WARN)
    return
  end

  local backups = list_patches(repo_name)

  Snacks.picker.pick {
    title = "  Git Backups – " .. repo_name,
    prompt = "Patch > ",
    ui_select = true,
    items = backups,

    format = function(item)
      if not item or not item.name then return { { "" } } end

      -- Filename: reponame_YYYYMMDD_HHMMSS.patch
      local _, date_str, time_str =
        item.name:match "^(.*)_(%d%d%d%d%d%d%d%d)_(%d%d%d%d%d%d)%.patch$"
      if not date_str then
        return { { "  ", "DiffAdd" }, { item.name, "Normal" } }
      end

      local fmt_date = string.format(
        "%s-%s-%s",
        date_str:sub(1, 4),
        date_str:sub(5, 6),
        date_str:sub(7, 8)
      )
      local fmt_time = string.format(
        "%s:%s:%s",
        time_str:sub(1, 2),
        time_str:sub(3, 4),
        time_str:sub(5, 6)
      )

      return {
        { "   ", "DiagnosticOk" },
        { repo_name .. "  ", "Function" },
        { "│ ", "NonText" },
        { " " .. fmt_date, "Comment" },
        { "  " },
        { " " .. fmt_time, "Constant" },
      }
    end,

    confirm = function(picker, item)
      picker:close()
      if not item then return end

      -- Show the diff in a scratch buffer
      local buf = vim.api.nvim_create_buf(false, true)
      vim.api.nvim_set_option_value("bufhidden", "wipe", { buf = buf })
      vim.api.nvim_set_option_value("filetype", "diff", { buf = buf })
      vim.api.nvim_buf_set_lines(buf, 0, -1, false, vim.fn.readfile(item.file))
      vim.api.nvim_set_option_value("modifiable", false, { buf = buf })
      vim.api.nvim_buf_set_name(
        buf,
        "Git Backup: " .. vim.fn.fnamemodify(item.file, ":t")
      )

      vim.cmd "vsplit"
      vim.api.nvim_win_set_buf(0, buf)

      local map_opts = { buffer = buf, noremap = true, silent = true }
      vim.keymap.set("n", "q", "<cmd>close<cr>", map_opts)
      vim.keymap.set("n", "<Leader>ga", function()
        vim.cmd "close"
        apply_patch(item.file)
      end, vim.tbl_extend(
        "force",
        map_opts,
        { desc = "Apply this backup" }
      ))

      vim.notify(
        "q = close  │  <Leader>ga = apply patch",
        vim.log.levels.INFO,
        { title = "git-flow backup" }
      )
    end,

    actions = {
      apply_patch = function(picker, _)
        local sel = picker:selected()
        if sel then apply_patch(sel.file) end
      end,
    },

    win = {
      input = {
        keys = {
          ["<C-y>"] = {
            "apply_patch",
            mode = { "n", "i" },
            desc = "Apply patch",
          },
        },
      },
    },
  }
end

return M
