---@meta

--- This file provides snacks.nvim pickers for managing git worktrees.
-- It supports switching between worktrees, creating new ones from remote branches,
-- and deleting worktrees with safety checks for uncommitted/unpushed changes.

local M = {}

local snacks = require "snacks"
local Job = require "plenary.job"
local Path = require "plenary.path"

-- 1. Define Hook Events
M.Events = {
  SWITCH_PRE = "switch_pre", -- Before switching (optional)
  SWITCH_POST = "switch_post", -- After switching (for LSP, buffer restore)
}

-- 2. Storage for hooks
M._hooks = {
  [M.Events.SWITCH_PRE] = {},
  [M.Events.SWITCH_POST] = {},
}

-- 3. Function to register a hook
---@param event string The event name (from M.Events)
---@param callback function(data) The function to run
function M.on(event, callback)
  if not M._hooks[event] then M._hooks[event] = {} end
  table.insert(M._hooks[event], callback)
end

-- 4. Internal helper to run hooks
local function emit(event, data)
  if M._hooks[event] then
    for _, callback in ipairs(M._hooks[event]) do
      -- Use pcall to prevent one bad hook from breaking the whole flow
      local status, err = pcall(callback, data)
      if not status then
        snacks.notify.error("Hook error [" .. event .. "]: " .. tostring(err))
      end
    end
  end
end

--- Helper wrapper for plenary.job
---@param args table List of git arguments
---@return table|nil output List of stdout lines
---@return number return_val Exit code
local function git_job(args)
  local job = Job:new {
    command = "git",
    args = args,
    cwd = vim.loop.cwd(),
    -- Determine what to do with stderr.
    -- For now, we capture it to potentially debug, but usually we just check code.
    on_stderr = function(_, _) end,
  }

  return job:sync()
end

--- Get the git root directory
---@return string|nil
local function get_git_root()
  local output, code = git_job { "rev-parse", "--show-toplevel" }
  if code ~= 0 or not output or #output == 0 then return nil end
  return vim.trim(output[1])
end

--- Get the git root directory
---@return string|nil
local function get_bare_repo_path()
  local output, code = git_job { "rev-parse", "--git-common-dir" }
  if code ~= 0 or not output or #output == 0 then return nil end
  return vim.trim(output[1])
end

--- Get list of all worktrees
---@return table[]
local function get_worktrees()
  local output, code = git_job { "worktree", "list", "--porcelain" }
  if code ~= 0 then return {} end

  local worktrees = {}
  local current_worktree = {}

  for _, line in ipairs(output) do
    local worktree_path = line:match "^worktree%s+(.+)$"

    if worktree_path then
      -- If we already have a worktree in progress, save it before starting new one
      if current_worktree.path then
        table.insert(worktrees, current_worktree)
      end
      current_worktree = { path = worktree_path }
    elseif line:match "^HEAD" then
      current_worktree.head = line:match "^HEAD%s+(.+)$"
    elseif line:match "^branch" then
      current_worktree.branch = line:match "^branch%s+refs/heads/(.+)$"
    end
  end

  -- Always add the final worktree
  if current_worktree.path then table.insert(worktrees, current_worktree) end

  return worktrees
end

--- Get list of all branches (local and remote)
---@return table[]
local function get_branches()
  -- 1. Get Branches
  local branch_args =
    { "branch", "-a", "--format=%(refname:short)|%(upstream:short)|%(HEAD)" }
  local branch_output, branch_code = git_job(branch_args)

  if branch_code ~= 0 then return {} end

  -- 2. Get Worktrees (to map existing ones)
  local worktrees = get_worktrees()
  local worktree_branches = {}
  for _, wt in ipairs(worktrees) do
    if wt.branch then worktree_branches[wt.branch] = wt.path end
  end

  -- 3. Get Remotes (to identify remote branches)
  local remote_output, remote_code = git_job { "remote" }
  local remotes = {}
  if remote_code == 0 then
    for _, remote in ipairs(remote_output) do
      remotes[remote] = true
    end
  end

  local branches = {}

  for _, line in ipairs(branch_output) do
    local parts = vim.split(line, "|")
    local ref = vim.trim(parts[1] or "")
    local upstream = vim.trim(parts[2] or "")
    local is_current = vim.trim(parts[3] or "") == "*"

    if not is_current and ref ~= "" and not ref:match "HEAD" then
      local is_remote = false
      local branch_name = ref
      local display_name = ref

      -- Check if the ref starts with any known remote name
      local first_part = ref:match "^([^/]+)/"
      if first_part and remotes[first_part] then
        is_remote = true
        -- Extract branch name by removing the remote prefix (e.g., origin/dev -> dev)
        branch_name = ref:sub(#first_part + 2)
      end

      -- Check if this branch already has a worktree
      local worktree_path = worktree_branches[branch_name]

      local entry = {
        ref = ref,
        branch_name = branch_name,
        display_name = display_name,
        is_remote = is_remote,
        is_current = is_current,
        upstream = upstream,
        has_worktree = worktree_path ~= nil,
        worktree_path = worktree_path,
      }

      -- Insert at position 1 if current, otherwise append to end
      table.insert(branches, is_current and 1 or (#branches + 1), entry)
    end
  end

  return branches
end

--- Check git status of a specific worktree path
---@param path string
---@return boolean has_changes
---@return string status_message
local function check_worktree_status(path)
  -- We run git status in the specific directory
  local job = Job:new {
    command = "git",
    args = { "status", "--porcelain" },
    cwd = path, -- Run command inside the worktree
  }

  local output, code = job:sync()

  if code ~= 0 then return false, "Unable to check status" end

  if output and #output > 0 then
    return true, #output .. " uncommitted file(s)"
  end

  return false, "Clean"
end

--- Create a new worktree from a branch
---@param branch_data table
---@return boolean success
---@return string message_or_path
local function create_worktree(branch_data)
  local git_root = get_bare_repo_path()
  if not git_root then return false, "Not in a git repository" end

  local branch_name = branch_data.branch_name
  local is_remote = branch_data.is_remote
  local full_ref = branch_data.ref

  -- Create directory name (migration/px4 -> migration-px4)
  local dir_name = branch_name:gsub("/", "-")

  -- Logic for Bare Repo: Place worktree as a sibling to the bare repo
  local worktree_path = git_root .. "/" .. dir_name

  if vim.fn.isdirectory(worktree_path) == 1 then
    return false, "Worktree directory already exists: " .. worktree_path
  end

  -- Build arguments for Plenary Job
  local args = { "worktree", "add" }

  if is_remote then
    -- git worktree add -b <branch> <path> <upstream>
    table.insert(args, "-b")
    table.insert(args, branch_name)
    table.insert(args, worktree_path)
    table.insert(args, full_ref)
  else
    -- git worktree add <path> <branch>
    table.insert(args, worktree_path)
    table.insert(args, branch_name)
  end

  local output, code = git_job(args)

  if code ~= 0 then
    -- Concatenate output lines to form the error message
    local error_msg = table.concat(output or {}, "\n")
    return false, "Failed: " .. error_msg
  end

  return true, worktree_path
end

--- Switch to a worktree directory
---@param worktree_path string
---@param context table|nil Generic data table passed to hooks (e.g. { file = "rel/path" })
local function switch_to_worktree(worktree_path, context)
  local ctx = context or {}

  -- Validate path exists before switching
  if vim.fn.isdirectory(worktree_path) == 0 then
    snacks.notify.error("Worktree path does not exist: " .. worktree_path)
    return
  end

  -- Run PRE hooks (optional, e.g. for saving state)
  emit(M.Events.SWITCH_PRE, { path = worktree_path, context = ctx })

  -- Change vim's current directory
  vim.cmd("cd " .. vim.fn.fnameescape(worktree_path))

  -- Clear jumps to prevent accidental navigation back to the old worktree
  vim.cmd "clearjumps"

  -- Run POST hooks (This is where LSP and Buffer restoration happens now)
  emit(M.Events.SWITCH_POST, { path = worktree_path, context = ctx })

  -- Notification (can also be a hook, but kept here for core UX)
  local folder_name = vim.fn.fnamemodify(worktree_path, ":t")
  snacks.notify.notify("Switched to worktree [" .. folder_name .. "]", {
    title = "Git Worktree",
    level = "info",
  })
end

--- Open a picker to cherry-pick commits from a specific branch
---@param branch_name string
local function cherry_pick_from_branch(branch_name)
  -- 1. Get git log for the target branch
  -- Format: hash|date|author|subject
  local args = {
    "log",
    branch_name,
    "--pretty=format:%h|%cd|%an|%s",
    "--date=short",
    "-n",
    "50",
  }
  local output, code = git_job(args)

  if code ~= 0 then
    snacks.notify.error("Failed to get log for branch: " .. branch_name)
    return
  end

  local items = {}
  for _, line in ipairs(output) do
    local parts = vim.split(line, "|")
    if #parts >= 4 then
      table.insert(items, {
        text = parts[1] .. " " .. parts[4], -- hash + subject
        hash = parts[1],
        date = parts[2],
        author = parts[3],
        subject = parts[4],
      })
    end
  end

  -- 2. Open Picker
  snacks.picker.pick {
    title = "Cherry Pick from " .. branch_name,
    finder = function() return items end,
    format = function(item)
      return {
        { item.hash, "Number" },
        { " " },
        { item.date, "Comment" },
        { " " },
        { item.author, "String" },
        { "  " },
        { item.subject, "Normal" },
      }
    end,
    preview = function(ctx)
      local item = ctx.item
      local cmd = string.format("git show %s --stat -p", item.hash)
      local show_output = vim.fn.system(cmd) -- Using system here for simple text output preview
      ctx.preview:set_lines(vim.split(show_output, "\n"))
      ctx.preview:highlight { ft = "git" }
    end,
    actions = {
      confirm = function(self, item)
        self:close()
        vim.schedule(function()
          local confirm = vim.fn.confirm(
            "Cherry pick commit " .. item.hash .. "?\n" .. item.subject,
            "&Yes\n&No",
            1
          )
          if confirm == 1 then
            local _, cp_code = git_job { "cherry-pick", item.hash }
            if cp_code == 0 then
              snacks.notify.info("Successfully cherry-picked " .. item.hash)
            else
              snacks.notify.error "Cherry-pick failed (conflict?)"
            end
          end
        end)
      end,
    },
  }
end

--- Delete a worktree
---@param worktree_path string
---@param force boolean
---@return boolean success
---@return string message
local function delete_worktree(worktree_path, force)
  local args = { "worktree", "remove" }

  if force then table.insert(args, "--force") end

  table.insert(args, worktree_path)

  local output, code = git_job(args)

  if code ~= 0 then
    -- Concatenate output lines to capture the specific git error (e.g., "contains modified or untracked files")
    local error_msg = table.concat(output or {}, "\n")
    return false, "Failed to delete worktree: " .. error_msg
  end

  return true, "Worktree deleted successfully"
end

--- Helper to ensure worktree exists before performing an action
---@param branch table The branch object
---@param on_ready function(path) Callback to run with the valid worktree path
local function ensure_worktree(branch, on_ready)
  if branch.has_worktree then
    on_ready(branch.worktree_path)
  else
    -- Prompt for creation
    if
      vim.fn.confirm(
        "Create new worktree for [" .. branch.branch_name .. "]?",
        "&Yes\n&No",
        2
      ) ~= 1
    then
      snacks.notify.info(
        "Worktree creation cancelled",
        { title = "Git Worktree" }
      )
      return
    end

    -- Attempt creation
    local success, result_path = create_worktree(branch)
    if success then
      snacks.notify.info(
        "Worktree created: " .. result_path,
        { title = "Git Worktree" }
      )
      on_ready(result_path)
    else
      snacks.notify.error(result_path, { title = "Git Worktree" })
    end
  end
end

--- Picker for switching/creating git worktrees
function M.switch_worktree()
  local branches = get_branches()

  if #branches == 0 then
    snacks.notify.notify("No git branches found", {
      title = "Git Worktree",
      level = "warn",
    })
    return
  end

  -- Transform branches into picker items
  local items = {}
  for _, branch in ipairs(branches) do
    local idx = branch.is_current and 1000 or branch.is_remote and 1 or 100
    table.insert(items, {
      text = branch.display_name,
      score = idx,
      data = branch,
    })
  end

  snacks.picker.pick {
    title = "  Git Worktrees - Switch/Create",
    items = items,
    focus = "list",
    format = function(item, _)
      if not item or not item.data then return { { "" } } end

      local branch = item.data
      local icon = "  "
      local hl = "Normal"
      local prefix = ""

      if branch.is_current then
        icon = "  "
        hl = "DiagnosticOk"
        prefix = "󱐋 [current] "
      elseif branch.is_remote then
        icon = "  "
        hl = "DiagnosticError"
        prefix = "[remote] "
      elseif branch.has_worktree then
        icon = "  "
        hl = "DiagnosticInfo"
        prefix = "[worktree] "
      else
        icon = "  "
        hl = "Normal"
        prefix = "[local] "
      end

      return {
        { icon, hl },
        { prefix, hl },
        { branch.display_name, hl },
      }
    end,
    sort = { fields = { "score:desc" } },
    preview = function(ctx)
      local item = ctx.item
      if not item or not item.data then return true end

      local branch = item.data
      local lines = {}

      -- Header Section
      table.insert(lines, "# Branch Details")
      table.insert(lines, "")
      table.insert(lines, "  󱓞 **Name**:       " .. branch.branch_name)
      table.insert(lines, "  󱔗 **Reference**: " .. branch.ref)

      -- Type Badge Logic
      local b_type = branch.is_remote and "󰓅 Remote" or "󰙅 Local"
      if branch.is_current then b_type = "󱐋 Current" end
      table.insert(lines, "  󰓹 **Type**:       " .. b_type)

      if branch.upstream and branch.upstream ~= "" then
        table.insert(lines, "  󰅟 **Upstream**:  " .. branch.upstream)
      end

      table.insert(lines, "")
      table.insert(lines, "---")
      table.insert(lines, "")

      -- Worktree Status Section
      if branch.has_worktree then
        table.insert(lines, "## 󰙅 Worktree Active")
        table.insert(lines, "  󰝰 **Path**: " .. branch.worktree_path)
        table.insert(lines, "")

        -- Status Check
        local has_changes, status_msg =
          check_worktree_status(branch.worktree_path)
        if has_changes then
          table.insert(lines, "  > [!CAUTION]")
          table.insert(lines, "  > 󱈸 **Uncommitted Changes Detected**")
          table.insert(lines, "  > " .. status_msg)
        else
          table.insert(lines, "  > [!NOTE]")
          table.insert(lines, "  > 󰄬 **Working Directory Clean**")
        end
      else
        table.insert(lines, "## 󰚰 No Active Worktree")
        local action_msg = branch.is_remote
            and "Select to create a new folder from this remote."
          or "Select to check this out into a new worktree folder."
        table.insert(lines, "  " .. action_msg)
      end

      -- Render to buffer
      ctx.preview:set_lines(lines)
      ctx.preview:highlight { ft = "markdown" }

      -- Set the title with a nice icon
      local title_icon = branch.has_worktree and "󰙅 " or "󰓅 "
      ctx.preview:set_title(title_icon .. branch.display_name)

      return true
    end,
    actions = {
      -- Action: Switch inside Neovim
      switch_nvim = function(self, item)
        local branch = item.data
        self:close()

        vim.schedule(function()
          ensure_worktree(
            branch,
            function(path) switch_to_worktree(path, branch) end
          )
        end)
      end,

      -- Action: Switch Tmux Session
      switch_tmux = function(self, item)
        local branch = item.data
        self:close()

        vim.schedule(function()
          ensure_worktree(branch, function(path)
            -- Execute external tmux script safely
            local output = vim.fn.system { "tmux-sessionizer", path }

            if vim.v.shell_error ~= 0 then
              snacks.notify.error("Failed to switch tmux session:\n" .. output)
            end
          end)
        end)
      end,

      -- Action: Cherry Pick (Unchanged)
      cherry_pick = function(self, item)
        local branch = item.data
        self:close()
        vim.schedule(function() cherry_pick_from_branch(branch.branch_name) end)
      end,
    },
    win = {
      input = {
        keys = {
          ["<CR>"] = { "switch_tmux", mode = { "n", "i" } },
          ["<c-y>"] = { "switch_nvim", mode = { "n", "i" } },
          ["<c-p>"] = { "cherry_pick", mode = { "n", "i" } }, -- Bind Ctrl+P to Cherry Pick
        },
      },
      list = {
        keys = {
          ["<CR>"] = { "switch_tmux", mode = { "n", "i" } },
          ["<c-y>"] = { "switch_nvim", mode = { "n", "i" } },
          ["<c-p>"] = { "cherry_pick", mode = { "n", "i" } }, -- Bind Ctrl+P to Cherry Pick
        },
      },
    },
  }
end

--- Picker for deleting git worktrees
function M.delete_worktree()
  local worktrees = get_worktrees()

  if #worktrees == 0 then
    snacks.notify.notify("No worktrees found", {
      title = "Git Worktree",
      level = "warn",
    })
    return
  end

  -- Filter out the main worktree (usually the first one)
  local deletable_worktrees = {}
  for i, wt in ipairs(worktrees) do
    if i > 1 then -- Skip the main worktree
      table.insert(deletable_worktrees, wt)
    end
  end

  if #deletable_worktrees == 0 then
    snacks.notify.notify(
      "No additional worktrees to delete (main worktree cannot be deleted)",
      {
        title = "Git Worktree",
        level = "warn",
      }
    )
    return
  end

  -- Transform worktrees into picker items
  local items = {}
  for _, wt in ipairs(deletable_worktrees) do
    table.insert(items, {
      text = wt.branch or "detached",
      data = wt,
    })
  end

  snacks.picker.pick {
    title = "  Git Worktrees - Delete",
    finder = function() return items end,
    format = function(item, _)
      if not item or not item.data then return { { "" } } end

      local wt = item.data
      local branch_display = wt.branch or "detached"
      local has_changes, _ = check_worktree_status(wt.path)

      local icon = has_changes and "  " or "  "
      local hl = has_changes and "DiagnosticWarn" or "Normal"

      return {
        { icon, hl },
        { " " .. branch_display, hl },
        { "  (" .. vim.fn.fnamemodify(wt.path, ":t") .. ")", "Comment" },
      }
    end,
    preview = function(ctx)
      local item = ctx.item
      if not item or not item.data then return true end

      local wt = item.data
      local lines = {}
      table.insert(lines, "Worktree Path:")
      table.insert(lines, "  " .. wt.path)
      table.insert(lines, "")
      table.insert(lines, "Branch: " .. (wt.branch or "detached"))

      if wt.head then table.insert(lines, "HEAD: " .. wt.head:sub(1, 12)) end

      table.insert(lines, "")
      table.insert(lines, "Status Check:")
      table.insert(lines, "")

      local has_changes, status_msg = check_worktree_status(wt.path)

      if has_changes then
        table.insert(lines, "⚠ WARNING: This worktree has " .. status_msg)
        table.insert(lines, "")
        table.insert(lines, "You can:")
        table.insert(lines, "  - Press <CR> to delete anyway (will fail)")
        table.insert(
          lines,
          "  - Press <C-f> to FORCE delete (⚠ LOSE CHANGES)"
        )
        table.insert(lines, "  - Press <Esc> to cancel")
      else
        table.insert(lines, "✓ Clean worktree")
        table.insert(lines, "")
        table.insert(lines, "Safe to delete:")
        table.insert(lines, "  - No uncommitted changes")
        table.insert(lines, "  - No unpushed commits")
        table.insert(lines, "")
        table.insert(lines, "Press <CR> to delete this worktree")
      end

      ctx.preview:set_lines(lines)
      ctx.preview:highlight { ft = "markdown" }
      ctx.preview:set_title(wt.branch or "detached")

      return true
    end,
    actions = {
      delete_normal = function(self, item)
        local wt = item.data
        local has_changes, status_msg = check_worktree_status(wt.path)

        if has_changes then
          snacks.notify.notify(
            "Cannot delete: worktree has "
              .. status_msg
              .. "\nUse <C-f> to force delete (will lose changes)",
            {
              title = "Git Worktree",
              level = "warn",
            }
          )
          return
        end

        self:close()

        -- Confirm force delete
        local confirm_msg = "Delete worktree?"
        if has_changes then
          confirm_msg = "⚠ Delete worktree with " .. status_msg .. "?"
        end

        local choice = vim.fn.confirm(confirm_msg, "&Yes\n&No", 2, "Warning")
        if choice ~= 1 then return end

        local success, message = delete_worktree(wt.path, false)
        if success then
          snacks.notify.notify(message, {
            title = "Git Worktree",
            level = "info",
          })
        else
          snacks.notify.notify(message, {
            title = "Git Worktree",
            level = "error",
          })
        end
      end,
      delete_force = function(self, item)
        local wt = item.data
        local has_changes, status_msg = check_worktree_status(wt.path)

        -- Confirm force delete
        local confirm_msg = "Force delete worktree?"
        if has_changes then
          confirm_msg = "⚠ Force delete worktree with " .. status_msg .. "?"
        end

        local choice = vim.fn.confirm(confirm_msg, "&Yes\n&No", 2, "Warning")
        if choice ~= 1 then return end

        self:close()

        local success, message = delete_worktree(wt.path, true)
        if success then
          snacks.notify.notify(message, {
            title = "Git Worktree",
            level = "info",
          })
        else
          snacks.notify.notify(message, {
            title = "Git Worktree",
            level = "error",
          })
        end
      end,
    },
    win = {
      input = {
        keys = {
          ["<CR>"] = { "delete_normal", mode = { "n", "i" } },
          ["<C-f>"] = { "delete_force", mode = { "n", "i" } },
        },
      },
      list = {
        keys = {
          ["<CR>"] = { "delete_normal", mode = { "n", "i" } },
          ["<C-f>"] = { "delete_force", mode = { "n", "i" } },
        },
      },
    },
  }
end

--- Get the current buffer's path relative to the git root
---@param git_root string
---@return string|nil relative_path
local function get_current_buffer_context(git_root)
  local current_buf = vim.api.nvim_buf_get_name(0)
  if current_buf == "" or not git_root then return nil end

  local buf_path = Path:new(current_buf)
  local root_path = Path:new(git_root)

  -- Only proceed if the file is actually inside the current git root
  if buf_path:absolute():find(root_path:absolute(), 1, true) then
    return buf_path:make_relative(root_path:absolute())
  end
  return nil
end

-- HOOK 1: PRE-SWITCH
-- Purpose: Capture the current file path relative to the OLD worktree root
M.on(M.Events.SWITCH_PRE, function(data)
  -- 'data.context' is shared with SWITCH_POST
  local git_root = get_git_root()
  if git_root then data.context.file = get_current_buffer_context(git_root) end
end)

-- HOOK 2: POST-SWITCH
-- Purpose: Open the captured file path in the NEW worktree
M.on(M.Events.SWITCH_POST, function(data)
  local file_rel = data.context.file

  if file_rel then
    -- Construct the full path in the new worktree
    local new_file_path = Path:new(data.path):joinpath(file_rel)

    if new_file_path:exists() then
      vim.cmd("e " .. vim.fn.fnameescape(new_file_path:absolute()))
    else
      -- Fallback: If file doesn't exist in new branch (e.g. deleted), open root
      vim.cmd "e ."
    end
  end
end)

M.on(M.Events.SWITCH_POST, function(data)
  -- Reload file to ensure treesitter attaches to the new root
  vim.cmd "checktime"

  -- Restart LSP servers if available
  if vim.fn.exists ":LspRestart" == 2 then vim.cmd "LspRestart" end
end)

--  Setup keymaps (optional - call this in your config)
function M.setup()
  vim.keymap.set(
    "n",
    "<leader>gw",
    M.switch_worktree,
    { desc = "Git Worktree: Switch/Create" }
  )
  vim.keymap.set(
    "n",
    "<leader>gW",
    M.delete_worktree,
    { desc = "Git Worktree: Delete" }
  )
end
return M
