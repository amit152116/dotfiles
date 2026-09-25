# AGENTS.md — Dotfiles Repository Guide

This repository contains personal dotfiles and Neovim configuration for Pop!_OS.
Primary languages: **Lua** (Neovim plugins/config), **Bash/Zsh** (shell scripts).

---

## Repository Layout

```
.
├── nvim/                    # Neovim configuration (AstroNvim v5 base)
│   ├── init.lua             # Bootstrap lazy.nvim
│   ├── lua/
│   │   ├── plugins/         # Plugin specs (one file per plugin)
│   │   ├── git_flow/        # Custom git-flow plugin
│   │   ├── remote_sync/     # Custom rsync/SSH sync plugin
│   │   ├── myPlugins/       # Miscellaneous local plugins
│   │   └── utils/           # Shared utilities (jobRunner, helper)
│   └── plugin/              # Entry-point loaders (deferred setup guards)
├── env/                     # Shell environment
│   ├── .zshrc               # Zsh entry point (sources .zsh/*.zsh modules)
│   └── .zsh/                # Modular zsh config (aliases, exports, functions…)
├── installs/                # Idempotent install scripts (one per tool)
├── scripts/                 # Utility scripts on PATH
├── templates/               # CMake / clang-format templates
├── alacritty/               # Alacritty terminal config
├── btop/ yazi/              # btop / yazi configs
└── setup                    # Main bootstrapper (run once per machine)
```

---

## Lint / Format Commands

No build system exists. Run tools directly:

### Lua (inside `nvim/`)
```bash
# Format a single file
stylua nvim/lua/remote_sync/sync.lua

# Format all Lua files
stylua nvim/

# Lint a single file
selene nvim/lua/remote_sync/sync.lua

# Lint all Lua files
selene nvim/
```

Config lives in `nvim/.stylua.toml` and `nvim/selene.toml`.
Run both from the `nvim/` directory so the config files are picked up automatically.

### Shell scripts
```bash
# Check a single script
shellcheck installs/docker

# Format a single script (writes in-place)
shfmt -w installs/docker

# Show diff without writing
shfmt -d installs/docker
```

### No test suite
There is no automated test framework. Verify shell scripts by running them in a
safe environment; verify Lua plugins by launching Neovim and exercising commands.

---

## Lua Code Style

Derived from `nvim/.stylua.toml`:

| Setting | Value |
|---|---|
| Column width | 80 |
| Indent | 2 spaces |
| Line endings | Unix (LF) |
| Quotes | Double preferred (`AutoPreferDouble`) |
| Call parentheses | Omitted for single string/table args |
| Simple statements | Collapsed to one line (`Always`) |

### Module pattern
Every file exposes a table `M`:
```lua
local M = {}
-- ... functions ...
return M
```

### Imports
```lua
-- No parens; double quotes; dot-separated path
local config = require "remote_sync.config"
local utils  = require "remote_sync.utils"
```

### LuaDoc annotations
Use EmmyLua/LuaLS annotations on all public functions and types:
```lua
---@param args string[] Git subcommand arguments
---@param cwd string|nil Working directory (nil = current buffer dir)
---@return string output Trimmed stdout
---@return number code Exit code (0 = success)
function M.git_sync(args, cwd) … end
```
Annotate classes with `---@class`, enums with `---@enum`, aliases with `---@alias`.

### Naming conventions
- Variables, functions, module files: `snake_case`
- Module names (require paths): `snake_case` (e.g. `remote_sync`, `git_flow`)
- Enum/constant keys: `UPPER_CASE` (e.g. `M.State.IDLE`)
- Private functions: `local function name()` (unexported)
- Public functions: `function M.name()` (exported via `M`)

### Single-line functions
Collapse trivial getters:
```lua
function M.is_busy() return state.current ~= M.State.IDLE end
```

### Async jobs
Use `vim.fn.jobstart()` for async shell commands.
Always wrap callbacks that touch Neovim state with `vim.schedule_wrap()`:
```lua
state.job_id = vim.fn.jobstart(cmd, {
  on_exit = vim.schedule_wrap(function(_, code) … end),
})
```

### Error handling
- Guard preconditions with early returns; never nest deeply.
- Always invoke the caller's callback before returning on failure.
- Use `utils.error()` / `utils.warn()` / `utils.info()` for user notifications
  (maps to `vim.notify` with appropriate log levels).
- Check exit codes from every system call; surface stderr to the user.

```lua
if not proj_config then
  utils.error "No remote sync config found."
  if callback then callback(false) end
  return
end
```

### File headers
Start plugin files with a triple-dash doc block:
```lua
---
--- module_name - one-line description
--- Additional context or caveats.
---
```

### Custom plugin structure
Each plugin under `nvim/lua/<plugin>/` must have:
- `init.lua` — public API; exports `setup()` and re-exports submodules
- `config.lua` — `M.defaults` table; `M.get_value(key)` accessor
- `commands.lua` — all `vim.api.nvim_create_user_command` calls
- `nvim/plugin/<plugin>.lua` — deferred entry point (VimEnter guard, idempotent)

---

## Bash / Zsh Code Style

### Script header (required)
```bash
#!/usr/bin/env bash
set -euo pipefail
```

### Logging helpers
```bash
log()  { echo "[INFO] $1"; }
warn() { echo "[WARN] $1"; }
err()  { echo "[ERROR] $1" >&2; }
```

### Naming conventions
- Functions and local variables: `snake_case`
- Environment / exported variables: `UPPER_CASE`
- Script files: `kebab-case` (scripts/) or `snake_case` (installs/)

### Idempotency
Always check before acting:
```bash
if ! command -v nvim >/dev/null 2>&1; then
  sudo snap install nvim --classic
fi
```

### Temp directories
```bash
TMPDIR=$(mktemp -d)
trap 'rm -rf "$TMPDIR"' EXIT
```

### Quoting
Always double-quote variable expansions: `"$VAR"`, `"${ARRAY[@]}"`.

### Redirect stderr
```bash
some_cmd >/dev/null 2>&1   # silence both
some_cmd 2>/dev/null       # silence stderr only
some_cmd >&2               # write to stderr
```

---

## Git Workflow

- Default branch: `master`
- Submodules enabled (`submodule.recurse = true`)
- Commit style: imperative mood, present tense (`Add`, `Fix`, `Update`)
- The repo auto-commits via `auto_commit.sh` (cron job) — manual commits should
  be intentional and descriptive.
- Use `git_common.sh` utilities for shared git operations in scripts.

---

## Key Conventions Summary

1. **Lua**: 2-space indent, 80-col wrap, double quotes, no parens on single args.
2. **Bash**: `set -euo pipefail`, idempotent, `[INFO]`/`[ERROR]` prefixes.
3. **Modules**: `local M = {} … return M` pattern everywhere.
4. **Annotations**: LuaDoc on all public APIs.
5. **Async**: `vim.fn.jobstart` + `vim.schedule_wrap`; never block the main loop.
6. **Errors**: Early-return style; always fire callbacks before returning.
7. **No tests**: Manual verification in Neovim / a live shell session.
