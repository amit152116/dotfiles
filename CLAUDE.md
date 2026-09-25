# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

Personal dotfiles for Pop!\_OS. Primary languages: **Lua** (Neovim config/plugins) and **Bash/Zsh** (shell scripts).

## Setup & Deployment

Symlinks are managed by `symlink.sh` via `createSymlink()`. Run `./setup` once per machine to bootstrap dependencies and create symlinks. There is no package.json, Makefile, or build step.

Files in `config/` are symlinked into `~/.config/`. Files in `env/` are symlinked to `$HOME` (e.g. `env/.zshrc` → `~/.zshrc`). Scripts in `scripts/` land on `$PATH`.

## Lint Commands

```bash
# Lua (inside config/nvim/)
stylua --check .
luacheck lua/

# Shell
shellcheck scripts/<file>
shfmt -d env/.zsh/*.zsh
```

No test suite exists.

## Tag Caveats In-Code

**Caveat / assumption / deferred work → tag comment at the code site. Not just chat.**

Format: `TAG(scope): what + why + fix`. One line. Use nvim `todo-comments` tags:

`FIX` bug to repair (`FIXME`/`BUG`/`ISSUE`) · `TODO` deferred work · `HACK` fragile shortcut · `WARN` footgun (`XXX`) · `PERF` hot-path cost (`OPTIM`) · `NOTE` rationale (`INFO`) · `TEST` weak coverage · `MEM` alloc/pointer/leak · `UI` Qt/GCS (`QT`/`GUI`) · `REVIEW` cleanup (`REFACTOR`)

Tightest tag — defect = `FIX` not `TODO`; cost = `PERF` not `NOTE`. List tags added in summary.

## Comment Style

**Compact, lowercase, one-liner. Only WHY non-obvious — never WHAT (code already says it).**

- No multi-line prose blocks, no doxygen essays, no section-banner ASCII dividers.
- Compress multi-sentence comment → single short line above (or trailing) the code.
- Drop comments restating the identifier/obvious behavior entirely (e.g. `// Logger` above `logger;`).
- Keep: non-obvious invariant, hidden constraint, race/safety reasoning, perf tradeoff.
- Genuinely multi-line content (class-level usage doc, multi-step rationale) → one `/* ... */` block, not stacked `//` lines.

## Architecture

### Neovim (`config/nvim/`)

AstroNvim v5 on top of lazy.nvim. Entry: `init.lua` → `lua/lazy_setup.lua`.

- `lua/plugins/` — one spec file per plugin (lazy.nvim format)
- `lua/remote_sync/` — custom rsync/SSH sync plugin
- `lua/git_flow/` — custom git-flow plugin
- `lua/myPlugins/` — miscellaneous local plugins
- `lua/utils/jobRunner.lua` — async job wrapper; use this, not raw `vim.fn.jobstart()`
- `lua/utils/helper.lua` — shared helpers
- `plugin/<name>.lua` — deferred entry points (VimEnter guard, idempotent)

Custom plugin layout (required): `init.lua` (public API + `setup()`), `config.lua` (`M.defaults`, `M.get_value(key)`), `commands.lua` (`nvim_create_user_command` calls), `plugin/<name>.lua` (deferred loader).

### Zsh (`env/.zsh/`)

Modular — `.zshrc` sources each module in order:

1. `exports.zsh` — PATH, env vars (Cargo, CUDA, Go, Android SDK, Gazebo, ROS)
2. `aliases.zsh` — shell aliases + CMake/Makefile helpers
3. `functions.zsh` — tmux resurrect helpers, `envm` universal env manager
4. `keybindings.zsh` — ZLE widgets + tmux-aware bindings
5. `taskwarrior.zsh` — tmux-scoped task functions

### Tmux (`.tmux.conf`)

Prefix: `Ctrl+A`. Plugin manager: TPM. Key shortcuts (all Alt/Meta):

| Key     | Action                                        |
| ------- | --------------------------------------------- |
| `Alt+F` | tmux-sessionizer (project picker)             |
| `Alt+G` | lazygit in current dir                        |
| `Alt+Y` | yazi file manager                             |
| `Alt+D` | glow markdown viewer                          |
| `Alt+,` | agent-selector (claude/codex/gemini/opencode) |
| `Alt+T` | task dashboard popup                          |
| `Alt+L` | switch to last session                        |
| `Alt+/` | switch to last window                         |

`tmux-is-shell` script detects whether the current pane is at a shell prompt vs. running a program — used by bindings that behave differently in each context.

### Taskwarrior (`env/.zsh/taskwarrior.zsh`)

Tasks are scoped to the current tmux session. Functions: `tq` (fast capture), `tstart`/`tstop`, `tfocus`/`tpark`/`treturn` (focus stack), `twhere` (show state), `tlog`.

### Scripts (`scripts/`)

Split into two PATH dirs (both on `$PATH`, see `env/.zsh/exports.zsh` and `env/.tmux.conf`):

- `scripts/bin/` — general-purpose tools, meant to be typed by hand from any project dir: `build`/`cbuild`/`rosbuild`, `builder/` (build tool's python package), `claude2`/`codex2`, `cmake-init`, `cmake.mk`, `fzf-submodule`, `gitignore`, `help-fzf`/`man-fzf`/`tldr-fzf`, `scangit`, `transfer`, `mesh_network`, `share_internet`, `firstboot`, `manage_chroot`, `setup_noip_ddns`, `notebooklm/`
- `scripts/internal/` — dotfiles-internal glue, only ever called from `.tmux.conf`/`.zsh/*` config, never typed manually: `tmux-sessionizer` (+ `tmux-sessionizer.conf`), `tmux-is-shell`, `tmux-is-vim`, `tmux-clip`, `tmux-swap-window`, `tmux-broadcast`, `tmux-task-status`, `tmux-resurrect-prune`, `agent-selector`, `command-runner`, `task-dashboard-popup`, `task-status.sh`, `git-pager`

Both dirs are flat on PATH — a script moved into a further subdir (e.g. `builder/`, `notebooklm/`) is not itself PATH-resolvable, only the top-level entrypoint next to it is.

### Build tool (`scripts/bin/build`, `scripts/bin/builder/`)

`build` is a bash wrapper: it sources ROS (only for ROS workspaces) then execs
`python3 -m builder` with `PYTHONPATH` set to its own resolved directory (`SCRIPT_DIR`,
via `readlink -f "${BASH_SOURCE[0]}"`), so it works whether invoked via PATH or a symlink.
The python side has no standalone entry — `builder/__main__.py` is reached only through
the wrapper, because a sourced `setup.bash` can't cross into Python.

Backend auto-detected by walking up from cwd (stopping below `$HOME`): any `package.xml`
→ `ros` (colcon), else `CMakeLists.txt` → `cmake`. Override with `--backend ros|cmake`.
Layouts handled, in priority order: `src/` workspace, flat repo (packages at top level,
needs `.git` unless it's cwd), grouped subdirs (cwd only), single package. Detection is
duplicated in `build`; keep the two in sync — `scripts/bin/builder/detect.py` is the source of truth.

- `detect.py` — backend/root resolution
- `config.py` — `BuildConfig`, arg parser, `.build.json` persistence
- `cmake_flags.py` — shared cmake/colcon flag construction (sanitizers, coverage, LTO)
- `backends/ros.py` — colcon build, dep pre-build in Release, venv setup
- `backends/cmake.py` — cmake configure/build/install/ctest/clang-tidy+cppcheck
- `__main__.py` — action dispatch; `runner.py` — command echo + dry-run; `logger.py` — colored output

Backend-specific flags: `-p/--pkg`, `-f/--force-deps` (ros); `-t/--target`, `-d/--build-dir`,
`--compiler`, `--install`, `--lint` (cmake). `-i/--install-prefix` is a prefix, not an action.
`--test` differs per backend: ros alone = test-only, cmake = build then ctest.

## Code Conventions

### Lua

- `snake_case` for locals/params/functions; `UPPER_CASE` for module-level constants; `PascalCase` for metatables
- LuaDoc annotations on all public functions: `---@param name type`, `---@return type`
- Async jobs: always use `JobRunner.new(cmd, opts)` from `utils/jobRunner`
- Error handling: `vim.notify(msg, vim.log.levels.ERROR)` — no silent swallowing
- Module pattern: `local M = {}` … `return M`

### Bash/Zsh

- Shebang: `#!/usr/bin/env bash`
- Log helpers: `log_info`, `log_warn`, `log_error` (don't use raw `echo` for status messages)
- `snake_case` for local vars/functions; `UPPER_CASE` for env vars
- Scripts must be idempotent
- Temp dirs: use `$TMPDIR`, never `/tmp` directly
- Double-quote all variable expansions; redirect stderr with `2>/dev/null` or `2>&1`
- **Nested quoting**: when generating tmux/zsh commands that wrap other commands, verify escape layers explicitly — `shellescape` inside already-quoted tmux strings breaks filenames with spaces

## Git Workflow

- Default branch: `master`
- Commit style: imperative mood (`Add`, `Fix`, `Update`)
- `auto_commit.sh` runs via cron for automated commits — manual commits should be intentional
