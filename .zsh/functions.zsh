
# C++ DEBUGGING VARIABLES
enable_sanitizers() {
  export ASAN_OPTIONS="new_delete_type_mismatch=0:detect_leaks=1:strict_init_order=1:check_initialization_order=1:symbolize=1:verbosity=1"
  export TSAN_OPTIONS="report_signal_unsafe=0:history_size=7:second_deadlock_stack=1:verbosity=1"
  export MSAN_OPTIONS="verbosity=1"
  echo "C++ sanitizers enabled ✅"
}

disable_sanitizers() {
  unset ASAN_OPTIONS TSAN_OPTIONS MSAN_OPTIONS
  echo "C++ sanitizers disabled ❌"
}

# Configuration reload
reload() {
  source  ~/.zshrc
  
  # Reload tmux configuration if available
  if command -v tmux &>/dev/null && [[ -f ~/.tmux.conf ]]; then
    # Use command substitution to prevent interference with fzf
    command tmux source-file ~/.tmux.conf 2>/dev/null
    echo "Reloaded ~/.zshrc & ~/.tmux.conf"
  else
    echo "Reloaded ~/.zshrc"
  fi
}

# Function to go to the root directory of the current Git repository
git-home() {
    local git_root
    git_root=$(git rev-parse --show-toplevel 2>/dev/null)
    
    if [[ -n "$git_root" ]]; then
        cd "$git_root" || echo "Failed to navigate to Git root"
    else
        echo "Not inside a Git repository."
    fi
}

# Function to load tmux resurrect sessions
tmux_resurrect() {
  local dir="$HOME/.tmux/resurrect"

  if [[ ! -d $dir ]]; then
    echo "No tmux-resurrect directory found at $dir"
    return 1
  fi

  local session_file="$dir/$1"

  if [[ ! -f $session_file ]]; then
    echo "Resurrect file not found: $session_file"
    return 1
  fi

  echo "Loading tmux resurrect session: $1"
  TMUX_PLUGIN_MANAGER_PATH="$HOME/.tmux/plugins" \
    bash "$HOME/.tmux/plugins/tmux-resurrect/scripts/restore.sh" "$session_file"
}

# Function to link the latest non-empty tmux session file
_link_tmux_session() {
  local dir="$HOME/.tmux/resurrect"
  [[ -d "$dir" ]] || { echo "Resurrect dir not found: $dir"; return 1 }

  # Find the latest non-empty resurrect file
  local target
  target=$(find "$dir" -type f -name 'tmux_resurrect_*.txt' -size +0c -printf "%T@ %p\n" \
           | sort -nr \
           | awk 'NR==1 {print $2}')

  if [[ -z "$target" ]]; then
    echo "No non-empty tmux_resurrect files found."
    return 1
  fi

  ln -sf "$target" "$dir/last"
  echo "Symlink 'last' now points to: ${target}"
}

# Override ta wrapper
ta() {
  if [[ -z $1 ]] || [[ ${1:0:1} == '-' ]]; then
    _zsh_tmux_plugin_run attach "$@" || {
      echo "Attach failed, trying to fix resurrect link..."
      _link_tmux_session && _zsh_tmux_plugin_run attach "$@"
    }
  else
    _zsh_tmux_plugin_run attach -t "$@" || {
      echo "Attach failed, trying to fix resurrect link..."
      _link_tmux_session && _zsh_tmux_plugin_run attach -t "$@"
    }
  fi
}

# Universal env manager
env() {
  # If no arguments → prompt with fzf to select env
  if [[ -z "$1" ]]; then
    local selection venvs conda_envs
    venvs=$(find . -maxdepth 1 -type d -exec test -f "{}/bin/activate" ';' -print | sed 's|^\./||' | awk '{print "[venv] " $0}')
    conda_envs=$(conda env list | awk 'NR>2 && $1 != "" && $1 != "#" {print "[conda] " $1}')

    selection=$(printf "%s\n%s\n" "$venvs" "$conda_envs" | fzf --no-preview)

    if [[ -z "$selection" ]]; then
      return
    fi

    if [[ "$selection" == "[venv]"* ]]; then
      source "${selection#\[venv\] }/bin/activate"
    else
      conda activate "${selection#\[conda\] }"
    fi
    return
  fi

  # If argument given as "deactivate"
  if [[ "$1" == "deactivate" ]]; then
    if [[ -n "$VIRTUAL_ENV" ]]; then
      deactivate
    elif [[ -n "$CONDA_DEFAULT_ENV" ]]; then
      conda deactivate
    else
      echo "⚠️ No environment active"
    fi
    return
  fi

  # If argument is given and looks like venv or conda env
  if [[ -f "$1/bin/activate" ]]; then
    source "$1/bin/activate"
  elif conda env list | awk 'NR>2 {print $1}' | grep -qx "$1"; then
    conda activate "$1"
  else
    echo "⚠️ Unknown environment: $1"
  fi
}

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!

conda(){
    unset -f conda
    if command -v conda >/dev/null 2>&1; then
      __conda_setup="$("$(command -v conda)" shell.zsh hook 2>/dev/null)"
      if [ $? -eq 0 ]; then
        eval "$__conda_setup"
      fi
      unset __conda_setup
    fi

    # Fallback
    if ! command -v conda >/dev/null 2>&1; then
      if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
        source "$HOME/miniconda3/etc/profile.d/conda.sh"
      elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
        source "$HOME/anaconda3/etc/profile.d/conda.sh"
      else
        echo "conda not found" >&2
        return 127
      fi
    fi
    unset __conda_setup
    conda "$@"
}

