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

idf() {
    # Add esp-idf if installed
    if [[ -d "$HOME/esp-idf" ]]; then
        source $HOME/esp-idf/export.sh
    fi
    unset -f idf
    alias idf="idf.py"
}

# Lazy-load nvm — node is already on PATH above; nvm() only needed if switching versions.
nvm() {
    unset -f nvm
    [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
    [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"
    nvm "$@"
}

# Configuration reload
reload() {
    source ~/.zshrc

    # Reload tmux configuration if available
    if command -v tmux &>/dev/null && [[ -f ~/.tmux.conf ]]; then
        # Use command substitution to prevent interference with fzf
        command tmux source-file ~/.tmux.conf 2>/dev/null
        echo "Reloaded ~/.zshrc & ~/.tmux.conf"
    else
        echo "Reloaded ~/.zshrc"
    fi
}

# Function to load tmux resurrect sessions
tmux-resurrect() {
    local dir="$HOME/.tmux/resurrect"

    if [[ ! -d $dir ]]; then
        echo "No tmux-resurrect directory found at $dir"
        return 1
    fi

    local session_file="$dir/tmux_resurrect_$1.txt"

    if [[ ! -f $session_file ]]; then
        echo "Resurrect file not found: $session_file"
        return 1
    fi

    echo "Loading tmux resurrect session: $1"
    TMUX_PLUGIN_MANAGER_PATH="$HOME/.tmux/plugins" \
        bash "$HOME/.tmux/plugins/tmux-resurrect/scripts/restore.sh" "$session_file"
}

# Function to link the latest non-empty tmux session file
__link_tmux_session() {
    local dir="$HOME/.tmux/resurrect"
    [[ -d "$dir" ]] || {
        echo "Resurrect dir not found: $dir"
        return 1
    }

    # Find the latest non-empty resurrect file
    local target
    target=$(find "$dir" -type f -name 'tmux_resurrect_*.txt' -size +0c -printf "%T@ %p\n" |
        sort -nr |
        awk 'NR==1 {print $2}')

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
            __link_tmux_session && _zsh_tmux_plugin_run attach "$@"
        }
    else
        _zsh_tmux_plugin_run attach -t "$@" || {
            echo "Attach failed, trying to fix resurrect link..."
            __link_tmux_session && _zsh_tmux_plugin_run attach -t "$@"
        }
    fi
}

if command -v zoxide >/dev/null 2>&1; then
    function cd() {
        __zoxide_z "$@"
    }
fi

# Refresh display/session env from tmux on every prompt.
# tmux-resurrect restores panes carrying stale DISPLAY/WAYLAND_DISPLAY from a
# dead login session; update-environment only fixes the *session* env for new
# panes, never already-running shells. This re-exports the live values that
# tmux holds into the current shell so clipboard, GUI launches, and ssh-agent
# keep working after a reboot/restore.
if [[ -n "$TMUX" ]]; then
    function _refresh_tmux_env() {
        # Single tmux call (-s = shell syntax) instead of one per variable.
        eval "$(tmux show-environment -s 2>/dev/null |
            grep -E '^(export )?(DISPLAY|WAYLAND_DISPLAY|XAUTHORITY|XDG_SESSION_TYPE|SSH_AUTH_SOCK)=')"
        if [[ -n "$WAYLAND_DISPLAY" && -S "${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}" ]]; then
            add-zsh-hook -d precmd _refresh_tmux_env
        fi
    }
    autoload -Uz add-zsh-hook
    add-zsh-hook precmd _refresh_tmux_env
fi
