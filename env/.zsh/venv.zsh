# Auto-activate a project's Python venv on cd.
#
# Walks up from the current dir to the nearest `.venv/bin/activate` and sources
# it. Deactivates automatically when leaving the project tree — but only venvs
# this hook activated, so a manually `source`d venv is left untouched.

_venv_auto() {
    # Find nearest .venv walking up from cwd.
    local dir="$PWD" found=""
    while [[ "$dir" != "/" ]]; do
        if [[ -f "$dir/.venv/bin/activate" ]]; then
            found="$dir/.venv"
            break
        fi
        dir="${dir:h}"
    done

    if [[ -n "$found" ]]; then
        [[ "$VIRTUAL_ENV" == "$found" ]] && return   # already active
        source "$found/bin/activate"
        export _VENV_AUTO="$found"
    elif [[ -n "$_VENV_AUTO" ]]; then
        deactivate 2>/dev/null                        # left the tree
        unset _VENV_AUTO
    fi
}

autoload -Uz add-zsh-hook
add-zsh-hook chpwd _venv_auto
_venv_auto   # run once for the starting directory
