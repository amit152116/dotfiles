# auto-activates nearest .venv on cd; deactivates on leaving the tree, but only venvs this hook activated
# (a manually `source`d venv is left untouched)

_venv_auto() {
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
