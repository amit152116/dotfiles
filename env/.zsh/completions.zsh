# -i silently ignores "insecure" dirs — distrobox shares $HOME as group-writable via uid/gid maps
autoload -Uz compinit bashcompinit
_zcompdump="${ZDOTDIR:-$HOME}/.zcompdump"
# -C skips security check, uses dump cache; regenerate only if >24h old
if [[ -n "$_zcompdump"(#qN.mh+24) ]]; then
    compinit -i
else
    compinit -i -C
fi
unset _zcompdump
# no-op: ROS/colcon setup scripts call compinit again, already initialized above
function compinit() { :; }
bashcompinit
zinit cdreplay -q

# cached tool completions; regen when binary newer (avoids spawning each tool every startup)
_zsh_cache="${XDG_CACHE_HOME:-$HOME/.cache}/zsh"
[[ -d "$_zsh_cache" ]] || mkdir -p "$_zsh_cache"
# usage: [FN=1] _cache_completion <cmd> <generator...>; FN=1 = bare #compdef function file (rg), autoloaded not sourced
_cache_completion() {
    local name=$1 f="$_zsh_cache/$1-completion.zsh"
    shift
    (($+commands[$name])) || return
    [[ -n $FN ]] && f="$_zsh_cache/fn/_$name"
    [[ -s $f && ! ${commands[$name]} -nt $f ]] || { mkdir -p "${f:h}" && "$@" >|"$f" 2>/dev/null; } || {
        rm -f "$f"
        return
    }
    if [[ -n $FN ]]; then
        fpath+=("${f:h}")
        autoload -Uz "_$name"
        compdef "_$name" "$name"
    else
        source "$f"
    fi
}
_cache_completion uv uv generate-shell-completion zsh
_cache_completion uvx uvx --generate-shell-completion zsh
_cache_completion ruff ruff generate-shell-completion zsh
_cache_completion bun bun completions
FN=1 _cache_completion rg rg --generate complete-zsh
_cache_completion kubectl kubectl completion zsh
_cache_completion pip pip completion --zsh
_cache_completion glow glow completion zsh
_cache_completion lazygit lazygit completion zsh
_cache_completion codex codex completion zsh
_cache_completion opencode opencode completion
FN=1 _cache_completion rustup rustup completions zsh
FN=1 _cache_completion cargo rustup completions zsh cargo
unfunction _cache_completion
unset _zsh_cache

__tmux_resurrect_complete() {
    local cur dir sessions
    cur=${words[CURRENT]} # what user typed so far
    dir="$HOME/.tmux/resurrect"
    [[ -d $dir ]] || return

    # collect files -> keep full filenames (just strip path)
    sessions=(${dir}/tmux_resurrect_*.txt(N))
    sessions=(${sessions##*/}) # remove path, keep prefix and suffix

    local matches=()
    for s in $sessions; do
        if [[ $s == $cur* ]]; then
            matches+=$s
        fi
    done

    compadd -Q -d "Resurrect sessions" -- $matches
}

compdef __tmux_resurrect_complete tmux-resurrect
