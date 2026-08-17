bindkey '^P' history-beginning-search-backward
bindkey '^N' history-beginning-search-forward
bindkey '^R' fzf-history-widget

# alt to right-arrow for accepting a suggestion
bindkey '^Y' autosuggest-accept
# bindkey -r '^S'  # remove Ctrl+S binding

setopt NO_NOTIFY      # don’t print “done” when background jobs finish
setopt NO_BG_NICE     # don’t lower priority of background jobs
unsetopt MONITOR      # disable job control entirely (optional, also hides [&] messages)

__open_file_explorer() {
    {
        xdg-open .
        } always {
        zle reset-prompt
    }
}
zle -N __open_file_explorer
bindkey '^o' __open_file_explorer



__silent_run() {
    eval "$BUFFER" &>/dev/null
    zle kill-whole-line
    zle reset-prompt
}
zle -N __silent_run
bindkey '^B' __silent_run

__exit_zsh() {
    zle kill-whole-line
    BUFFER="exit"
    zle accept-line
}
zle -N __exit_zsh

# FZF submodule selector — delegates to scripts/fzf-submodule, then cd's
__fzf_repo_cd() {
    local selected
    selected=$(fzf-submodule)
    if [[ -n "$selected" ]]; then
        cd "$selected"
        zle reset-prompt
    fi
}
zle -N __fzf_repo_cd
bindkey '^F' __fzf_repo_cd


if [[ -n "$TMUX" ]]; then

    # inside distrobox, tmux server runs on host and binary isn't in the container - route via host
    if command -v tmux >/dev/null 2>&1; then
        __tmux() { command tmux "$@"; }
    elif command -v distrobox-host-exec >/dev/null 2>&1; then
        __tmux() { distrobox-host-exec tmux "$@"; }
    else
        __tmux() { return 1; }
    fi

    __tmux_kill_pane() {
        local current_pane=$TMUX_PANE
        local panes=$(__tmux list-panes -s | wc -l)

        if [ "$panes" -eq 1 ]; then
            if ! __tmux switch-client -l 2>/dev/null; then
                __tmux switch-client -p
            fi
        fi

        # Kill the pane directly so nested shells (host -> box -> onhost -> ...)
        # all die at once instead of unwinding one `exit` per layer.
        __tmux kill-pane
    }
    zle -N __tmux_kill_pane

    bindkey '\eq' __tmux_kill_pane   # Alt+Q


    __tmux_lazygit(){
        local current_dir
        current_dir=$(git rev-parse --show-toplevel 2>/dev/null) || current_dir="${PWD}"
        BUFFER=""
        zle reset-prompt
        tmux neww "cd '$current_dir' && tmux-sessionizer -c lazygit -- -w ./" &>/dev/null
    }
    zle -N __tmux_lazygit

    bindkey '\eg' __tmux_lazygit   # Alt+G

    __tmux_yazi(){
        local current_dir="${PWD}"
        BUFFER=""
        zle reset-prompt
        tmux neww "cd '$current_dir' && tmux-sessionizer -c yazi" &>/dev/null
    }
    zle -N __tmux_yazi
    bindkey '\ey' __tmux_yazi   # Alt+Y

    __tmux_glow(){
        local current_dir="${PWD}"
        BUFFER=""
        zle reset-prompt
        tmux neww "cd '$current_dir' && tmux-sessionizer -c glow" &>/dev/null
    }
    zle -N __tmux_glow
    bindkey '\ed' __tmux_glow   # Alt+D

else
    bindkey '\eq' __exit_zsh         # Alt+Q
fi

# toggle host<->container: runs `box`/`exit` (Alt+C taken by fzf's cd widget)
__toggle_box() {
    zle kill-whole-line
    if (( IN_CONTAINER )); then
        BUFFER="exit"
    else
        BUFFER="box"
    fi
    zle accept-line
}
zle -N __toggle_box
bindkey '\ee' __toggle_box   # Alt+E

# Ctrl+Space → insert "tq " at prompt for fast task capture
bindkey -s '^@' 'tq '

# zsh-vi-mode's zvm_init() wipes all keymaps above on first precmd; re-applied via ZVM_AFTER_INIT_COMMANDS after it runs
_zvm_rebind_custom_keys() {
    # Bump timeout so terminal Alt+key ESC sequences aren't swallowed by vi mode-switch
    KEYTIMEOUT=20

    bindkey -M viins '^P' history-beginning-search-backward
    bindkey -M vicmd '^P' history-beginning-search-backward
    bindkey -M viins '^N' history-beginning-search-forward
    bindkey -M vicmd '^N' history-beginning-search-forward
    bindkey -M viins '^o' __open_file_explorer
    bindkey -M viins '^B' __silent_run
    bindkey -M viins '^F' __fzf_repo_cd
    bindkey -M viins '^Y' autosuggest-accept
    bindkey -M vicmd '^Y' autosuggest-accept
    bindkey -M viins -s '^@' 'tq '
    bindkey -M viins '\ee' __toggle_box
    bindkey -M vicmd '\ee' __toggle_box

    if [[ -n "$TMUX" ]]; then
        bindkey -M viins '\eq' __tmux_kill_pane
        bindkey -M vicmd '\eq' __tmux_kill_pane
        bindkey -M viins '\eg' __tmux_lazygit
        bindkey -M vicmd '\eg' __tmux_lazygit
        bindkey -M viins '\ey' __tmux_yazi
        bindkey -M vicmd '\ey' __tmux_yazi
        bindkey -M viins '\ed' __tmux_glow
        bindkey -M vicmd '\ed' __tmux_glow
    else
        bindkey -M viins '\eq' __exit_zsh
        bindkey -M vicmd '\eq' __exit_zsh
    fi
}
zvm_after_init_commands+=('_zvm_rebind_custom_keys')
zvm_after_lazy_keybindings_commands+=('_zvm_rebind_custom_keys')
