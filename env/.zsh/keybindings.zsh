# Make Ctrl-P go to previous matching command
bindkey '^P' history-beginning-search-backward

# Make Ctrl-N go to next matching command
bindkey '^N' history-beginning-search-forward


bindkey '^R' fzf-history-widget
# bindkey -r '^S'  # remove Ctrl+S binding

setopt NO_NOTIFY      # don’t print “done” when background jobs finish
setopt NO_BG_NICE     # don’t lower priority of background jobs
unsetopt MONITOR      # disable job control entirely (optional, also hides [&] messages)

# Open file explorer in current directory
__open_file_explorer() {
    {
        xdg-open .
        } always {
        zle reset-prompt
    }
}
zle -N __open_file_explorer
bindkey '^o' __open_file_explorer



# Define a ZLE widget
__silent_run() {
    # Run the current buffer silently
    eval "$BUFFER" &>/dev/null
    # Remove the command from the line
    zle kill-whole-line
    # Redraw prompt
    zle reset-prompt
}
zle -N __silent_run
bindkey '^B' __silent_run

# Define ZLE widget to cleanly exit Zsh
__exit_zsh() {
    # Clear whatever is currently typed, write "exit", and press Enter
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


# TMUX BINDINGS
if [[ -n "$TMUX" ]]; then

    # Resolve a usable tmux. Inside distrobox the tmux server runs on the host
    # and the binary isn't installed in the container, so route via the host.
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

        # Only switch if there is exactly 1 pane in the session
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


    # Open lazygit in current directory via tmux-sessionizer
    __tmux_lazygit(){
        local current_dir="${PWD}"
        BUFFER=""
        zle reset-prompt
        tmux neww "cd '$current_dir' && tmux-sessionizer -c lazygit -- -w ./" &>/dev/null
    }
    zle -N __tmux_lazygit

    bindkey '\eg' __tmux_lazygit   # Alt+G

    # Open yazi in current directory via tmux-sessionizer
    __tmux_yazi(){
        local current_dir="${PWD}"
        BUFFER=""
        zle reset-prompt
        tmux neww "cd '$current_dir' && tmux-sessionizer -c yazi" &>/dev/null
    }
    zle -N __tmux_yazi
    bindkey '\ey' __tmux_yazi   # Alt+Y

    # Open glow in current directory via tmux-sessionizer
    __tmux_glow(){
        local current_dir="${PWD}"
        BUFFER=""
        zle reset-prompt
        tmux neww "cd '$current_dir' && tmux-sessionizer -c glow" &>/dev/null
    }
    zle -N __tmux_glow
    bindkey '\ed' __tmux_glow   # Alt+D

else
    # Not in tmux, just exit the shell
    bindkey '\eq' __exit_zsh         # Alt+Q
fi

# Alt+E → toggle between host and the repo's distrobox container in this pane.
# Host  -> runs `box` (enters the marker's container).
# Box   -> runs `exit` (returns to the host shell underneath).
# (Alt+C is taken by fzf's cd widget.)
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

# zsh-vi-mode calls zvm_init() on first precmd, wiping all keymaps above.
# Called via ZVM_AFTER_INIT_COMMANDS after zvm_init() completes.
_zvm_rebind_custom_keys() {
    # Bump timeout so terminal Alt+key ESC sequences aren't swallowed by vi mode-switch
    KEYTIMEOUT=15

    bindkey -M viins '^P' history-beginning-search-backward
    bindkey -M vicmd '^P' history-beginning-search-backward
    bindkey -M viins '^N' history-beginning-search-forward
    bindkey -M vicmd '^N' history-beginning-search-forward
    bindkey -M viins '^o' __open_file_explorer
    bindkey -M viins '^B' __silent_run
    bindkey -M viins '^F' __fzf_repo_cd
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
