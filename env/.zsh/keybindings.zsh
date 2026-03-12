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

    __tmux_kill_pane() {
        local current_pane=$TMUX_PANE
        local panes=$(tmux list-panes -s | wc -l)

        # Only switch if there is exactly 1 pane in the session
        if [ "$panes" -eq 1 ]; then
            if ! tmux switch-client -l 2>/dev/null; then
                tmux switch-client -p
            fi
        fi

        # Exit the shell, which will naturally close the pane/popup
        __exit_zsh
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
