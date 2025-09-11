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


# Man FZF
if [[ ! -n "$TMUX" ]]; then
  __man_fzf(){
    man-pages
  }

  zle -N __man_fzf
  bindkey '\em' __man_fzf
fi


if [[ ! -n "$TMUX" ]]; then

# Define ZLE widget to exit Zsh
__exit_zsh() {
  exit
}

zle -N __exit_zsh
bindkey '\eq' __exit_zsh   # Alt+Q
  
fi

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


# TMUX BINDINGS
if [[ -n "$TMUX" ]]; then

  __tmux_kill_pane() {
    current_pane=$TMUX_PANE
    panes=$(tmux list-panes -s | wc -l)

    # Only switch if there is more than 1 pane
    if [ "$panes" -eq 1 ]; then
      if ! tmux switch-client -l 2>/dev/null; then
        tmux switch-client -p
      fi
    fi

    # Kill the current pane
    tmux kill-pane -t "$current_pane"
  }

  zle -N __tmux_kill_pane

  bindkey '\eq' __tmux_kill_pane

fi

