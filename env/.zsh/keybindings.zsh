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
  # tmux-sessionizer (new window)
  __tmux_sessionizer_new() {
    tmux neww tmux-sessionizer
  }
  zle -N __tmux_sessionizer_new
  bindkey '^f' __tmux_sessionizer_new

  # tmux-sessionizer -s 0..3
  __tmux_sessionizer_s0() { tmux-sessionizer -s 0 }
  __tmux_sessionizer_s1() { tmux-sessionizer -s 1 }
  __tmux_sessionizer_s2() { tmux-sessionizer -s 2 }
  __tmux_sessionizer_s3() { tmux-sessionizer -s 3 }

  zle -N __tmux_sessionizer_s0
  zle -N __tmux_sessionizer_s1
  zle -N __tmux_sessionizer_s2
  zle -N __tmux_sessionizer_s3

  bindkey '\em' __tmux_sessionizer_s0
  bindkey '\eb' __tmux_sessionizer_s1
  bindkey '\en' __tmux_sessionizer_s2
  bindkey '\es' __tmux_sessionizer_s3

  __tmux_kill_pane() {
    current_pane=$TMUX_PANE
    panes=$(tmux list-panes -s | wc -l)

    # Only switch if there is more than 1 pane
    if [ "$panes" -eq 1 ]; then
      tmux switch-client -p
    fi

    # Kill the current pane
    tmux kill-pane -t "$current_pane"
  }

  zle -N __tmux_kill_pane

  bindkey '\eq' __tmux_kill_pane

  __tmux_switch_session(){
    tmux switch-client -l
  }
  zle -N __tmux_switch_session
  bindkey '\el' __tmux_switch_session

  __tmux_switch_window(){
    tmux last-window
  }
  zle -N __tmux_switch_window
  bindkey '\ew' __tmux_switch_window

fi

