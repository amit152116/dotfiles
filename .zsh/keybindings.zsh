# Make Ctrl-P go to previous matching command
bindkey '^P' history-beginning-search-backward

# Make Ctrl-N go to next matching command
bindkey '^N' history-beginning-search-forward


setopt NO_NOTIFY      # don’t print “done” when background jobs finish
setopt NO_BG_NICE     # don’t lower priority of background jobs
unsetopt MONITOR      # disable job control entirely (optional, also hides [&] messages)

# Open file explorer in current directory
__open_file_explorer() {
 {
    nohup xdg-open . >/dev/null 2>&1 &
  } always {
    zle reset-prompt
  }
}
zle -N __open_file_explorer
bindkey '^o' __open_file_explorer


export_display(){
    export DISPLAY=${DISPLAY}
}

zle -N export_display

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

  bindkey '\eh' __tmux_sessionizer_s0
  bindkey '\et' __tmux_sessionizer_s1
  bindkey '\en' __tmux_sessionizer_s2
  bindkey '\es' __tmux_sessionizer_s3

  # tmux-kill variants
  __tmux_kill_session() {
    tmux kill-session -t "$TMUX_PANE"; tmux switch-client -p
  }
  __tmux_kill_pane() {
    tmux kill-pane -t "$TMUX_PANE"; tmux switch-client -p
  }
  __tmux_kill_window() {
    tmux kill-window -t "$TMUX_PANE"; tmux switch-client -p
  }

  zle -N __tmux_kill_session
  zle -N __tmux_kill_pane
  zle -N __tmux_kill_window

  bindkey '\es' __tmux_kill_session
  bindkey '\ep' __tmux_kill_pane
  bindkey '\ew' __tmux_kill_window


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

# 2. Tell zsh this function is a ZLE widget
zle -N __silent_run

# 3. Bind the widget to Ctrl-B (you can choose any key)
bindkey '^B' __silent_run
