#!/usr/bin/env bash

ENVS=$(tmux show-env | awk '!/^-/{printf "export %s; ", $0}')

tmux list-panes -a -F "#{session_name}:#{window_index}.#{pane_index} #{pane_current_command}" |
    while read pane_process; do
        IFS=' ' read -ra pane_process <<<"$pane_process"
        pane="${pane_process[0]}"
        cmd="${pane_process[1]}"

        case "$cmd" in
        bash | zsh | fish | sh) # only shells
            tmux send-keys -t "$pane" "$ENVS" Enter
            ;;
        esac
    done
