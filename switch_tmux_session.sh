#!/bin/bash
index="$1"
session=$(tmux list-sessions -F "#{session_name}" | sed -n "${index}p")

if [ -n "$session" ]; then
  tmux switch-client -t "$session"
else
  echo "No session at index $index" >&2
fi
