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
function compinit() { : }
bashcompinit
zinit cdreplay -q

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
