# autoload -U +X bashcompinit && bashcompinit
# autoload -U +X compinit && compinit
autoload -Uz bashcompinit; compinit
autoload -Uz compinit; compinit


# Alias completion
for alias_name in $(alias | awk -F= '{print $1}' | awk '{print $2}'); do
    compdef "$alias_name=$(alias "$alias_name" | sed "s/^.*='\([^']*\)'.*/\1/")"
done

# Autocomplete for tmux_resurrect
__tmux_resurrect_complete() {
  local dir sessions
  dir="$HOME/.tmux/resurrect"
  [[ -d $dir ]] || return

  # collect files -> strip prefix/suffix -> get bare session names
  sessions=(${dir}/tmux_resurrect_*.txt(N))
  sessions=(${sessions##*/})
  sessions=(${sessions#tmux_resurrect_})
  sessions=(${sessions%.txt})

# Filter matching current input 
local matches=()
for s in $sessions; do 
  input=$s pretty_date=$(date -d "${input:0:8}\
    ${input:9:2}:${input:11:2}:${input:13:2}"\
    "+%Y-%m-%d %H:%M:%S")
    matches+=$pretty_date done

  compadd -Q -d matches -- $sessions
}

__tmux_resurrect_complete() {
  local cur dir sessions
  cur=${words[CURRENT]}           # what user typed so far
  dir="$HOME/.tmux/resurrect"
  [[ -d $dir ]] || return

  # collect files -> keep full filenames (just strip path)
  sessions=(${dir}/tmux_resurrect_*.txt(N))
  sessions=(${sessions##*/})  # remove path, keep prefix and suffix

  # Filter matching current input
  local matches=()
  for s in $sessions; do
    if [[ $s == $cur* ]]; then
      matches+=$s
    fi
  done

  compadd -Q -d "Resurrect sessions" -- $matches
}

# Tmux resurrect completion
compdef __tmux_resurrect_complete tmux-resurrect
