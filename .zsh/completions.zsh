
autoload -U +X bashcompinit && bashcompinit
autoload -U +X compinit && compinit

# Alias completion
for alias_name in $(alias | awk -F= '{print $1}' | awk '{print $2}'); do
    compdef "$alias_name=$(alias "$alias_name" | sed "s/^.*='\([^']*\)'.*/\1/")"
done

# Autocomplete for tmux_resurrect
_tmux_resurrect_complete() {
  local cur dir sessions
  cur=${words[CURRENT]}           # what user typed so far
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

  # feed to compadd (Zsh handles filtering with $cur automatically)
  compadd -Q -d matches -- $sessions
}

# Tmux resurrect completion
compdef _tmux_resurrect_complete tmux_resurrect
