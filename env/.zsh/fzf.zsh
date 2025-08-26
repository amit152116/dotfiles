

if ! command -v fzf &>/dev/null && [ ! -d "$HOME/.fzf" ]; then
    echo "Installing fzf..."
    git clone --depth 1 https://github.com/junegunn/fzf.git "$HOME/.fzf"
    "$HOME/.fzf/install" --all
fi


export FZF_DEFAULT_COMMAND="fdfind --type f --strip-cwd-prefix --hidden --follow --exclude .git"
export FZF_COMMON_OPTS='
--prompt="❯ "                
--pointer="➤ "                       
--marker="✓ "                        
--color=marker:#98c379,fg+:#abb2bf,prompt:#61afef,hl+:#e06c75
'

# FZF Default Options
export FZF_DEFAULT_OPTS="
$FZF_COMMON_OPTS
--height=50%                        
--layout=reverse                    
--border=rounded                           
--style=minimal
--no-preview
--info=inline                       
--color=fg:#e8e6e9,bg:#111111,fg+:#e8e6e9,bg+:#484867,spinner:#6d6dc9,header:#6ab6bd,info:#c6a642,pointer:#e1a51c,marker:#48a842,hl:#d61d52,hl+:#e15877,prompt:#5556d3
"


export FZF_ALT_C_OPTS="
--preview='tree -C -L 2 {}'
--preview-window='right:50%'        
"

export FZF_CTRL_T_OPTS="
--preview-window='right:50%'        
--preview='
  if [ -f {} ]; then
    batcat --style=numbers --color=always --line-range :100 {}
  elif [ -d {} ]; then
    tree -C -L 2 {}
  fi
'
"

export FZF_CTRL_R_OPTS="
  --bind 'ctrl-y:execute-silent(echo -n {2..} | pbcopy)+abort'
  --color header:italic
  --header 'Press CTRL-Y to copy command into clipboard'"

