# Source fzf configuration
[ -f ~/.fzf.zsh ] && source ~/.fzf.zsh

# FZF Default Options
export FZF_DEFAULT_OPTS='
--height=50%                        
--layout=reverse                    
--border                            
--preview="batcat --style=numbers --color=always --line-range :100 {}"  
--preview-window="right:50%"        
--info=inline                       
--prompt="Search > "                
--pointer="▶"                       
--marker="✓"                        
--color=bg+:#282c34,bg:#21252b,spinner:#61afef,hl:#e06c75
--color=fg:#abb2bf,header:#56b6c2,info:#c678dd,pointer:#e5c07b
--color=marker:#98c379,fg+:#abb2bf,prompt:#61afef,hl+:#e06c75
'

export FZF_TMUX_OPTS=" -p70%,70% "


