#############################################################################
#                            ZSH CONFIGURATION                              #
#############################################################################


#=============================================================================
# SECTION: OH-MY-ZSH CONFIGURATION
#=============================================================================

# Oh My Zsh setup
export ZSH="$HOME/.oh-my-zsh"
ZSH_TMUX_AUTOSTART_ONCE=true
ZSH_THEME="powerlevel10k/powerlevel10k"

# Hyphen-insensitive completion
HYPHEN_INSENSITIVE="true"

# Enable command auto-correction
ENABLE_CORRECTION="true"

# Display red dots while waiting for completion
COMPLETION_WAITING_DOTS="true"

# Auto-update behavior
zstyle ':omz:update' mode auto

#=============================================================================
# SECTION: Source Modular 
#=============================================================================
# Source all configs from ~/.zsh
for config in ~/.zsh/*.zsh; do
  source $config
done
