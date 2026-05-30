#!/usr/bin/env zsh

# Oh My Zsh setup
export ZSH="$HOME/.oh-my-zsh"
ZSH_CUSTOM="$ZSH/custom"

typeset -A custom_plugins
custom_plugins=(
    alias-tips "https://github.com/djui/alias-tips.git"
    zsh-autosuggestions "https://github.com/zsh-users/zsh-autosuggestions.git"
    zsh-completions "https://github.com/zsh-users/zsh-completions.git"
    zsh-history-substring-search "https://github.com/zsh-users/zsh-history-substring-search.git"
    zsh-syntax-highlighting "https://github.com/zsh-users/zsh-syntax-highlighting.git"
    fzf-tab "https://github.com/Aloxaf/fzf-tab.git"
)

for plugin in ${(k)custom_plugins}; do
    if [ ! -d "$ZSH_CUSTOM/plugins/$plugin" ]; then
        echo "Installing $plugin..."
        git clone --depth=1 "${custom_plugins[$plugin]}" "$ZSH_CUSTOM/plugins/$plugin"
    fi
done

# Plugins that always load (pure-shell, no external binary required)
plugins=(
    alias-tips
    zsh-autosuggestions
    zsh-completions
    zsh-history-substring-search
    zsh-syntax-highlighting
    fzf-tab
    git
    gitignore
    aliases
    common-aliases
    web-search
    sudo
    vi-mode
    jsontools
)

# Plugins guarded on an external package — load only if the binary exists.
# Maps plugin name -> required command.
typeset -A guarded_plugins
guarded_plugins=(
    fzf fzf
    tmux tmux
    taskwarrior task
    docker docker
)

for plugin in ${(k)guarded_plugins}; do
    if command -v "${guarded_plugins[$plugin]}" >/dev/null 2>&1; then
        plugins+=("$plugin")
    fi
done

ZSH_TMUX_AUTOSTART_ONCE=true
ZSH_THEME="powerlevel10k/powerlevel10k"

# Hyphen-insensitive completion
HYPHEN_INSENSITIVE="true"

# Display red dots while waiting for completion
COMPLETION_WAITING_DOTS="true"

# Auto-update behavior
zstyle ':omz:update' mode auto
zstyle ':completion:*:*:docker:*' option-stacking yes
zstyle ':completion:*:*:docker-*:*' option-stacking yes
# Source Oh My Zsh
source $ZSH/oh-my-zsh.sh

# fzf-tab: enable on host, disable inside a container.
# Detection: docker (/.dockerenv), podman (/run/.containerenv), or $container var.
if [[ -f /.dockerenv || -f /run/.containerenv || -n "$container" ]]; then
    (( $+functions[disable-fzf-tab] )) && disable-fzf-tab
fi
