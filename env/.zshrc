# Enable Powerlevel10k instant prompt. Should stay close to the top of ~/.zshrc.
# Initialization code that may require console input (password prompts, [y/n]
# confirmations, etc.) must go above this block; everything else may go below.
if [[ -r "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh" ]]; then
    source "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh"
fi

if  command -v zoxide >/dev/null 2>&1; then
    eval "$(zoxide init zsh)"
fi

# 0️⃣ Environment exports (PATH, etc.)
source ~/.zsh/exports.zsh

# 1️⃣ Load Oh My Zsh first
source ~/.zsh/omz.zsh

# 2️⃣ Load FZF
source ~/.zsh/fzf.zsh

# 3️⃣ Load completion
source ~/.zsh/completions.zsh
source ~/.zsh/netclient.zsh


# 4️⃣ Load functions
source ~/.zsh/functions.zsh

# 5️⃣  Load ROS configs
source ~/.zsh/ros.zsh

# 6️⃣  Load aliases
source ~/.zsh/aliases.zsh

source ~/.zsh/keybindings.zsh

# To customize prompt, run `p10k configure` or edit ~/.p10k.zsh.
[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh


# Source env.sh or .env if present in current directory
_source_local_env() {
    if [[ -f "$PWD/env.sh" ]]; then
        source "$PWD/env.sh"
    fi
    if [[ -f "$PWD/.env" ]]; then
        source "$PWD/.env"
    fi
}

autoload -U add-zsh-hook
add-zsh-hook chpwd _source_local_env
_source_local_env
