# Enable Powerlevel10k instant prompt. Should stay close to the top of $HOME/.zshrc.
# Initialization code that may require console input (password prompts, [y/n]
# confirmations, etc.) must go above this block; everything else may go below.
if [[ -r "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh" ]]; then
    source "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh"
fi

if  command -v zoxide >/dev/null 2>&1; then
    eval "$(zoxide init zsh)"
fi

# 0️⃣ Environment exports (PATH, etc.)
source "$HOME"/.zsh/exports.zsh

# 1️⃣ Load Oh My Zsh first
source "$HOME"/.zsh/omz.zsh

# 2️⃣ Load FZF
source "$HOME"/.zsh/fzf.zsh

# 3️⃣ Load completion
source "$HOME"/.zsh/completions.zsh
source "$HOME"/.zsh/netclient.zsh


# 4️⃣ Load functions
source "$HOME"/.zsh/functions.zsh

# 5️⃣  Load ROS configs
source "$HOME"/.zsh/ros.zsh

# 6️⃣  Load aliases
source "$HOME"/.zsh/aliases.zsh

source "$HOME"/.zsh/keybindings.zsh

source "$HOME"/.zsh/taskwarrior.zsh

# To customize prompt, run `p10k configure` or edit $HOME/.p10k.zsh.
[[ ! -f $HOME/.p10k.zsh ]] || source "$HOME"/.p10k.zsh


if command -v direnv >/dev/null 2>&1; then
    eval "$(direnv hook zsh)"
fi

