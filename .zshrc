# Enable Powerlevel10k instant prompt. Should stay close to the top of ~/.zshrc.
# Initialization code that may require console input (password prompts, [y/n]
# confirmations, etc.) must go above this block; everything else may go below.
if [[ -r "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh" ]]; then
  source "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh"
fi


# 0️⃣ Environment exports (PATH, etc.)
source ~/.zsh/exports.zsh

# 1️⃣ Load Oh My Zsh first
source ~/.zsh/omz.zsh

# 2️⃣ Load FZF
source ~/.zsh/fzf.zsh

# 3️⃣ Load completion
source ~/.zsh/completions.zsh

# 4️⃣ Load functions
source ~/.zsh/functions.zsh

# 5️⃣ Load aliases
source ~/.zsh/aliases.zsh

# 6️⃣ Load ROS configs
source ~/.zsh/ros.zsh

source ~/.zsh/keybindings.zsh


eval "$(zoxide init --cmd cd zsh)"

# To customize prompt, run `p10k configure` or edit ~/.p10k.zsh.
[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh
