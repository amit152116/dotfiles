# Distrobox: auto-enter container for marked repos (run before p10k instant prompt
# so we exec into the container without flashing a host prompt first).
source "$HOME"/.zsh/distrobox.zsh

# Enable Powerlevel10k instant prompt. Should stay close to the top of $HOME/.zshrc.
# Initialization code that may require console input (password prompts, [y/n]
# confirmations, etc.) must go above this block; everything else may go below.
if [[ -r "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh" ]]; then
    source "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh"
fi

# Cache zoxide/direnv hook output — regenerates only when binary changes
_zsh_cache="${XDG_CACHE_HOME:-$HOME/.cache}/zsh"
[[ -d "$_zsh_cache" ]] || mkdir -p "$_zsh_cache"
_zsh_bin=$(command -v zoxide 2>/dev/null)
if [[ -n "$_zsh_bin" ]]; then
    [[ -f "$_zsh_cache/zoxide-init.zsh" && ! "$_zsh_bin" -nt "$_zsh_cache/zoxide-init.zsh" ]] ||
    zoxide init zsh >|"$_zsh_cache/zoxide-init.zsh"
    source "$_zsh_cache/zoxide-init.zsh"
fi
_zsh_bin=$(command -v direnv 2>/dev/null)
if [[ -n "$_zsh_bin" ]]; then
    [[ -f "$_zsh_cache/direnv-hook.zsh" && ! "$_zsh_bin" -nt "$_zsh_cache/direnv-hook.zsh" ]] ||
    direnv hook zsh >|"$_zsh_cache/direnv-hook.zsh"
    source "$_zsh_cache/direnv-hook.zsh"
fi
unset _zsh_cache _zsh_bin

# 0️⃣ Environment exports (PATH, etc.)
source "$HOME"/.zsh/exports.zsh

# 1️⃣ Load Zinit first
source "$HOME"/.zsh/zinit.zsh

# 2️⃣ Load FZF
source "$HOME"/.zsh/fzf.zsh

# 3️⃣ Load completion
source "$HOME"/.zsh/completions.zsh

# 4️⃣ Load functions
source "$HOME"/.zsh/functions.zsh

# Auto-activate project .venv on cd
source "$HOME"/.zsh/venv.zsh

# 5️⃣  Load ROS configs
source "$HOME"/.zsh/ros.zsh

# 6️⃣  Load aliases
source "$HOME"/.zsh/aliases.zsh

source "$HOME"/.zsh/keybindings.zsh

source "$HOME"/.zsh/taskwarrior.zsh

# To customize prompt, run `p10k configure` or edit $HOME/.p10k.zsh.
[[ ! -f $HOME/.p10k.zsh ]] || source "$HOME"/.p10k.zsh
