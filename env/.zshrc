# auto-enter container for marked repos before p10k instant prompt, so we exec in without flashing a host prompt
source "$HOME"/.zsh/distrobox.zsh

# p10k instant prompt must stay near top: init code needing console input (password/[y/n] prompts) goes above it
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

source "$HOME"/.zsh/exports.zsh
source "$HOME"/.zsh/zinit.zsh

# must follow zinit.zsh — history opts live there
source "$HOME"/.zsh/history.zsh

source "$HOME"/.zsh/fzf.zsh
source "$HOME"/.zsh/completions.zsh
source "$HOME"/.zsh/functions.zsh
source "$HOME"/.zsh/venv.zsh
source "$HOME"/.zsh/ros.zsh
source "$HOME"/.zsh/aliases.zsh
source "$HOME"/.zsh/keybindings.zsh

source "$HOME"/.zsh/taskwarrior.zsh

# To customize prompt, run `p10k configure` or edit $HOME/.p10k.zsh.
[[ ! -f $HOME/.p10k.zsh ]] || source "$HOME"/.p10k.zsh
