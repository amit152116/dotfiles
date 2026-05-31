#!/usr/bin/env zsh

# --- Zinit bootstrap ---
ZINIT_HOME="${XDG_DATA_HOME:-$HOME/.local/share}/zinit/zinit.git"
[[ -d "$ZINIT_HOME/.git" ]] || git clone --depth=1 https://github.com/zdharma-continuum/zinit.git "$ZINIT_HOME"
source "$ZINIT_HOME/zinit.zsh"
autoload -Uz _zinit
((${+_comps})) && _comps[zinit]=_zinit

# --- History ---
HISTFILE="$HOME/.zsh_history"
HISTSIZE=50000
SAVEHIST=10000
setopt HIST_EXPIRE_DUPS_FIRST HIST_IGNORE_DUPS HIST_IGNORE_SPACE HIST_VERIFY
setopt SHARE_HISTORY EXTENDED_HISTORY
setopt AUTO_CD INTERACTIVE_COMMENTS
unsetopt BEEP

# --- Completion options ---
zstyle ':completion:*' menu select
zstyle ':completion:*' matcher-list 'm:{a-zA-Z-_}={A-Za-z_-}'
zstyle ':completion:*:*:docker:*' option-stacking yes
zstyle ':completion:*:*:docker-*:*' option-stacking yes
export ZSH_CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/zsh"
[[ -d "$ZSH_CACHE_DIR/completions" ]] || mkdir -p "$ZSH_CACHE_DIR/completions"

# --- Powerlevel10k (immediate — must load before prompt) ---
zinit ice depth"1"
zinit light romkatv/powerlevel10k

# --- OMZ lib files (immediate) ---
zinit snippet OMZ::lib/git.zsh
zinit snippet OMZ::lib/key-bindings.zsh
zinit snippet OMZ::lib/functions.zsh
zinit snippet OMZ::lib/directories.zsh
zinit snippet OMZ::lib/misc.zsh
zinit snippet OMZ::lib/termsupport.zsh

# --- OMZ built-in plugins (immediate) ---
zinit snippet OMZP::git
zinit snippet OMZP::sudo
zinit snippet OMZP::gitignore
zinit snippet OMZP::common-aliases
zinit snippet OMZP::web-search
zinit snippet OMZP::jsontools

# --- zsh-completions: blockf prevents fpath duplication ---
zinit ice blockf
zinit light zsh-users/zsh-completions

# --- vi mode ---
# Disable lazy keybindings: without this, vicmd bindings are deferred until
# first ESC press, which wipes custom rebinds set in ZVM_AFTER_INIT_COMMANDS.
ZVM_LAZY_KEYBINDINGS=false
# NEX engine does its own ESC parsing, ignoring ZLE bindkey table — Alt+key breaks.
# ZLE engine uses standard bindkey matching so \eq, \eg etc. work correctly.
ZVM_READKEY_ENGINE='zle'
zinit ice depth"1"
zinit light jeffreytse/zsh-vi-mode

# Correct lowercase variable names the plugin actually reads.
# keybindings.zsh appends _zvm_rebind_custom_keys to both.
zvm_after_init_commands=(
    '(( $+functions[fzf-history-widget] )) && bindkey "^R" fzf-history-widget'
    '(( $+functions[fzf-file-widget] ))    && bindkey "^T" fzf-file-widget'
    '(( $+functions[fzf-cd-widget] ))      && bindkey "^[c" fzf-cd-widget'
)
zvm_after_lazy_keybindings_commands=()

# --- Guarded plugins (immediate) ---
if command -v tmux &>/dev/null; then
    ZSH_TMUX_AUTOSTART_ONCE=true
    zinit snippet OMZP::tmux
fi
command -v task &>/dev/null && zinit snippet OMZP::taskwarrior
command -v docker &>/dev/null && zinit snippet OMZP::docker
command -v podman &>/dev/null && zinit snippet OMZP::podman
if command -v ssh &>/dev/null; then
    zstyle :omz:plugins:ssh-agent quiet yes
    zstyle :omz:plugins:ssh-agent lazy yes
    zinit snippet OMZP::ssh-agent
fi

# ↑ completions.zsh runs after this: compinit → zinit cdreplay -q ↑

# --- Turbo plugins (deferred post-prompt, fired after compinit) ---
zinit wait lucid for \
    atload"_zsh_autosuggest_start" \
    zsh-users/zsh-autosuggestions \
    zsh-users/zsh-history-substring-search \
    djui/alias-tips \
    atload'[[ -f /.dockerenv || -f /run/.containerenv || -n "$container" ]] && (( $+functions[disable-fzf-tab] )) && disable-fzf-tab' \
    Aloxaf/fzf-tab \
    atload"autopair-init" hlissner/zsh-autopair \
    OMZP::colored-man-pages

# fast-syntax-highlighting must be last
zinit ice wait lucid atinit"zicompdef"
zinit light zdharma-continuum/fast-syntax-highlighting
