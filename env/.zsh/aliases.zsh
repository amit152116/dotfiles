alias cls="clear"
alias python="python3"
unalias t 2>/dev/null # Remove OMZ alias, use custom function from functions.zsh
# alias pip="pip3"
alias dotfiles='cd "$(git -C "$(realpath ~/.zshrc | xargs dirname)" rev-parse --show-toplevel)"'

alias cat='batcat --style=plain'
alias pop='popd'
alias vim='nvim'
alias home="builtin cd ~"
alias dbox="distrobox"

if command -v eza &>/dev/null; then
    unalias l lr lrt lsr lt 2>/dev/null
    alias l="eza -lh --icons"
    alias lr="eza -tRh --icons"
    alias lrt="eza -lFrh --icons"
    alias lsr="eza -lARh --icons"
    alias lt="eza -lh --icons --sort=modified"
    alias la="eza -lah --icons"
    alias lD='eza -glD'
    alias lDD='eza -glDa'
    alias lS='eza -gl -ssize'
    alias lT='eza -gl -snewest'
    alias la='eza -gla --icons'
    alias ldot='eza -gld .*'
    alias ll='eza -gl --icons'
    alias ls='eza -g --icons'
    alias lsa='ls -lah'
    alias lsd='eza -gd'
    alias lsdl='eza -gdl'
else
    # ls, the common ones I use a lot shortened for rapid fire usage
    alias l='ls -lFh'   #size,show type,human readable
    alias la='ls -lAFh' #long list,show almost all,show type,human readable
    alias lr='ls -tRFh' #sorted by date,recursive,show type,human readable
    alias lt='ls -ltFh' #long list,sorted by date,show type,human readable
    alias ll='ls -l'    #long list
    alias ldot='ls -ld .*'
    alias lS='ls -1FSsh'
    alias lart='ls -1Fcart'
    alias lrt='ls -1Fcrt'
    alias lsr='ls -lARFh' #Recursive list of files and directories
    alias lsn='ls -1'     #A column contains name of files and directories
fi

if command -v docker &>/dev/null; then
    alias docker-clean="docker container prune -f && docker image prune -f "
fi
# System aliases based on OS detection
if [[ -f /etc/debian_version ]]; then
    # Debian/Ubuntu aliases
    alias upgrade="sudo apt update && sudo apt upgrade -y"
    alias install="sudo apt install"
    alias remove="sudo apt purge"
    alias clean="sudo apt autoremove -y && sudo apt clean"
    alias update="sudo apt update"
fi

# JSON pretty print (check if appropriate tools are available)
if command -v jq &>/dev/null; then
    alias pjson='jq .'
elif command -v python3 &>/dev/null; then
    alias pjson='python3 -m json.tool'
elif command -v pp_json &>/dev/null; then
    alias pjson='pp_json'
fi

# cmake build system helpers
TEMPLATES_DIR="$HOME/.dotfiles/templates"

# Copy CMakeLists templates
alias cmk-exe='cp $TEMPLATES_DIR/CMakeLists-executable.txt ./CMakeLists.txt'
alias cmk-lib='cp $TEMPLATES_DIR/CMakeLists-library.txt ./CMakeLists.txt'
alias cmk-header='cp $TEMPLATES_DIR/CMakeLists-header-only.txt ./CMakeLists.txt'
alias cmk-test='cp $TEMPLATES_DIR/CMakeLists-tests.txt ./tests/CMakeLists.txt'
alias cmk-clangd='cp $TEMPLATES_DIR/clangd ./.clangd && cp $TEMPLATES_DIR/clang-format ./.clang-format'

# Makefile aliases
alias mknew='cp $TEMPLATES_DIR/makefile ./Makefile'

# GitHub aliases (requires gh CLI)
alias gob='gh browse'
alias gobs='gh browse --settings'
alias gobr='gh browse --releases'
alias gobp='gh browse --projects'
alias gobw='gh browse --wiki'
