alias cls="clear"
alias python="python3"
# alias pip="pip3"
alias dotfiles='cd "$(git -C "$(realpath ~/.zshrc | xargs dirname)" rev-parse --show-toplevel)"'

alias cat='batcat --style=plain'
alias pop='popd'

alias home='builtin cd ~'
if command -v eza &>/dev/null; then
	alias ls="eza -g --icons"
	alias la="eza -gla --icons"
	alias ll="eza -gl --icons"
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
	alias search="apt search"
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

# ============================================================================
# CMake Build System Helpers
# ============================================================================

TEMPLATES_DIR="$HOME/.dotfiles/templates"

# Copy CMakeLists templates
alias cmk-exe='cp $TEMPLATES_DIR/CMakeLists-executable.txt ./CMakeLists.txt'
alias cmk-lib='cp $TEMPLATES_DIR/CMakeLists-library.txt ./CMakeLists.txt'
alias cmk-header='cp $TEMPLATES_DIR/CMakeLists-header-only.txt ./CMakeLists.txt'
alias cmk-test='cp $TEMPLATES_DIR/CMakeLists-tests.txt ./tests/CMakeLists.txt'
alias cmk-clangd='cp $TEMPLATES_DIR/clangd ./.clangd && cp $TEMPLATES_DIR/clang-format ./.clang-format'

# Makefile aliases
alias mknew='cp $TEMPLATES_DIR/makefile ./Makefile'
