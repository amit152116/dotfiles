
alias cls="clear"
alias python="python3"
# alias pip="pip3"
alias dotfiles='cd "$(dirname "$(realpath ~/.zshrc)")"'

alias cd='z'
alias home='builtin cd ~'

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
