# Enable Powerlevel10k instant prompt. Should stay close to the top of ~/.zshrc.
# Initialization code requiring console input (password prompts, [y/n] confirmations) must go above this block.
if [[ -r "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh" ]]; then
  source "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh"
fi

# Source fzf configuration
[ -f ~/.fzf.zsh ] && source ~/.fzf.zsh

# Paths
export PATH="$HOME/.local/bin:/usr/local/bin:$HOME/.cargo/bin:/usr/local/nodejs/bin:$HOME/go/bin:$(go env GOPATH)/bin:$PATH"

# Aliases Auto-Completion 
autoload -U +X bashcompinit && bashcompinit
autoload -U +X compinit && compinit

for alias_name in $(alias | awk -F= '{print $1}' | awk '{print $2}'); do
    compdef "$alias_name=$(alias "$alias_name" | sed "s/^.*='\([^']*\)'.*/\1/")"
done

# Oh My Zsh setup
export ZSH="$HOME/.oh-my-zsh"
ZSH_THEME="powerlevel10k/powerlevel10k"
POWERLEVEL9K_MODE="nerdfont-complete"

# Android SDK settings
export ANDROID_HOME="$HOME/Android/Sdk"
export PATH="$PATH:$ANDROID_HOME/emulator"
alias debugPath="app/build/outputs/apk/debug/app-debug.apk"
alias releasePath="app/build/outputs/apk/release/app-release.apk"

# FZF Default Options
export FZF_DEFAULT_OPTS='
--height=50%                        
--layout=reverse                    
--border                            
--preview="batcat --style=numbers --color=always --line-range :100 {}"  
--preview-window="right:50%"        
--info=inline                       
--prompt="Search > "                
--pointer="▶"                       
--marker="✓"                        
--color=bg+:#282c34,bg:#21252b,spinner:#61afef,hl:#e06c75
--color=fg:#abb2bf,header:#56b6c2,info:#c678dd,pointer:#e5c07b
--color=marker:#98c379,fg+:#abb2bf,prompt:#61afef,hl+:#e06c75
'

export FZF_TMUX_OPTS=" -p70%,70% "

# Zsh Plugins (Load Syntax Highlighting Last)
plugins=(
	zsh-autosuggestions
	zsh-completions
	fzf
	z
	zsh-history-substring-search
	alias-tips
	tmux
	aliases
	common-aliases
	web-search
	zsh-syntax-highlighting
	docker 
	sudo 
	jsontools
)

source $ZSH/oh-my-zsh.sh

alias pjson='pp_json'
alias pysource='source ./myenv/bin/activate'
# User Configuration


# ROS2 settings
export ROS_DOMAIN_ID=0


source /opt/ros/humble/setup.zsh 
source ~/Documents/aim_ros2/install/setup.zsh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
source /opt/ros/humble/share/ros2cli/environment/ros2-argcomplete.zsh

alias ros='ros2'
alias rostopic='ros2 topic'
alias rossrv='ros2 service'
alias rqt_graph='ros2 run rqt_graph rqt_graph'
alias rqt_plot='ros2 run rqt_plot rqt_plot'
alias rqt_console='ros2 run rqt_console rqt_console'
alias rqt_gui='ros2 run rqt_gui rqt_gui'
alias roslaunch='ros2 launch'

rospkg() {
    if [[ $# -lt 2 ]]; then
        echo "Usage: rospkg <py|cpp> <package_name> [dependencies...]"
        return 1
    fi

    local lang=$1
    local pkg_name=$2
    shift 2
    local dependencies="$*"

    case "$lang" in
        py)
            ros2 pkg create "$pkg_name" --build-type ament_python --dependencies "rclpy $dependencies" --license GPL-3.0-only
            ;;
        cpp)
            ros2 pkg create "$pkg_name" --build-type ament_cmake --dependencies "rclcpp $dependencies" --license GPL-3.0-only
            ;;
        *)
            echo "Invalid language. Use 'py' for Python or 'cpp' for C++."
            return 1
            ;;
    esac
}

# Fuzzy search and run a node
rosrun() {
    package=$(ros2 pkg list | fzf --prompt="Select a package: ")
    [ -z "$package" ] && echo "No package selected. Exiting..." && return 1

    node=$(ros2 pkg executables "$package" | awk '{print $2}' | fzf --prompt="Select a node: ")
    [ -z "$node" ] && echo "No node selected. Exiting..." && return 1

    echo "Running: ros2 run $package $node"
    ros2 run "$package" "$node"
}


# List and run available launch files

nodes() {
    BUILT_PACKAGES=$(colcon list --names-only)
    if [ -z "$BUILT_PACKAGES" ]; then
        echo "No packages found. Did you run colcon build?"
        return 1
    else
        echo "$BUILT_PACKAGES" | while IFS= read -r pkg; do
            EXECUTABLES=$(ros2 pkg executables "$pkg")
            echo "$EXECUTABLES"  # Remove the extra \n here
            echo ""
      done
    fi
}


# Display ROS logs in real time with filtering
roslog() {
    journalctl -u ros2 -f --no-tail | fzf --prompt="Filter logs: "
}


# Open a shell inside ROS 2 environment
rossh() {
    echo "Entering ROS shell..."
    env -i bash --rcfile <(echo "source /opt/ros/humble/setup.bash; exec bash")
}


# Reload Zsh and Tmux configuration 
reload() {
  source ~/.zshrc && tmux source-file ~/.tmux.conf && echo "Reloaded ~/.zshrc & ~/.tmux.conf"
}


# Fuzzy search and run a launch file
fuzzy() {
	selection=$(eval "$* | fzf")
}


# GIT Aliases
alias status='git status'
alias logs='git log --oneline'
alias amend='git commit --amend --no-edit'

commit() {
  if [ -z "$1" ]; then
    echo "Usage: commit \"commit message\""
    return 1
  fi
  git commit -m "$1"
}

pull() {
  git pull "$@"
}

push() {
  git push "$@"
}

branch(){
	git branch "$@"
}

fetch(){
	git fetch "$@"
}

reset(){
	git reset "$@"
}

rebase() {
  git rebase "$@"
}

merge(){
	git merge "$@"
}

stash(){
	git stash "$@"
}

pick(){
	git cherry-pick "$@"
}

checkout() {
  # Check if a command was provided
  if [[ $# -eq 0 ]]; then
  	# Show git branches and let the user select one to checkout
		local branch
		branch=$(git branch --color=always | fzf --ansi | sed 's/^[ *]*//')
		if [[ -n "$branch" ]]; then
			git checkout "$branch"
		else
			echo "No branch selected."
		fi
		return 0
  fi

  # Run the command and pipe its output to fzf
  local selection
  selection=$(eval "$*" | fzf --ansi)

  # If a selection is made, perform git checkout
  if [[ -n "$selection" ]]; then
    # Trim whitespace and checkout the selected branch
    git checkout "$(echo "$selection" | xargs)"
  else
    echo "No selection made or invalid input."
    return 1
  fi
}

 
# Uncomment the following line to use case-sensitive completion.
# CASE_SENSITIVE="true"

# Uncomment the following line to use hyphen-insensitive completion.
HYPHEN_INSENSITIVE="true"

# Uncomment one of the following lines to change the auto-update behavior.
zstyle ':omz:update' mode auto      # Auto-update without asking.
# zstyle ':omz:update' frequency 13  # Update frequency in days.

# Uncomment if pasting URLs or text is broken.
# DISABLE_MAGIC_FUNCTIONS="true"

# Uncomment to disable auto-setting terminal title.
# DISABLE_AUTO_TITLE="true"

# Uncomment to enable command auto-correction.
ENABLE_CORRECTION="true"

# Uncomment to display red dots while waiting for completion.
COMPLETION_WAITING_DOTS="true"

# Uncomment to disable marking untracked files as dirty in VCS.
# DISABLE_UNTRACKED_FILES_DIRTY="true"

# Uncomment to change the command execution timestamp format in history.
# HIST_STAMPS="yyyy-mm-dd"

# Prompt customization
[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh
export GAZEBO_MODEL_PATH=~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH
