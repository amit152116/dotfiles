#############################################################################
#                            ZSH CONFIGURATION                              #
#############################################################################

#=============================================================================
# SECTION: POWERLEVEL10K INSTANT PROMPT
#=============================================================================

# Enable Powerlevel10k instant prompt. Should stay close to the top of ~/.zshrc.
# Initialization code requiring console input must go above this block.
if [[ -r "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh" ]]; then
  source "${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-${(%):-%n}.zsh"
fi

#=============================================================================
# SECTION: PATH AND ENVIRONMENT VARIABLES
#=============================================================================

# Paths
export PATH="$HOME/.local/bin:/usr/local/bin:/usr/lib/ccache:$PATH"

export DOCKER_HOST=unix:///var/run/docker.sock

# Add Cargo if installed
if [[ -d "$HOME/.cargo/bin" ]]; then
  export PATH="$HOME/.cargo/bin:$PATH"
fi

# Cuda is installed
if [[ -d "/usr/local/cuda" ]]; then
  export PATH="/usr/local/cuda/bin:$PATH"
  export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"
fi
# Add Micro-XRCE-DDS-Gen if installed
# if [[ -d "$HOME/ardupilot/Micro-XRCE-DDS-Gen" ]]; then
#   export PATH=$PATH:$HOME/ardupilot/Micro-XRCE-DDS-Gen/scripts
# fi

# Add Ardupilot-Autopilot if installed
if [[ -d "$HOME/ardupilot" ]]; then
  export PATH=$PATH:$HOME/ardupilot/Tools/autotest
  source $HOME/ardupilot/Tools/completion/completion.zsh
fi

# if [[ -d "$HOME/ardupilot/src/ardupilot_gazebo" ]]; then
#   export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
#   export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot/src/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
#   export GZ_FUEL_CACHE_ONLY=1
#   export GZ_FUEL_DOWNLOAD_MODE=none
#   source /usr/local/share/ardupilot_gazebo/local_setup.zsh
#   source $HOME/ardupilot/install/setup.zsh
#
# fi

# Add Node.js if installed
if [[ -d "/usr/local/nodejs/bin" ]]; then
  export PATH="/usr/local/nodejs/bin:$PATH"
fi

# Add Go if installed
if command -v go &>/dev/null; then
  export PATH="$(go env GOPATH)/bin:$PATH"
fi

# Set editor preferences (fallback to vim if nvim not available)
if command -v nvim &>/dev/null; then
  export EDITOR="nvim"
  export VISUAL="nvim"
else
  export EDITOR="vim"
  export VISUAL="vim"
fi

# Hyphen-insensitive completion
HYPHEN_INSENSITIVE="true"

# Enable command auto-correction
ENABLE_CORRECTION="true"

# Display red dots while waiting for completion
COMPLETION_WAITING_DOTS="true"

#=============================================================================
# SECTION: OH-MY-ZSH CONFIGURATION
#=============================================================================

# Oh My Zsh setup
export ZSH="$HOME/.oh-my-zsh"
ZSH_TMUX_AUTOSTART_ONCE=true
ZSH_THEME="powerlevel10k/powerlevel10k"
POWERLEVEL9K_MODE="nerdfont-complete"

# Auto-update behavior
zstyle ':omz:update' mode auto

# Define plugins
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
	docker 
	sudo 
	jsontools
	# Load syntax highlighting last
	zsh-syntax-highlighting
)

#=============================================================================
# SECTION: COMPLETION SETTINGS
#=============================================================================

# Aliases Auto-Completion 
autoload -U +X bashcompinit && bashcompinit
autoload -U +X compinit && compinit

for alias_name in $(alias | awk -F= '{print $1}' | awk '{print $2}'); do
    compdef "$alias_name=$(alias "$alias_name" | sed "s/^.*='\([^']*\)'.*/\1/")"
done

#=============================================================================
# SECTION: FZF CONFIGURATION
#=============================================================================

# Source fzf configuration
[ -f ~/.fzf.zsh ] && source ~/.fzf.zsh

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

#=============================================================================
# SECTION: GENERAL ALIASES
#=============================================================================

# System aliases based on OS detection
if [[ -f /etc/debian_version ]]; then
  # Debian/Ubuntu aliases
  alias upgrade="sudo apt update && sudo apt upgrade -y"
  alias install="sudo apt install"
  alias remove="sudo apt purge"
  alias clean="sudo apt autoremove -y && sudo apt clean"
  alias search="apt search"
  alias update="sudo apt update"
elif [[ -f /etc/arch-release ]]; then
  # Arch Linux aliases
  alias upgrade="sudo pacman -Syu"
  alias install="sudo pacman -S"
  alias remove="sudo pacman -R"
  alias search="pacman -Ss"
  alias update="sudo pacman -Sy"
elif [[ -f /etc/fedora-release || -f /etc/redhat-release ]]; then
  # Fedora/RHEL/CentOS aliases
  alias upgrade="sudo dnf upgrade -y"
  alias install="sudo dnf install"
  alias remove="sudo dnf remove"
  alias search="dnf search"
  alias update="sudo dnf check-update"
elif [[ "$(uname)" == "Darwin" ]]; then
  # macOS with Homebrew aliases
  if command -v brew &>/dev/null; then
    alias upgrade="brew update && brew upgrade && brew cleanup"
    alias install="brew install"
    alias remove="brew uninstall"
    alias search="brew search"
    alias update="brew update"
  fi
fi

alias cls="clear"
alias python="python3"
# alias pip="pip3"
alias dotfiles='cd "$(dirname "$(realpath ~/.zshrc)")"'

# Utility aliases
alias pysource='source ./myenv/bin/activate'

# JSON pretty print (check if appropriate tools are available)
if command -v jq &>/dev/null; then
  alias pjson='jq .'
elif command -v python3 &>/dev/null; then
  alias pjson='python3 -m json.tool'
elif command -v pp_json &>/dev/null; then
  alias pjson='pp_json'
fi


#=============================================================================
# SECTION: GENERAL FUNCTIONS
#=============================================================================

# Configuration reload
reload() {
  source  ~/.zshrc
  
  # Reload tmux configuration if available
  if command -v tmux &>/dev/null && [[ -f ~/.tmux.conf ]]; then
    # Use command substitution to prevent interference with fzf
    command tmux source-file ~/.tmux.conf 2>/dev/null
    echo "Reloaded ~/.zshrc & ~/.tmux.conf"
  else
    echo "Reloaded ~/.zshrc"
  fi
}

# Function to link the latest non-empty tmux session file
link_latest_tmux_session() {
  local dir="$HOME/.tmux/resurrect"
  local files=($(find "$dir" -type f -name 'tmux_resurrect_*.txt' -size +0c | sort -r))

  if (( ${#files[@]} == 0 )); then
    echo "No non-empty tmux_resurrect files found."
    return 1
  fi

  local target=""
  for file in $files; do
    echo "$file\n"
    if [[ -s "$file" ]]; then
      target="$file"
      break
    fi
  done

  if [[ -z "$target" ]]; then
    echo "No non-empty tmux_resurrect files found."
    return 1
  fi

  ln -sf "$target" "$dir/last"
  echo "Symlink 'last' now points to: ${target##*/}"
}

# Run the command and pipe its output to fzf
local selection
if [[ -n "$*" && -t 0 ]]; then
    selection=$(eval "$*" | fzf --ansi)
fi

#=============================================================================
# SECTION: GIT ALIASES AND FUNCTIONS
#=============================================================================

# Function to go to the root directory of the current Git repository
home() {
    local git_root
    git_root=$(git rev-parse --show-toplevel 2>/dev/null)
    
    if [[ -n "$git_root" ]]; then
        cd "$git_root" || echo "Failed to navigate to Git root"
    else
        echo "Not inside a Git repository."
    fi
}

# SECTION: ANDROID SDK SETTINGS
#=============================================================================

# Check if Android SDK exists before loading Android configurations
if [[ -d "$HOME/Android/Sdk" ]]; then
  # Android SDK settings
  export ANDROID_HOME="$HOME/Android/Sdk"
  export PATH="$PATH:$ANDROID_HOME/emulator"
  alias debugPath="app/build/outputs/apk/debug/app-debug.apk"
  alias releasePath="app/build/outputs/apk/release/app-release.apk"
fi

#=============================================================================
# SECTION: ROS2 CONFIGURATION
#=============================================================================

# Check for ROS2 installations in common locations
local ROS_WS="$HOME/Documents/aim_ros2/"

# Check Humble (Ubuntu 22.04)
if [[ -f "/opt/ros/humble/setup.zsh" ]]; then
  ROS_DISTRO="humble"
# Check Iron (Ubuntu 22.04)
elif [[ -f "/opt/ros/iron/setup.zsh" ]]; then
  ROS_DISTRO="iron"
# Check Foxy (Ubuntu 20.04)
elif [[ -f "/opt/ros/foxy/setup.zsh" ]]; then
  ROS_DISTRO="foxy"
# Check Galactic (Ubuntu 20.04)
elif [[ -f "/opt/ros/galactic/setup.zsh" ]]; then
  ROS_DISTRO="galactic"
# User-defined ROS location
elif [[ -n "$ROS_INSTALL_PATH" && -f "$ROS_INSTALL_PATH/setup.zsh" ]]; then
  ROS_DISTRO=$(basename "$ROS_INSTALL_PATH")
fi

# Only load ROS configuration if found
if ! [[ -z "$ROS_DISTRO" ]]; then

  # ROS settings
  export GZ_VERSION=harmonic
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_LOCALHOST_ONLY=0
  # export CYCLONEDDS_URI=file:///home/amit_152116/Documents/aim_ros2/cyclonedds.xml

  # Source ROS2 environment
  source /opt/ros/humble/setup.zsh 
  
  # Source workspace if it exists
  if [[ -d "$ROS_WS" ]]; then
    source "$ROS_WS"install/setup.zsh
  fi
  
  # Source completion scripts if they exist
  if [[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh ]]; then
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
  fi
  
  if [[ -f /opt/ros/humble/share/ros2cli/environment/ros2-argcomplete.zsh ]]; then
    source /opt/ros/humble/share/ros2cli/environment/ros2-argcomplete.zsh
  fi

  # Set GAZEBO_MODEL_PATH if required
  if [[ -d $HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models ]]; then
    export GAZEBO_MODEL_PATH=$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH
  fi


  # ROS2 aliases
  alias ros='ros2'
  alias rostopic='ros2 topic'
  alias rossrv='ros2 service'
  alias rqt_graph='ros2 run rqt_graph rqt_graph'
  alias rqt_plot='ros2 run rqt_plot rqt_plot'
  alias rqt_console='ros2 run rqt_console rqt_console'
  alias rqt_gui='ros2 run rqt_gui rqt_gui'
  alias roslaunch='ros2 launch'
  alias ros_ws="cd $ROS_WS"

  # ROS2 functions
  rospkg() {
    if [[ $# -lt 2 ]]; then
        echo "Usage: rospkg <py|cpp> <package_name> [dependencies...]"
        return 1
    fi

    # Locate the ROS 2 workspace (assumes running inside a workspace)
    local ws_root="$(pwd)"
    while [[ "$ws_root" != "/" && ! -d "$ws_root/src" ]]; do
        ws_root="$(dirname "$ws_root")"
    done

    # Check if src folder exists in the detected workspace
    if [[ ! -d "$ws_root/src" ]]; then
        echo "Error: Could not find a ROS 2 workspace (src/ folder missing)."
        return 1
    fi

    local lang=$1
    local pkg_name=$2
    shift 2
    local dependencies="$*"

    # Move to src directory
    cd "$ws_root/src" || return 1

    case "$lang" in
        py)
            ros2 pkg create "$pkg_name" --build-type ament_python --dependencies rclpy "$dependencies" --license GPL-3.0-only
            ;;
        cpp)
            ros2 pkg create "$pkg_name" --build-type ament_cmake --dependencies rclcpp "$dependencies" --license GPL-3.0-only
            ;;
        *)
            echo "Invalid language. Use 'py' for Python or 'cpp' for C++."
            return 1
            ;;
    esac

    # Return to the original directory
    cd - > /dev/null
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

  # List available nodes
  nodes() {
      BUILT_PACKAGES=$(colcon list --names-only)
      if [ -z "$BUILT_PACKAGES" ]; then
          echo "No packages found. Did you run colcon build?"
          return 1
      else
          echo "$BUILT_PACKAGES" | while IFS= read -r pkg; do
              EXECUTABLES=$(ros2 pkg executables "$pkg")
              echo "$EXECUTABLES"
              echo ""
        done
      fi
  }

  # List of Launch files
  roslaunch_files() {
      local ros_distro=${ROS_DISTRO:-humble}  # Default to "humble" if not set

      # Find system launch files
      local system_files=()
      while IFS= read -r file; do
          system_files+=("$file")
      done < <(fdfind '.*\.(launch\.py|launch\.xml|launch)$' /opt/ros/$ros_distro/share --max-depth 4 -t f)

      # Find workspace launch files
      local workspace_files=()
      while IFS= read -r file; do
          workspace_files+=("$file")
      done < <(colcon list -p | xargs -I {} fdfind '.*\.(launch\.py|launch\.xml|launch)$' "{}" --max-depth 4 -t f)

      # Print results
      printf "%s\n" "${workspace_files[@]}"

      printf "%s\n" "${system_files[@]}"
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
fi


#=============================================================================
# SECTION: LOAD OH-MY-ZSH AND FINALIZE CONFIGURATIONS
#=============================================================================
# Source Oh My Zsh
source $ZSH/oh-my-zsh.sh

# Source powerlevel10k configuration if it exists
[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh

#=============================================================================
# SECTION: CONDA INITIALIZATION 
#=============================================================================
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/amit_152116/miniconda3/bin/conda' 'shell.zsh' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/amit_152116/miniconda3/etc/profile.d/conda.sh" ]; then
# . "/home/amit_152116/miniconda3/etc/profile.d/conda.sh"  # commented out by conda initialize
    else
# export PATH="/home/amit_152116/miniconda3/bin:$PATH"  # commented out by conda initialize
    fi
fi
unset __conda_setup


