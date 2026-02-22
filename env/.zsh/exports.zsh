# Paths
export PATH="$HOME/.local/bin:$HOME/.dotfiles/scripts:$HOME/.fzf/bin:/usr/local/bin:/usr/lib/ccache:/snap/bin:$PATH"

# opencode
export PATH=/home/amit_152116/.opencode/bin:$PATH

export DOTFILES_DIR=$HOME/.dotfiles
export MANPAGER="sh -c 'col -bx | batcat -l man -p'"

# Cargo [Rust Manager]
if [[ -d "$HOME/.cargo/bin" ]]; then
    export PATH="$HOME/.cargo/bin:$PATH"
fi

# CUDA
if [[ -d "/usr/local/cuda" ]]; then
    export PATH="/usr/local/cuda/bin:$PATH"
    export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"
fi

# Node.js
if [[ -d "/usr/local/nodejs/bin" ]]; then
    export PATH="/usr/local/nodejs/bin:$PATH"
fi

# GO
if command -v go &>/dev/null; then
    export PATH="$(go env GOPATH)/bin:$PATH"
fi

# Editors
if command -v nvim &>/dev/null; then
    export EDITOR="nvim"
    export VISUAL="nvim"
else
    export EDITOR="vim"
    export VISUAL="vim"
fi

# Docker
if command -v docker &>/dev/null; then
    export DOCKER_HOST=unix:///var/run/docker.sock
    export DOCKER_BUILDKIT=1
fi

# Check if Android SDK exists before loading Android configurations
if [[ -d "$HOME/Android/Sdk" ]]; then
    # Android SDK settings
    export ANDROID_HOME="$HOME/Android/Sdk"
    export PATH="$PATH:$ANDROID_HOME/emulator"
    alias debugPath="app/build/outputs/apk/debug/app-debug.apk"
    alias releasePath="app/build/outputs/apk/release/app-release.apk"
fi

if [[ -d "$HOME/ardu_ws" ]]; then

    export PATH=$PATH:$HOME/ardu_ws/Micro-XRCE-DDS-Gen/scripts
    export PATH=$PATH:$HOME/ardu_ws/src/ardupilot/Tools/autotest
    source $HOME/ardu_ws/src/ardupilot/Tools/completion/completion.zsh

    # export GZ_FUEL_CACHE_ONLY=1
    # export GZ_FUEL_DOWNLOAD_MODE=none

fi

# Set GAZEBO_MODEL_PATH if required
if [[ -d $HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models ]]; then
    export GAZEBO_MODEL_PATH=$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH
fi
