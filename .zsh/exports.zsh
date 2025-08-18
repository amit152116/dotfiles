
# Paths
export PATH="$HOME/.local/bin:/usr/local/bin:/usr/lib/ccache:/snap/bin:$PATH"

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
fi

# Check if Android SDK exists before loading Android configurations
if [[ -d "$HOME/Android/Sdk" ]]; then
  # Android SDK settings
  export ANDROID_HOME="$HOME/Android/Sdk"
  export PATH="$PATH:$ANDROID_HOME/emulator"
  alias debugPath="app/build/outputs/apk/debug/app-debug.apk"
  alias releasePath="app/build/outputs/apk/release/app-release.apk"
fi

# Add Ardupilot-Autopilot if installed
if [[ -d "$HOME/ardupilot" ]]; then
  export PATH=$PATH:$HOME/ardupilot/Tools/autotest
  source $HOME/ardupilot/Tools/completion/completion.zsh
fi

# Add Micro-XRCE-DDS-Gen if installed
# if [[ -d "$HOME/ardupilot/Micro-XRCE-DDS-Gen" ]]; then
#   export PATH=$PATH:$HOME/ardupilot/Micro-XRCE-DDS-Gen/scripts
# fi



# if [[ -d "$HOME/ardupilot/src/ardupilot_gazebo" ]]; then
#   export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
#   export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot/src/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
#   export GZ_FUEL_CACHE_ONLY=1
#   export GZ_FUEL_DOWNLOAD_MODE=none
#   source /usr/local/share/ardupilot_gazebo/local_setup.zsh
#   source $HOME/ardupilot/install/setup.zsh
#
# fi

# GZ/ROS
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
