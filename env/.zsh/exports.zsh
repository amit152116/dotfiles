# Paths
export PATH="$HOME/.local/bin:$HOME/.dotfiles/scripts:$HOME/.fzf/bin:/usr/local/bin:/usr/sbin:/sbin:/usr/lib/ccache:/snap/bin:$PATH"

export DOTFILES_DIR=$HOME/.dotfiles
export MANPAGER="sh -c 'col -bx | batcat -l man -p'"

export NVM_DIR="$HOME/.config/nvm"

# Resolve nvm's default node and put its bin on PATH directly (fast; works in
# tmux popups/scripts that skip .zshrc since the server inherits this PATH).
if [ -f "$NVM_DIR/alias/default" ]; then
    _nvm_def="$(cat "$NVM_DIR/alias/default")"
    _nvm_bin="$(ls -d "$NVM_DIR/versions/node/v${_nvm_def#v}"*/bin 2>/dev/null | sort -V | tail -1)"
    [ -n "$_nvm_bin" ] && export PATH="$_nvm_bin:$PATH"
    unset _nvm_def _nvm_bin
fi

[[ -f "$HOME/.env.local" ]] && source "$HOME/.env.local"

# Cargo [Rust Manager]
if [[ -d "$HOME/.cargo/bin" ]]; then
    export PATH="$HOME/.cargo/bin:$PATH"
fi

# GO
if [[ -d "$HOME/go/bin" ]]; then
    export PATH="$HOME/go/bin:$PATH"
fi

# opencode
if [[ -d "$HOME/.opencode/bin" ]]; then
    export PATH=$HOME/.opencode/bin:$PATH
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
    # Lazy-load ardupilot completion on first use of ardupilot commands
    _ardu_completion="$HOME/ardu_ws/src/ardupilot/Tools/completion/completion.zsh"
    [[ -f "$_ardu_completion" ]] && source "$_ardu_completion"
    unset _ardu_completion

    # export GZ_FUEL_CACHE_ONLY=1
    # export GZ_FUEL_DOWNLOAD_MODE=none

fi

# Set GAZEBO_MODEL_PATH if required
if [[ -d $HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models ]]; then
    export GAZEBO_MODEL_PATH=$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH
fi
