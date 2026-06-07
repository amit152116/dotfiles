# Auto-detect ROS distribution from /opt/ros/*/setup.zsh
for ros_setup in /opt/ros/*/setup.zsh(N); do
    if [[ -f "$ros_setup" ]]; then
        ROS_DISTRO=$(basename "$(dirname "$ros_setup")")
        break
    fi
done
if [[ -z "$ROS_DISTRO" && -n "$ROS_INSTALL_PATH" && -f "$ROS_INSTALL_PATH/setup.zsh" ]]; then
    ROS_DISTRO=$(basename "$ROS_INSTALL_PATH")
fi
if [[ -z "$ROS_DISTRO" ]]; then
    return 0
fi

_ros_loaded=false
_ros_extras_loaded=false
_ROS_CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/zsh/ros"
_ROS_STATS_LOG="$_ROS_CACHE_DIR/stats.log"

# --- Stubs: command-triggered lazy load (outside workspace) ---

ros2() {
    unset -f ros2
    unset -f ros
    _load_ros
    ros2 "$@"
}

ros() {
    unset -f ros
    unset -f ros2
    _load_ros
    ros2 "$@"
}

# --- Fast setup: aliases, env vars, completions (no file sourcing) ---

_setup_ros_extras() {
    [[ "$_ros_extras_loaded" == "true" ]] && return 0

    export GZ_VERSION=harmonic
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export MICRO_ROS_RMW_IMPLEMENTATION=rmw_microxrcedds
    export RCUTILS_COLORIZED_OUTPUT=1
    export RCL_LOG_COLORIZE=1
    export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message}"

    [[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh ]] &&
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
    [[ -f /opt/ros/${ROS_DISTRO}/share/ros2cli/environment/ros2-argcomplete.zsh ]] &&
    source /opt/ros/${ROS_DISTRO}/share/ros2cli/environment/ros2-argcomplete.zsh

    alias ros='ros2'
    alias rqt_graph='ros2 run rqt_graph rqt_graph'
    alias rqt_image_view='ros2 run rqt_image_view rqt_image_view'
    alias rqt_plot='ros2 run rqt_plot rqt_plot'
    alias rqt_console='ros2 run rqt_console rqt_console'
    alias rqt_gui='ros2 run rqt_gui rqt_gui'
    alias roslaunch='ros2 launch'
    alias rosdep_install='rosdep install --from-paths src --ignore-src -r -y'

    rospkg() {
        if [[ $# -lt 2 ]]; then
            echo "[ROS] Usage: rospkg <py|cpp> <package_name> [dependencies...]"
            return 1
        fi
        local ws_root
        ws_root="$(pwd)"
        while [[ "$ws_root" != "/" && ! -d "$ws_root/src" ]]; do
            ws_root="$(dirname "$ws_root")"
        done
        if [[ ! -d "$ws_root/src" ]]; then
            echo "[ROS] Error: Could not find a ROS 2 workspace (src/ folder missing)."
            return 1
        fi
        local lang=$1 pkg_name=$2
        shift 2
        local dependencies="$*"
        cd "$ws_root/src" || return 1
        case "$lang" in
            py) ros2 pkg create "$pkg_name" --build-type ament_python --dependencies rclpy "$dependencies" --license GPL-3.0-only ;;
            cpp) ros2 pkg create "$pkg_name" --build-type ament_cmake --dependencies rclcpp "$dependencies" --license GPL-3.0-only ;;
            *)
                echo "[ROS] Invalid language. Use 'py' or 'cpp'."
                return 1
                ;;
        esac
        cd - >/dev/null
    }

    roslog() {
        journalctl -u ros2 -f --no-tail | fzf --prompt="Filter logs: "
    }

    _ros_extras_loaded=true
}

# --- Slow setup: base ROS install (command-triggered path, no workspace) ---

_load_ros() {
    [[ "$_ros_loaded" == "true" ]] && return 0
    source /opt/ros/${ROS_DISTRO}/setup.zsh
    _setup_ros_extras
    _ros_loaded=true
}

# --- Cache helpers ---

_ros_save_cache() {
    local cache_file="$1"
    local ros_vars=(
        PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH
        PYTHONPATH PKG_CONFIG_PATH COLCON_PREFIX_PATH
        ROS_VERSION ROS_PYTHON_VERSION AMENT_CURRENT_PREFIX
    )
    {
        for var in "${ros_vars[@]}"; do
            [[ -n "${(P)var+x}" ]] && typeset -px "$var"
        done
    } >"$cache_file"
}

_ros_log_stat() {
    mkdir -p "$_ROS_CACHE_DIR"
    echo "$(date '+%Y-%m-%d %H:%M:%S') $1 $2" >>"$_ROS_STATS_LOG"
}

# --- Public: cache stats summary ---

ros-cache-stats() {
    if [[ ! -f "$_ROS_STATS_LOG" ]]; then
        echo "[ROS] No stats yet — open a ROS workspace terminal first."
        return
    fi
    local fresh cached total
    fresh=$(grep -c ' FRESH ' "$_ROS_STATS_LOG" 2>/dev/null)
    fresh=${fresh:-0}
    cached=$(grep -c ' CACHED ' "$_ROS_STATS_LOG" 2>/dev/null)
    cached=${cached:-0}
    total=$((fresh + cached))
    echo "[ROS Cache Stats]  log: $_ROS_STATS_LOG"
    echo "  Fresh  (slow source):  $fresh"
    echo "  Cached (fast restore): $cached"
    echo "  Total:                 $total"
    ((total > 0)) && printf "  Hit rate:              %d%%\n" $((cached * 100 / total))
}

# --- chpwd hook: workspace-aware load with env snapshot cache ---

chpwd_ros() {
    local current_dir="$PWD" workspace_dir=""

    while [[ -n "$current_dir" && "$current_dir" != "/" ]]; do
        [[ -f "$current_dir/install/setup.zsh" ]] && workspace_dir="$current_dir" && break
        current_dir="${current_dir%/*}"
    done

    [[ -z "$workspace_dir" || "$_ros_loaded" == "true" ]] && return 0

    mkdir -p "$_ROS_CACHE_DIR"
    local cache_file="$_ROS_CACHE_DIR/${workspace_dir//\//_}"

    if [[ -f "$cache_file" && "$cache_file" -nt "$workspace_dir/install/setup.zsh" ]]; then
        source "$cache_file"
        _ros_log_stat "CACHED" "$workspace_dir"
        echo "[ROS] env restored from cache (${workspace_dir:t})"
    else
        source /opt/ros/${ROS_DISTRO}/setup.zsh
        source "$workspace_dir/install/setup.zsh"
        _ros_save_cache "$cache_file"
        _ros_log_stat "FRESH" "$workspace_dir"
        echo "[ROS] env sourced fresh, snapshot saved (${workspace_dir:t})"
    fi

    _setup_ros_extras
    _ros_loaded=true
}

autoload -U add-zsh-hook
add-zsh-hook chpwd chpwd_ros
chpwd_ros

# --- Fuzzy node runner ---

rosrun() {
    local package node
    package=$(ros2 pkg list | fzf --prompt="Select a package: ")
    [[ -z "$package" ]] && echo "[ROS] No package selected." && return 1
    node=$(ros2 pkg executables "$package" | awk '{print $2}' | fzf --prompt="Select a node: ")
    [[ -z "$node" ]] && echo "[ROS] No node selected." && return 1
    echo "[ROS] Running: ros2 run $package $node"
    ros2 run "$package" "$node"
}
