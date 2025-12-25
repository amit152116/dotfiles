# Check Humble (Ubuntu 22.04)
if [[ -f "/opt/ros/humble/setup.zsh" ]]; then
    ROS_DISTRO="humble"
    # Check Iron (Ubuntu 22.04)
elif [[ -f "/opt/ros/iron/setup.zsh" ]]; then
    ROS_DISTRO="iron"
    # Check Jazzy (Ubuntu 20.04)
elif [[ -f "/opt/ros/foxy/setup.zsh" ]]; then
    ROS_DISTRO="jazzy"
    # User-defined ROS location
elif [[ -n "$ROS_INSTALL_PATH" && -f "$ROS_INSTALL_PATH/setup.zsh" ]]; then
    ROS_DISTRO=$(basename "$ROS_INSTALL_PATH")
fi

# Only load ROS configuration if found
if [[ -z "$ROS_DISTRO" ]]; then
    # just skip silently
    return 0
fi

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.zsh

# Lazy loading ROS configuration
_ros_loaded=false

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

# Function to load ROS environment
_load_ros() {
    if [[ "$_ros_loaded" == "true" ]]; then
        return 0
    fi
    echo "Loading ROS environment..."

    # ROS settings
    export GZ_VERSION=harmonic
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_LOCALHOST_ONLY=0
    export RCUTILS_COLORIZED_OUTPUT=1
    export RCL_LOG_COLORIZE=1

    # GZ/ROS
    if [[ -d "$HOME/ardupilot_gazebo" ]]; then
        export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
        export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
        # export GZ_FUEL_CACHE_ONLY=1
        # export GZ_FUEL_DOWNLOAD_MODE=none

    fi

    # export CYCLONEDDS_URI=file:///home/amit_152116/Documents/aim_ros2/cyclonedds.xml

    # Check for ROS2 installations in common locations
    local ROS_WS="$HOME/Documents/aim_ros2/"

    # Source workspace if it exists
    if [[ -d "$ROS_WS" ]]; then
        source "$ROS_WS"install/setup.zsh
    fi

    # Source completion scripts if they exist
    if [[ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh ]]; then
        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
    fi
    if [[ -f /opt/ros/${ROS_DISTRO}/share/ros2cli/environment/ros2-argcomplete.zsh ]]; then
        source /opt/ros/${ROS_DISTRO}/share/ros2cli/environment/ros2-argcomplete.zsh
    fi

    # Set GAZEBO_MODEL_PATH if required
    if [[ -d $HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models ]]; then
        export GAZEBO_MODEL_PATH=$HOME/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$GAZEBO_MODEL_PATH
    fi

    _ros_loaded=true
    echo "ROS environment loaded!"

    # ROS2 aliases
    alias ros='ros2'
    alias rqt_graph='ros2 run rqt_graph rqt_graph'
    alias rqt_image_view='ros2 run rqt_image_view rqt_image_view'
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
        cd - >/dev/null
    }

    # Display ROS logs in real time with filtering
    roslog() {
        journalctl -u ros2 -f --no-tail | fzf --prompt="Filter logs: "
    }

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

# Auto-load when entering ROS workspace
chpwd_ros() {
    if [[ -f "$(pwd)/install/setup.zsh" ]] || [[ -f "$(pwd)/../install/setup.zsh" ]] || [[ "$(pwd)" == *"aim_ros2"* ]]; then
        if [[ "$_ros_loaded" == "false" ]]; then
            _load_ros
        fi
    fi
}

# Add the function to chpwd hooks (runs when changing directories)
autoload -U add-zsh-hook
add-zsh-hook chpwd chpwd_ros

# Run once on shell startup (in case we start directly in a ROS workspace)
chpwd_ros
