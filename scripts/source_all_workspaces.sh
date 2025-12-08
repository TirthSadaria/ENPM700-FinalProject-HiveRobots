#!/bin/bash
# Source all required workspaces for multi-robot SLAM
# This includes the main workspace and the dependencies workspace

# Source ROS2 base
source /opt/ros/humble/setup.bash

# Source dependencies workspace (contains multirobot_map_merge)
if [ -f ~/ros_deps_ws/install/setup.bash ]; then
    echo "Sourcing dependencies workspace: ~/ros_deps_ws"
    source ~/ros_deps_ws/install/setup.bash
else
    echo "WARNING: Dependencies workspace not found at ~/ros_deps_ws/install/setup.bash"
    echo "Map merge may not work. Build the workspace first:"
    echo "  cd ~/ros_deps_ws && colcon build"
fi

# Source main project workspace
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
if [ -f "$PROJECT_DIR/install/setup.bash" ]; then
    echo "Sourcing project workspace: $PROJECT_DIR"
    source "$PROJECT_DIR/install/setup.bash"
else
    echo "WARNING: Project workspace not found at $PROJECT_DIR/install/setup.bash"
    echo "Build the workspace first:"
    echo "  cd $PROJECT_DIR && colcon build"
fi

echo "All workspaces sourced successfully!"
echo ""
echo "Available packages:"
ros2 pkg list | grep -E "(multirobot|map_merge|hive_control)" || echo "No matching packages found"

