#!/bin/bash
set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
    echo "ROS setup file not found: $ROS_SETUP" >&2
    exit 1
fi

source "$ROS_SETUP"

for workspace_setup in \
    "$PROJECT_ROOT/bcr_bot_ws/install/setup.bash" \
    "$PROJECT_ROOT/tdle_ws/install/setup.bash" \
    "$PROJECT_ROOT/explore_with_bcr_ws/install/setup.bash"
do
    if [ ! -f "$workspace_setup" ]; then
        echo "Workspace setup file not found: $workspace_setup" >&2
        echo "Run ./build_all.sh first." >&2
        exit 1
    fi
    source "$workspace_setup"
done

exec ros2 launch explore_with_bcr explore_with_mapping.launch.py
