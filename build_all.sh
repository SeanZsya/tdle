#!/bin/bash
set -e
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash

echo "Building bcr_bot_ws..."
cd "$PROJECT_ROOT/bcr_bot_ws"
colcon build --symlink-install --allow-overriding bcr_bot

echo "Building tdle_ws..."
cd "$PROJECT_ROOT/tdle_ws"
colcon build --symlink-install --allow-overriding tdle

echo "Building explore_with_bcr_ws..."
cd "$PROJECT_ROOT/explore_with_bcr_ws"
source ../bcr_bot_ws/install/setup.bash
source ../tdle_ws/install/setup.bash
colcon build --symlink-install

echo "Build complete!"
