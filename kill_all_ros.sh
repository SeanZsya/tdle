#!/bin/bash
# 一键清理所有ROS2和Gazebo进程

echo "=== Stopping ROS2 daemon ==="
ros2 daemon stop 2>/dev/null

echo "=== Killing all ROS/Gazebo processes ==="
pkill -9 -f "parameter_bridge" 2>/dev/null
pkill -9 -f "fpoint_detector" 2>/dev/null
pkill -9 -f "fpoint_filter" 2>/dev/null
pkill -9 -f "fpoint_selection" 2>/dev/null
pkill -9 -f "subregion_arrangement" 2>/dev/null
pkill -9 -f "cmd_vel_remapper" 2>/dev/null
pkill -9 -f "static_transform_publisher" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ign gazebo" 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
pkill -9 -f "ros2-daemon" 2>/dev/null
pkill -9 -f "component_container" 2>/dev/null
pkill -9 -f "slam_toolbox" 2>/dev/null
pkill -9 -f "nav2" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
pkill -9 -f "robot_state_publisher" 2>/dev/null
pkill -9 -f "joint_state_publisher" 2>/dev/null

sleep 2

echo "=== Restarting ROS2 daemon ==="
ros2 daemon start 2>/dev/null

echo "=== Remaining processes ==="
remaining=$(ps aux | grep -E "(fpoint|subregion|gz|parameter_bridge|cmd_vel_remapper|static_transform_publisher|slam_toolbox|nav2)" | grep -v grep | wc -l)
if [ "$remaining" -eq 0 ]; then
    echo "All cleaned up!"
else
    echo "Warning: $remaining processes still running"
    ps aux | grep -E "(fpoint|subregion|gz|parameter_bridge)" | grep -v grep
fi

echo ""
echo "=== ros2 node list ==="
timeout 5 ros2 node list || echo "(no nodes)"
