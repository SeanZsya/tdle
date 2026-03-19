#!/usr/bin/env python3
"""
Launch file for TDLE exploration only (assuming bcr_bot simulation is already running).

This is useful for:
- Debugging exploration without restarting Gazebo
- Testing with different exploration parameters
- Running exploration on an already-mapped environment

Prerequisites:
- Gazebo simulation with bcr_bot is running
- SLAM Toolbox is running (or a map server is providing /map)
- Nav2 is running
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    tdle_dir = get_package_share_directory('tdle')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Topic configurations for TDLE - using bcr_bot's topics
    map_topic = LaunchConfiguration('map_topic', default='/map')
    odom_topic = LaunchConfiguration('odom_topic', default='/bcr_bot/odom')
    costmap_topic = LaunchConfiguration('costmap_topic', default='/global_costmap/costmap')
    tdle_params_file = os.path.join(tdle_dir, 'param', 'tdle_params.yaml')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_topic = DeclareLaunchArgument(
        'map_topic',
        default_value='/map',
        description='Topic name for the occupancy grid map'
    )

    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/bcr_bot/odom',
        description='Topic name for odometry (bcr_bot uses /bcr_bot/odom)'
    )

    declare_costmap_topic = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/global_costmap/costmap',
        description='Topic name for the global costmap'
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # TDLE exploration nodes
    fpoint_detector = Node(
        package='tdle',
        executable='fpoint_detector',
        name='fpoint_detector',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            tdle_params_file,
            {
                'stride': 0.5,
                'map_topic': map_topic,
                'odom_topic': odom_topic,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    fpoint_filter = Node(
        package='tdle',
        executable='fpoint_filter',
        name='fpoint_filter',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            tdle_params_file,
            {
                'map_topic': map_topic,
                'costmap_topic': costmap_topic,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    subregion_arrangement = Node(
        package='tdle',
        executable='subregion_arrangement',
        name='subregion_arrangement',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            tdle_params_file,
            {
                'map_topic': map_topic,
                'odom_topic': odom_topic,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    fpoint_selection = Node(
        package='tdle',
        executable='fpoint_selection',
        name='fpoint_selection',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            tdle_params_file,
            {
                'map_topic': map_topic,
                'odom_topic': odom_topic,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # Create launch description
    ld = LaunchDescription()

    # Add declarations
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_topic)
    ld.add_action(declare_odom_topic)
    ld.add_action(declare_costmap_topic)
    ld.add_action(declare_log_level)

    # Add TDLE exploration nodes
    ld.add_action(fpoint_detector)
    ld.add_action(fpoint_filter)
    ld.add_action(subregion_arrangement)
    ld.add_action(fpoint_selection)

    return ld
