#!/usr/bin/env python3
"""
Integration launch file for TDLE exploration with bcr_bot simulation.

This launch file starts:
1. bcr_bot Gazebo simulation (Ignition Gazebo)
2. SLAM Toolbox for online mapping
3. Nav2 for navigation
4. TDLE exploration nodes
5. RViz2 for visualization
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    bcr_bot_dir = get_package_share_directory('bcr_bot')
    tdle_dir = get_package_share_directory('tdle')
    pkg_slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file')
    autostart = LaunchConfiguration('autostart', default='true')

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

    declare_world_file = DeclareLaunchArgument(
        'world_file',
        # default_value=os.path.join(bcr_bot_dir, 'worlds', 'small_warehouse.sdf'),
        default_value=os.path.join(bcr_bot_dir, 'worlds', 'small_room.sdf'),
        description='Full path to the Gazebo world file'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the navigation stack'
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

    # Log level configuration
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    declare_nav_log_level = DeclareLaunchArgument(
        'nav_log_level',
        default_value='warn',
        description='Logging level for navigation nodes (controller_server, bt_navigator, etc.)'
    )

    declare_explore_log_level = DeclareLaunchArgument(
        'explore_log_level',
        default_value='info',
        description='Logging level for exploration nodes'
    )

    # Set Gazebo resource paths
    gz_env_var = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(bcr_bot_dir, 'worlds')
    )

    gz_model_env_var = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(bcr_bot_dir, 'models')
    )

    # 1. Start Gazebo simulation
    gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PythonExpression(["'", world_file, " -r'"])
        }.items()
    )

    # 2. Spawn bcr_bot robot
    spawn_bcr_bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bcr_bot_dir, 'launch', 'bcr_bot_ign_spawn.launch.py')
        )
    )

    # 2.5. Cmd_vel remapper for keyboard teleop
    # The robot expects /bcr_bot/cmd_vel but teleop publishes to /cmd_vel
    # bcr_bot provides a remapper node that handles this
    cmd_vel_remapper = Node(
        package='bcr_bot',
        executable='remapper.py',
        name='cmd_vel_remapper',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 3. Start SLAM Toolbox (online async) - Using fixed config for mapping mode
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(
                get_package_share_directory('explore_with_bcr'),
                'config', 'slam_toolbox_mapping.yaml'
            ),
        }.items()
    )

    # 4. Start Nav2 (minimal bringup - just for navigation, not localization)
    # Note: Even with slam:=True,need to provide a dummy map argument as it's required by bringup_launch.py
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': os.path.join(bcr_bot_dir, 'config', 'nav2_params.yaml'),
            'slam': 'True',  # Enable SLAM mode in Nav2 (disables AMCL, map server)
            'map': '/dev/null',  # Dummy path required by bringup_launch.py even when SLAM is enabled
        }.items()
    )

    # 5. Start TDLE exploration nodes
    # These nodes detect frontier points, filter them, arrange subregions, and select targets
    fpoint_detector = Node(
        package='tdle',
        executable='fpoint_detector',
        name='fpoint_detector',
        output='screen',
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
        parameters=[
            tdle_params_file,
            {
                'map_topic': map_topic,
                'odom_topic': odom_topic,
                'use_sim_time': use_sim_time,
                'min_points_per_region': 2,
                'grid_divisions': 3,
            }
        ],
    )

    fpoint_selection = Node(
        package='tdle',
        executable='fpoint_selection',
        name='fpoint_selection',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('explore_log_level')],
        parameters=[
            tdle_params_file,
            {
                'map_topic': map_topic,
                'odom_topic': odom_topic,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # 6. Start RViz2 with bcr_bot's map visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(bcr_bot_dir, 'rviz', 'exp.rviz')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add declarations
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world_file)
    ld.add_action(declare_autostart)
    ld.add_action(declare_map_topic)
    ld.add_action(declare_odom_topic)
    ld.add_action(declare_costmap_topic)
    ld.add_action(declare_log_level)
    ld.add_action(declare_nav_log_level)
    ld.add_action(declare_explore_log_level)

    # Add environment variables
    ld.add_action(gz_env_var)
    ld.add_action(gz_model_env_var)

    # Add Gazebo simulation
    ld.add_action(gz_sim)
    ld.add_action(spawn_bcr_bot)
    ld.add_action(cmd_vel_remapper)  # Remap /cmd_vel to /bcr_bot/cmd_vel for teleop

    # Add SLAM and Nav2
    ld.add_action(slam_toolbox)
    ld.add_action(nav2_bringup)

    # Add TDLE exploration nodes
    ld.add_action(fpoint_detector)
    ld.add_action(fpoint_filter)
    ld.add_action(subregion_arrangement)
    ld.add_action(fpoint_selection)

    # Add visualization
    ld.add_action(rviz)

    return ld
