# explore_with_bcr

Integration of TDLE (2D LiDAR Exploration) with bcr_bot simulation platform.

## Overview

This package integrates two existing projects:
- **bcr_bot**: Differential-drive robot simulation in Ignition Gazebo
- **TDLE**: Frontier-based autonomous exploration framework

## Prerequisites

1. **bcr_bot workspace** must be built:
   ```bash
   cd /home/seanz/rbt/explore_from_zero/bcr_bot_ws
   source /opt/ros/humble/setup.bash
   colcon build
   ```

2. **TDLE workspace** must be built:
   ```bash
   cd /home/seanz/rbt/explore_from_zero/tdle
   source /opt/ros/humble/setup.bash
   colcon build
   ```

## Building

```bash
cd ~/explore_with_bcr_ws
source /opt/ros/humble/setup.bash
source /home/seanz/rbt/explore_from_zero/bcr_bot_ws/install/setup.bash
source /home/seanz/rbt/explore_from_zero/tdle/install/setup.bash
colcon build
```

## Usage

### Using the Helper Script

A convenience script is provided for easy launching:

```bash
cd ~/explore_with_bcr_ws/src/explore_with_bcr
./start_exploration.sh              # Start everything
./start_exploration.sh only         # Start exploration only
./start_exploration.sh help         # Show help
```

### Full Integration (Gazebo + SLAM + Nav2 + Exploration)

```bash
cd ~/explore_with_bcr_ws
source install/setup.bash
ros2 launch explore_with_bcr explore_with_mapping.launch.py
```

This starts:
- Ignition Gazebo simulation with bcr_bot
- SLAM Toolbox for online mapping
- Nav2 for navigation
- TDLE exploration nodes (frontier detection, filtering, subregion arrangement, target selection)
- RViz2 for visualization

### Exploration Only (when simulation is already running)

```bash
cd ~/explore_with_bcr_ws
source install/setup.bash
ros2 launch explore_with_bcr explore_only.launch.py
```

Use this when:
- Gazebo is already running from another terminal
- You want to restart exploration without restarting simulation
- You're debugging exploration behavior

### With Custom World

```bash
ros2 launch explore_with_bcr explore_with_mapping.launch.py \
    world_file:=/path/to/custom_world.sdf
```

## Architecture

```
[Ignition Gazebo] → /bcr_bot/odom, /bcr_bot/scan → [SLAM Toolbox] → /map
                                                        ↓
[TDLE fpoint_detector] ← /map, /bcr_bot/odom ────────┘
            ↓
[TDLE fpoint_filter] ← /global_costmap/costmap
            ↓
[TDLE subregion_arrangement] ← /map, /bcr_bot/odom
            ↓
[TDLE fpoint_selection] ← /map, /bcr_bot/odom
            ↓
    navigate_to_pose (Action)
            ↓
        [Nav2]
            ↓
    /cmd_vel → [bcr_bot in Gazebo]
```

## Topic Configuration

The key integration point is the odometry topic. bcr_bot publishes to `/bcr_bot/odom`,
so TDLE nodes are configured to subscribe to this topic:

| TDLE Node | Parameter | Value |
|-----------|-----------|-------|
| fpoint_detector | odom_topic | `/bcr_bot/odom` |
| fpoint_filter | costmap_topic | `/global_costmap/costmap` |
| subregion_arrangement | odom_topic | `/bcr_bot/odom` |
| fpoint_selection | odom_topic | `/bcr_bot/odom` |

## Debugging

Check topic data:
```bash
# Check odometry
ros2 topic echo /bcr_bot/odom

# Check laser scan
ros2 topic echo /bcr_bot/scan

# Check map
ros2 topic echo /map

# Check frontier points
ros2 topic echo /detected_points
ros2 topic echo /filtered_points
ros2 topic echo /candidate_fpoint
```

Check actions:
```bash
ros2 action list
ros2 action info /navigate_to_pose
```

## Troubleshooting

### Error: "Included launch description missing required argument 'map'"

**Cause**: Nav2's `bringup_launch.py` requires a `map` argument even when using SLAM mode.

**Fix**: The launch file now sets `slam:='True'` and provides a dummy map path (`map:='/dev/null'`). When SLAM is enabled, Nav2 will not use the static map file.

### TDLE nodes not receiving odometry

Check that the odometry topic is correct:
```bash
ros2 topic echo /bcr_bot/odom
```

If empty, check the ros_gz_bridge is running:
```bash
ros2 topic list | grep odom
```

## Files

- `launch/explore_with_mapping.launch.py`: Full integration launch (Gazebo + SLAM + Nav2 + TDLE)
- `launch/explore_only.launch.py`: TDLE exploration nodes only
- `config/tdle_params.yaml`: TDLE parameter configuration
- `start_exploration.sh`: Helper script for easy launching
- `test/test_integration.sh`: Integration tests
- `test/test_launch_files.py`: Python unit tests
- `README.md`: This file
