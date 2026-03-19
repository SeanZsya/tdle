# TDLE

[The corresponding paper ***[TDLE: 2D Lidar Exploration with Hierarchical Planning Using Regional Division](https://arxiv.org/pdf/2307.02852.pdf)*** has been accepted by IEEE CASE 2023.]


## Introduction

Demonstration Video:

[![TDLE: 2D Lidar Exploration with Hierarchical Planning Using Regional Division](https://res.cloudinary.com/marcomontalbano/image/upload/v1685324236/video_to_markdown/images/youtube--aPXxOKf1o10-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/aPXxOKf1o10 "TDLE: 2D Lidar Exploration with Hierarchical Planning Using Regional Division")

### Frontier Detection
The frontier detection module is adapted from Hasauino's [rrt_exploration](https://github.com/hasauino/rrt_exploration) (sincere thanks for his work), with series of improvements, including:
  
  - Refactored the entire code for clearer logic and efficient implementation;

  - Set sampling boundary automatically;

  - Dynamically adjust the number of nodes in the global tree.

### Hierarchical Planning

The hierarchical exploration planning module is the core component of this framework. 

In the first phase, the planning space is divided into several subregions. After that, subregions with frontier points inside or mostly unknown are selected. The selected subregions are then sorted to provide global guidance for exploration. 

In the second phase, the subregions are visited one by one in the order determined in the first phase. In each subregion, various indicators, including global compatibility, information gain, and motion consistency, are calculated. Robot selects the exploration target with the highest comprehensive revenue.

## Quick Start (ROS2 - Current)

This repository now includes a ROS2 implementation (`tdle` package format 3, `ament_cmake`).

### Prerequisites

- ROS2 environment (recommended: Humble or compatible distribution with Nav2 and Gazebo/Ignition stack)
- `nav2_bringup`
- `slam_toolbox` (when running mapping mode)
- `ros_gz_sim` (for Ignition/Gazebo simulation integration)

### Build

Build `tdle` in your ROS2 workspace:

```bash
cd tdle
./build_all.sh
```

### Run (Integrated simulation with bcr_bot)

If you use the integrated ROS2 stack in this repository (`explore_with_bcr_ws`):

```bash
cd tdle
./run_explore_with_mapping.sh
```
