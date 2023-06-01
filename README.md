# TDLE

[The corresponding paper *TDLE: 2D Lidar Exploration with Hierarchical Planning Using Regional Division* has been accepted by IEEE CASE 2023.]


## Introduction

Demonstration Video:

[![TDLE: 2D Lidar Exploration with Hierarchical Planning Using Regional Division](https://res.cloudinary.com/marcomontalbano/image/upload/v1685324236/video_to_markdown/images/youtube--aPXxOKf1o10-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/aPXxOKf1o10 "TDLE: 2D Lidar Exploration with Hierarchical Planning Using Regional Division")

### Frontier Detection
The frontier detection module is adapted from Hasauino's [rrt_exploration](https://github.com/hasauino/rrt_exploration) (sincere thanks for his work), with series of improvements, including:

  - Set sampling boundary automatically;

  - Dynamically adjust the number of nodes in the global tree.

The entire code has been refactored for clearer logic and efficient implementation as well.

### Hierarchical Planning

We use a hierarchical planning approach, which divides the exploration space into several regions and arrange their orders firstly, then selects exploration targets for each region separately. 

A method of dividing regions and sequential planning is designed to obtain a global perspective with few increase of computational overhead. Indicators for revenue calculation have been completely redesigned to fit the subregion arrangement.

### Other Supporting Modules

  - Added functions of returning and escaping. Return to initial point once exploration finished; try to escape by backing to former point when getting stuck.

  - Introducing Cartographer into exploration system. Changed the value representation of occupancy grids, so they can cooperate with ROS navigation stack. 

## Quick Start

### Prerequisites

  We tried to minimize the unnecessary use of third-party libraries (like OpenCV, tf, etc). Simple external functions were implemented manually.

  - **Basic ROS Environment**
    
    Make sure you have ROS environment fully installed. This project has been tested on serval devices under ROS-Melodic. (recommand `sudo apt install ros-melodic-desktop-full`)
    
    sudo apt install ros-noetic-navigation
    sudo apt install ros-noetic-teb-local-planner

  - **Cartographer_ROS**
    
    TDLE use cartographer as its mapping module, with a few adaptions that make it suitable for exploration. Using [this fork](https://github.com/SeanZsya/cartographer_ros) instead of original repository. 
    
      In fact, TDLE supports any method that can generate an occupancy grid map with message type `nav_msgs/OccupancyGrid`.
    
### Run TDLE

  - **For Gazebo Simulation**
    
        roslaunch tdle tdle_sim.launch

  - **For Field Test**
        
        roslaunch tdle tdle_real.launch
        
     Note: If you use SSH to establish a connection with the robot, RViz will not open on the operation interface. You need to use remote desktop or setup master/slave communication in ROS for visualization.
