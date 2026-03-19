#ifndef COMMON_FUNC_H
#define COMMON_FUNC_H

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tdle/msg/point_array.hpp"

using namespace std;

float euclideanDistance(vector<float>, vector<float>);

float sign(float);

vector<float> Nearest(vector<vector<float> >, vector<float>);

vector<float> Steer(vector<float>, vector<float>, float);

int gridValue(nav_msgs::msg::OccupancyGrid &, vector<float>);

int ObstacleFree(vector<float>, vector<float> &, nav_msgs::msg::OccupancyGrid);

void visMarkerInit(visualization_msgs::msg::Marker &marker, string ID, int type, int action,
                   float scale, float r, float g, float b, float a);

void visMarkerSet(visualization_msgs::msg::Marker &marker, vector<float> &point);

#endif
