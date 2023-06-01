#ifndef FPOINT_FILTER_NODE_H
#define FPOINT_FILTER_NODE_H

#include <ros/ros.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <tdle/PointArray.h>

using namespace std;

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);

void costmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);

void fpointCallBack(const geometry_msgs::PointStamped& msg);

vector<geometry_msgs::Point> disFilter(geometry_msgs::Point point, vector<geometry_msgs::Point>& fpoints);

vector<geometry_msgs::Point> meanshiftFilter(geometry_msgs::Point point, vector<geometry_msgs::Point> &fpoints, float kernel_bandwidth, float convergence_threshold);

tdle::PointArray delPrevious(vector<geometry_msgs::Point>& filtered_points, visualization_msgs::Marker& fpoint_vis) ;



#endif