#ifndef SUBREGION_ARRANGEMENT_NODE_H
#define SUBREGION_ARRANGEMENT_NODE_H

#include <map>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <sstream>
#include <vector>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <random>
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

void odomCallBack(const nav_msgs::Odometry& msg);

void fpCallBack(const tdle::PointArray& msg);

void drawSubRegi(vector<int> serial, ros::Publisher marker_pub);

void drawGrid(ros::Publisher marker_pub);

bool isInside(geometry_msgs::Point center, geometry_msgs::Point point);

bool isUnknown(geometry_msgs::Point center);

vector<int> selectSubRegi();

vector<int> sortSubRgi_tsp(int full_graph[10][10], int x_bot_grid, vector<int> selected_sr);

vector<int> sortSubRgi_bfs(vector<vector<float>> full_graph,vector<int> tbd_waypoints,vector<int> previous_path,int sr_bot);

float similarity_dtw(vector<vector<float>> full_graph, vector<int> route1, vector<int> route2);

vector<vector<geometry_msgs::Point>> classifyFpoints(vector<int> sr_sorted);

float navi_cost(geometry_msgs::Point next_center, geometry_msgs::Point fpoint);

#endif