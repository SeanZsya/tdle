#ifndef COMMON_FUNC_H
#define COMMON_FUNC_H
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

float euclideanDistance( vector<float> , vector<float> );

float sign(float );

vector<float> Nearest(vector<vector<float> >,vector<float> );

vector<float> Steer( vector<float>, vector<float>, float );

int gridValue(nav_msgs::OccupancyGrid &,vector<float>);

int ObstacleFree(vector<float> , vector<float> & , nav_msgs::OccupancyGrid);

void visMarkerInit(visualization_msgs::Marker &marker, string ID, int type, int action, float scale, float r, float g, float b, float a);

void visMarkerSet (visualization_msgs::Marker &marker, vector<float> &point);


#endif