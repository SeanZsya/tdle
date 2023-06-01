#include <tdle/common_func.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>
#include <ctime>

using namespace std;

nav_msgs::Odometry odom;
nav_msgs::OccupancyGrid mapData;
geometry_msgs::Point map_origin, next_center;
float map_width, map_height;


void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapData = *msg;
	map_width = msg->info.width * msg->info.resolution;
	map_height = msg->info.height * msg->info.resolution;
	map_origin = msg->info.origin.position;
}

void odomCallBack(const nav_msgs::Odometry& msg)
{
	odom = msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "FPonit_Assigner");
	ros::NodeHandle nh;

	string map_topic, odom_topic;

	ros::param::param<std::string>("/map_topic", map_topic, "/map"); 
	ros::param::param<std::string>("/odom_topic", odom_topic, "/odom"); 
	ros::Subscriber mapsub = nh.subscribe(map_topic, 10, mapCallBack);	
	ros::Subscriber odomsub = nh.subscribe(odom_topic, 10, odomCallBack);
	

	ros::Rate rate(1); // run at 1Hz
	
	// wait until map is received
	while (mapData.header.seq<1 or mapData.data.size()<1)  {ros::spinOnce();  rate.sleep();}

	// Get current time
	time_t currentTime;
	time(&currentTime);

	// Convert current time to string
	char* timeString = ctime(&currentTime);
	timeString[strlen(timeString) - 1] = '\0';
	// Create filename with current time
	string filePath = "/home/sean/tdle_data/";
	string filename = filePath + string(timeString) + ".txt";

	// save robot position and map size to a txt file, name with current time
	ofstream outputFile(filename);

	// Main loop
	while (ros::ok())
	{	
		outputFile << odom.pose.pose.position.x << " " << odom.pose.pose.position.y << " " << map_width * map_height << endl;

		ros::spinOnce();
		rate.sleep();
	}
	outputFile.close();
	return 0;
}