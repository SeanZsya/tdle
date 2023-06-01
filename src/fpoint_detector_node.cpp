#include <tdle/common_func.h>
#include <tdle/fpoint_detector_node.h>

using namespace std;

nav_msgs::Odometry odom;
nav_msgs::OccupancyGrid mapData;
geometry_msgs::Point map_origin;
float map_width, map_height;
vector<float> x_bot;
geometry_msgs::PointStamped detected_point;
visualization_msgs::Marker grid_vis;
float stride;

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapData = *msg;
	map_width = mapData.info.width * mapData.info.resolution;
	map_height = mapData.info.height * mapData.info.resolution;
	map_origin = mapData.info.origin.position;
}

void odomCallBack(const nav_msgs::Odometry& msg)
{
	odom = msg;
	x_bot.clear();
	x_bot.push_back(odom.pose.pose.position.x);
	x_bot.push_back(odom.pose.pose.position.y);
}

int rrtExpand (vector< vector<float> > &tree_nodes, visualization_msgs::Marker &vis_marker)
{
	vector<float> x_rand,x_nearest,x_new,x_start;
	float xr,yr;
	// Sampling random point 
	// Mersenne Twister generator, faster than rand()
	random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0, 1);
	xr = dis(gen) * map_width + map_origin.x;
	yr = dis(gen) * map_height + map_origin.y;
	x_rand.clear();
	x_rand.push_back(xr); 
	x_rand.push_back(yr);
	x_nearest=Nearest(tree_nodes,x_rand);
	x_new=Steer(x_nearest,x_rand,stride);
	// Check new point
	// ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
	int det_check=ObstacleFree(x_nearest,x_new,mapData);
	if (det_check==-1)
	{
		detected_point.header.stamp=ros::Time(0);
		detected_point.header.frame_id=mapData.header.frame_id;
		detected_point.point.x=x_new[0];
		detected_point.point.y=x_new[1];
		detected_point.point.z=0.0;
		// visMarkerSet(fpoint_g_vis,x_new);
	}	
	else if (det_check==1)
	{
		tree_nodes.push_back(x_new);
		// keep enough points, dicard the rest
		if (tree_nodes.size()> (map_height*map_width/stride/stride))
			tree_nodes.erase(tree_nodes.begin());
		// for visualization only
		if (vis_marker.points.size()> (map_height*map_width/stride/stride))
			vis_marker.points.erase(vis_marker.points.begin(),vis_marker.points.begin()+2);
		visMarkerSet(vis_marker,x_new);
		visMarkerSet(vis_marker,x_nearest);
	} 
	return det_check;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fpoint_detector");
	ros::NodeHandle nh;
	
	// fetching all parameters
	vector< std::vector<float>  > V_g, V_l;
	visualization_msgs::Marker fpoint_g_vis, tree_g_vis, tree_l_vis;

	ros::param::param<float>("/stride", stride, 0.5);
	
	string map_topic, odom_topic;
    ros::param::param<std::string>("/map_topic", map_topic, "/map"); 
	ros::param::param<std::string>("/odom_topic", odom_topic, "/odom"); 
	ros::Subscriber mapsub= nh.subscribe(map_topic, 1, mapCallBack);	
	ros::Subscriber odomsub= nh.subscribe(odom_topic, 1, odomCallBack);
	ros::Publisher grid_pub = nh.advertise<visualization_msgs::Marker>("/grid_marker", 1);
	ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
	ros::Publisher localmkpub = nh.advertise<visualization_msgs::Marker>("local_shapes", 10);
	ros::Publisher globalmkpub = nh.advertise<visualization_msgs::Marker>("global_shapes", 10);	
	ros::Publisher testpub = nh.advertise<visualization_msgs::Marker>("test_shapes", 10);
	ros::Rate rate(100); // run at 100Hz
	
	// wait until map is received
	while (mapData.header.seq<1 or mapData.data.size()<100)  {ros::spinOnce();  rate.sleep();}

	visMarkerInit(tree_g_vis,	mapData.header.frame_id, 5, 0, 0.03, 0, 0.5, 0.5, 0.15);
	visMarkerInit(tree_l_vis,	mapData.header.frame_id, 5, 0, 0.03, 1, 0.1, 0.2, 0.2);

	vector<float> x_start;
	x_start.push_back(0.0); x_start.push_back(0.0);
	V_g.push_back(x_start);
	V_l.push_back(x_start);

	// Main loop
	while (ros::ok())
	{
		// Local Tree
		int det_check = rrtExpand(V_l,tree_l_vis);
		if(det_check == -1) //once detected, publish and reset
		{	
			targetspub.publish(detected_point);
			V_l.clear();
			V_l.push_back(x_bot);
			tree_l_vis.points.clear();
		}
		
		// Global Tree
		rrtExpand(V_g,tree_g_vis);
		targetspub.publish(detected_point);

		//Visualization
		globalmkpub.publish(tree_g_vis);	
		localmkpub.publish(tree_l_vis);

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}