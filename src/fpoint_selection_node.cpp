#include <tdle/common_func.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tdle/fpoint_selection_node.h>

using namespace std;

nav_msgs::Odometry odom;
nav_msgs::OccupancyGrid mapData;
EXP_STATE state = NORMAL;
geometry_msgs::Point map_origin, next_center;
float map_width, map_height;
vector<geometry_msgs::Point> filtered_points, candidate_fpoints, waypoints;


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

void fpCallBack(const tdle::PointArray& msg)
{
	filtered_points = msg.points;
}

void cand_fpCallBack(const tdle::PointArray& msg)
{
	next_center = msg.points.back();
	candidate_fpoints = msg.points;
	candidate_fpoints.pop_back();
}

// global compatibility 
float gloComp(geometry_msgs::Point fpoint,geometry_msgs::Point next_center)
{
	float cost = max(abs(fpoint.x - next_center.x),abs(fpoint.y - next_center.y))-max(map_height,map_width)/6;
	return cost;
}

// motion consistency
float motionCosist(geometry_msgs::Point fpoint)
{
	geometry_msgs::Point r_position = odom.pose.pose.position;
	geometry_msgs::Quaternion r_orient = odom.pose.pose.orientation;
	float angle = atan2(fpoint.y - r_position.y, fpoint.x - r_position.x);
	float yaw = atan2(
		2 *(r_orient.w * r_orient.z + r_orient.x * r_orient.y),
		1 - 2 * (r_orient.y * r_orient.y + r_orient.z * r_orient.z) );
	float cost = exp(2 * ( 0.637 * abs(angle - yaw) - 1));
	// ROS_INFO_THROTTLE(3,"motion consistency: %f",cost);
	return cost;
}

// info gain: fpoints can be seen in filtered_points at candidate point
float infoGain(geometry_msgs::Point fpoint)
{
	float cost = 0;
	for(int i=0; i<filtered_points.size(); i++)
	{
		if(abs(filtered_points[i].x - fpoint.x) < 1 && abs(filtered_points[i].y - fpoint.y) < 1)
			cost += 1;
	}
	return cost;
}

vector<float> zscoreNor(vector<float> data)
{	
	float sum = 0;
	for(int i=0; i<data.size(); i++)
		sum += data[i];
	float mean = sum / data.size();
	float std = 0;
	for(int i=0; i<data.size(); i++)
		std += pow(data[i] - mean,2);
	std = sqrt(std / data.size());
	vector<float> norm_data;
	for(int i=0; i<data.size(); i++)
	{
		if(std == 0)
			norm_data.push_back(0.0);
		else
			norm_data.push_back((data[i] - mean) / std);
	}
	return norm_data;
}

geometry_msgs::Point getTarget()
{
    geometry_msgs::Point target_point;
    float best_score = -100;

    vector<float> gloComp_vec, motionCosist_vec, infoGain_vec;
    for (const auto& point : candidate_fpoints)
    {
        gloComp_vec.push_back(gloComp(point, next_center));
        motionCosist_vec.push_back(motionCosist(point));
        infoGain_vec.push_back(infoGain(point));
    }
	
    vector<float> norm_gloComp = zscoreNor(gloComp_vec);
    vector<float> norm_infoGain = zscoreNor(infoGain_vec);

    for (int i = 0; i < candidate_fpoints.size(); i++)
    {
        float score = 0.5 * norm_gloComp[i] + 0.5 * norm_infoGain[i] - motionCosist_vec[i];
        if (score > best_score)
        {
            best_score = score;
            target_point = candidate_fpoints[i];
        }
    }

    return target_point;
}

void stateCheck()
{
	// Only works with DWA Local Planner currently (TEB Local Planner not supported)
	waypoints.push_back(odom.pose.pose.position);
	int count = 0;
	ROS_WARN_THROTTLE(3,"check state");
	for(int i=0; i<waypoints.size(); i++)
	{
		if(abs(waypoints[i].x - odom.pose.pose.position.x) < 0.1 
			&& abs(waypoints[i].y - odom.pose.pose.position.y) < 0.1)
			count++;
	}
	if (waypoints.size()>15 && count > 5)
	{
		if(candidate_fpoints.size() == 0)
		{
			ROS_WARN_THROTTLE(3,"No candidate fpoint, return to origin");
			state = FINISHED;
		}
		else
		{
			for(int i=0; i<waypoints.size(); i++)
				ROS_INFO_THROTTLE(3,"waypoints: %f, %f",waypoints[i].x,waypoints[i].y);
			ROS_WARN_THROTTLE(3,"No local plan, stucked");
			state = STUCKED;
		}
	}
	else
	{
		state = NORMAL;
		ROS_WARN_THROTTLE(3,"running normally");
	}
		
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "FPonit_Assigner");
	ros::NodeHandle nh;

	string map_topic, odom_topic,local_plan;

	ros::param::param<std::string>("/map_topic", map_topic, "/map"); 
	ros::param::param<std::string>("/odom_topic", odom_topic, "/odom");
	ros::param::param<std::string>("/local_plan", local_plan, "/move_base/TebLocalPlannerROS/local_plan");
	ros::Subscriber mapsub = nh.subscribe(map_topic, 10, mapCallBack);
	ros::Subscriber odomsub = nh.subscribe(odom_topic, 10, odomCallBack);
	ros::Subscriber fpoint_sub = nh.subscribe("/filtered_points", 10, fpCallBack);
	ros::Subscriber cand_fp_sub = nh.subscribe("/candidate_fpoint", 10, cand_fpCallBack);
	// ros::Timer state_timer = nh.createTimer(ros::Duration(0.5), boost::bind(stateCheck));

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	ros::Rate rate(10);
	
  	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){ROS_WARN("Waiting move_base action server");}
	
	// wait until map is received
	while (mapData.header.seq<1 or mapData.data.size()<100)  {ros::spinOnce();  rate.sleep();}

	// Main loop
	while (ros::ok())
	{	
		geometry_msgs::Point target_point;
		if(state == FINISHED)
			target_point = waypoints[0];
		else if(state == STUCKED)
			target_point = waypoints[waypoints.size()-15];
		else
			target_point = getTarget();
		
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = target_point.x;
		goal.target_pose.pose.position.y = target_point.y;
		goal.target_pose.pose.orientation.w = 1.0;
		
		ac.sendGoal(goal);

		// ROS_WARN("heading to subregion %d", goal.target_pose);	

		ros::spinOnce();
		ros::Duration(1).sleep();
	}

	return 0;
}