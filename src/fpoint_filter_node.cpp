#include <tdle/common_func.h>
#include <tdle/fpoint_filter_node.h>

using namespace std;

vector<geometry_msgs::Point> frontier_points;
nav_msgs::OccupancyGrid mapData, costmapData;
geometry_msgs::Point map_origin, next_center;
float map_width, map_height;

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	mapData = *msg;
	map_width = msg->info.width * msg->info.resolution;
	map_height = msg->info.height * msg->info.resolution;
	map_origin = msg->info.origin.position;
}

void costmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    costmapData = *msg;
}

void fpointCallBack(const geometry_msgs::PointStamped& msg)
{
    // Only keep 200 fpoints
    if (frontier_points.size() > 200) 
        frontier_points.erase(frontier_points.begin());
    
    // Deploy frontier points filter (MeanShift of disFilter)
    // frontier_points = meanshiftFilter(msg.point, frontier_points, 0.3, 0.1);
    frontier_points = disFilter(msg.point, frontier_points);
}

vector<geometry_msgs::Point> disFilter(geometry_msgs::Point point, vector<geometry_msgs::Point>& fpoints)
{
    //add new point if not close to points in fpoints
    bool is_close = false;
    for (int i = 0; i < fpoints.size(); i++) {
        if (sqrt(pow(point.x - fpoints[i].x, 2) + pow(point.y - fpoints[i].y, 2)) < 0.5) {
            is_close = true;
            break;
        }
    }
    if (!is_close) fpoints.push_back(point);
    
    return fpoints;
}


// vector<geometry_msgs::Point> meanshiftFilter(geometry_msgs::Point point, vector<geometry_msgs::Point> &fpoints, float kernel_bandwidth, float convergence_threshold) {
//     fpoints.push_back(point);
//     vector<geometry_msgs::Point> centroids;
//     for (int i = 0; i < fpoints.size(); i++) {
//         geometry_msgs::Point centroid = fpoints[i];
//         geometry_msgs::Point next_centroid;
//         while (1) {
//             float sum_x = 0.0;
//             float sum_y = 0.0;
//             float sum_w = 0.0;
//             for (int j = 0; j < fpoints.size(); j++) {
//                 float dist = sqrt(pow(centroid.x - fpoints[j].x, 2) + pow(centroid.y - fpoints[j].y, 2));
//                 float weight = exp(-dist * dist / (2 * kernel_bandwidth * kernel_bandwidth));
//                 sum_x += weight * fpoints[j].x;
//                 sum_y += weight * fpoints[j].y;
//                 sum_w += weight;
//             }
//             next_centroid.x = sum_x / sum_w;
//             next_centroid.y = sum_y / sum_w;
//             if (sqrt(pow(centroid.x - next_centroid.x, 2) + pow(centroid.y - next_centroid.y, 2)) < convergence_threshold) {
//                 break;
//             }
//             centroid = next_centroid;
//         }
//         centroids.push_back(centroid);
//     }

//     return centroids;
// }

tdle::PointArray delPrevious(vector<geometry_msgs::Point>& filtered_points, visualization_msgs::Marker& fpoint_vis) 
{
    tdle::PointArray candi_points;
    fpoint_vis.points.clear();
    for (int i = 0; i < filtered_points.size(); i++) {
        geometry_msgs::Point point = filtered_points[i];
        int grid_x = (point.x - map_origin.x) / mapData.info.resolution;
        int grid_y = (point.y - map_origin.y) / mapData.info.resolution;
        
    //filter out point with costmap value more than threshold
        int costmap_value = costmapData.data[grid_y * costmapData.info.width + grid_x];
        if (costmap_value > 30) {
            filtered_points.erase(filtered_points.begin() + i);
            i--;
            continue;
        }

    // filter out point with few unknown grid in circle 
        int unknown_num = 0; 
        for (int j = -2; j <= 2; j++) 
            for (int k = -2; k <= 2; k++)  
                if (mapData.data[(grid_y + 2 * j) * mapData.info.width + grid_x + 2 * k] == -1) unknown_num++; 
        if (unknown_num < 5){
            filtered_points.erase(filtered_points.begin() + i);
            i--;
            continue;
        }

        candi_points.points.push_back(point); 
        fpoint_vis.points.push_back(point); 
    }
    return candi_points;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fpoint_filter");
	ros::NodeHandle nh;

    ros::Subscriber mapsub = nh.subscribe("/map", 10, mapCallBack);	
    ros::Subscriber costmap_sub = nh.subscribe("/move_base/global_costmap/costmap", 1, costmapCallBack);
    ros::Subscriber fpoint_sub = nh.subscribe("/detected_points", 1, fpointCallBack);
    ros::Publisher fpoint_pub = nh.advertise<tdle::PointArray>("/filtered_points", 10);
    ros::Publisher fpoint_vis_pub = nh.advertise<visualization_msgs::Marker>("/candidate_points", 10);
    ros::Rate rate(10);

    visualization_msgs::Marker fpoint_vis;

    float bandwidth = 0.3;
    float conv_thr = 0.1;

    // wait until map is received
	while (mapData.header.seq<1 or mapData.data.size()<100)  
    {
        ROS_WARN_THROTTLE(1,"Waiting for the map"); 
        ros::spinOnce();  rate.sleep();
    }
    while (costmapData.data.size()<100)
    {
        ROS_WARN_THROTTLE(1,"Waiting for the global cost map");
        ros::spinOnce();  rate.sleep();
    }

    visMarkerInit(fpoint_vis, mapData.header.frame_id, 8, 0, 0.2, 29.0/255.0, 111.0/255.0, 210.0/255.0, 0.5);
    
    while (ros::ok())
	{        
        tdle::PointArray candi_points = delPrevious(frontier_points, fpoint_vis);

        fpoint_pub.publish(candi_points);
        fpoint_vis_pub.publish(fpoint_vis);
        
        ros::spinOnce();
		rate.sleep();
    }

    return 0;
}