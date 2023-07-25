#include <tdle/common_func.h>
#include <tdle/PointArray.h>
#include <tdle/subregion_arrangement_node.h>

using namespace std;

nav_msgs::Odometry odom;
nav_msgs::OccupancyGrid mapData;
geometry_msgs::Point map_origin;
float map_width, map_height;
int sr_bot;
vector<geometry_msgs::Point> filtered_points, center_points;

// CallBack Functions ========================================
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

// Visualization Functions ========================================
void drawSubRegi(vector<int> sr_sorted, ros::Publisher marker_pub)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = mapData.header.frame_id;
  marker.ns = "SubRegion";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  // Set the scale of the marker.
  marker.scale.x = map_width / 3;
  marker.scale.y = map_height / 3;
  marker.scale.z = 0.01;
  marker.lifetime = ros::Duration();

  // Calculate the grid positions.
  double grid_width = map_width / 3;
  double grid_height = map_height / 3;

  geometry_msgs::Point position;
  std_msgs::ColorRGBA color;

  for(int i=0; i<sr_sorted.size(); i++)
  {
    int num_subregion = sr_sorted[i];
    // Calculate the x and y position of the grid.
    position.x = map_origin.x + (num_subregion % 3) * grid_width + grid_width / 2;
    position.y = map_origin.y + (num_subregion / 3) * grid_height + grid_height / 2;
    position.z = 0.0;
    // Add the position to the marker.
    marker.points.push_back(position);

    // float color_factor =  (1.0 + (float)i/sr_sorted.size()) * 0.5;
	float color_factor =  (float)i/sr_sorted.size();
    color.r = 0.2 * log10 ( 10 * color_factor + 1) + 0.1;
    color.g = 0.6 * log10 ( 10 * color_factor + 1) + 0.3;
    color.b = 0.3 * log10 ( 10 * color_factor + 1) + 0.15;
    color.a = 1 + 0.3 * log10 ( 10 * color_factor + 10);
    marker.colors.push_back(color);
  }
  // Publish the marker.
  marker_pub.publish(marker);
}

void drawGrid(ros::Publisher marker_pub)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = mapData.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "grid";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.color.r = 0.4;
  marker.color.g = 1.0;
  marker.color.b = 0.8;
  marker.color.a = 0.6;
  marker.lifetime = ros::Duration();
  // Calculate the grid sizes and positions
  double grid_width = map_width / 3;
  double grid_height = map_height / 3;
  double x = map_origin.x + grid_width;
  double y = map_origin.y + grid_height;
  // Draw the vertical grid lines
  for (int i = 0; i < 2; i++)
  {
    geometry_msgs::Point start_point;
    start_point.x = x;
    start_point.y = map_origin.y;
    start_point.z = 0.0;

    geometry_msgs::Point end_point;
    end_point.x = x;
    end_point.y = map_origin.y + map_height;
    end_point.z = 0.0;

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    x += grid_width;
  }
  // Draw the horizontal grid lines
  for (int i = 0; i < 2; i++)
  {
    geometry_msgs::Point start_point;
    start_point.x = map_origin.x;
    start_point.y = y;
    start_point.z = 0.0;

    geometry_msgs::Point end_point;
    end_point.x = map_origin.x + map_width;
    end_point.y = y;
    end_point.z = 0.0;

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    y += grid_height;
  }

  marker_pub.publish(marker);
}

// Select SubRegions ==============================================
vector<int> selectSubRegi()
{
	//select grids(1-9) with filtered point inside
	vector<int> sr_sel;
	int num_subregion = 9;
	double grid_width = map_width / 3;
	double grid_height = map_height / 3;

	for(int i=0; i<num_subregion; i++)
	{
		bool fpoint_inside = false;
		int fpoint_count = 0;
		double x = map_origin.x + (i % 3) * grid_width + grid_width / 2;
		double y = map_origin.y + (i / 3) * grid_height + grid_height / 2;
		geometry_msgs::Point center;
		center.x = x;
		center.y = y;
		center.z = 0.0;
		//check filtered points inside
		for(int j=0; j<filtered_points.size(); j++)
		{
			if(isInside(center, filtered_points[j])) 
				fpoint_count++;
			if(fpoint_count >= 2) 
			{
				fpoint_inside = true;
				break;
			}
		}
		//unknown or has filtered points inside, add to sr_sorted
		if(isUnknown(center) || fpoint_inside) 
			sr_sel.push_back(i);

		//determine which subregion the robot is in here by the way
		if(isInside(center, odom.pose.pose.position))
			sr_bot = i;
    
		//save center_points by the way
		center_points.push_back(center);
	}
  
	return sr_sel;
}

bool isInside(geometry_msgs::Point center, geometry_msgs::Point point)
{
	double grid_width = map_width / 3;
	double grid_height = map_height / 3;
	double x = center.x;
	double y = center.y;
	double x1 = x - grid_width/2;
	double x2 = x + grid_width/2;
	double y1 = y - grid_height/2;
	double y2 = y + grid_height/2;
	if((point.x > x1) && (point.x < x2) && (point.y > y1) && (point.y < y2))
		return true;
	else
		return false;
}

bool isUnknown(geometry_msgs::Point center)
{
  double grid_width = map_width / 3;
  double grid_height = map_height / 3;
  double x = center.x;
  double y = center.y;
  double x1 = x - grid_width/2;
  double x2 = x + grid_width/2;
  double y1 = y - grid_height/2;
  double y2 = y + grid_height/2;
  double x_step = (x2 - x1) / 10;
  double y_step = (y2 - y1) / 10;
  int unknown_count = 0;
  for(int i=0; i<10; i++)
  {
    for(int j=0; j<10; j++)
    {
      float x = x1 + i * x_step;
      float y = y1 + j * y_step;
      vector<float> grid;
      grid.push_back(x);
      grid.push_back(y);
      int gridvalue = gridValue(mapData, grid);//
      if(gridvalue == -1)
        unknown_count++;
    }
  }
  if(unknown_count > 80)
    return true;
  else
    return false;
}

// Sort SubRegions ==============================================
float route_length(vector<vector<float>> full_graph, vector<int> route)
{
    // ROS_WARN("stage2");
    float length = 0;
    for (int i = 0; i < route.size() - 1; i++) {
        length += full_graph[route[i]][route[i+1]];
    }
    // ROS_WARN("stage3");
    return length;
}

float navi_cost(geometry_msgs::Point next_center, geometry_msgs::Point fpoint)
{	
	// distance to next subregion center
	float cost = sqrt(pow(next_center.x - fpoint.x, 2) + pow(next_center.y - fpoint.y, 2));
	// distance to next subregion edge
	// float cost = max(abs(next_center.x - fpoint.x),abs(next_center.y - fpoint.y)) - max(map_height,map_width)/6;
	return cost;
}

float similarity_dtw(vector<vector<float>> full_graph, vector<int> sr_sel, vector<int> sr_sorted_before)
{
    int n = sr_sel.size();
    int m = sr_sorted_before.size();
    // Initialize the cost matrix with large values
    vector<vector<float>> cost(n, vector<float>(m, 1e9));
    // Initialize the first row and column with 0
    cost[0][0] = full_graph[sr_sel[0]][sr_sorted_before[0]];
    for (int i = 1; i < n; i++) {
        cost[i][0] = cost[i-1][0] + full_graph[sr_sel[i-1]][sr_sorted_before[0]];
    }
    for (int j = 1; j < m; j++) {
        cost[0][j] = cost[0][j-1] + full_graph[sr_sel[0]][sr_sorted_before[j-1]];
    }

    // Fill in the rest of the cost matrix
    for (int i = 1; i < n; i++) {
        for (int j = 1; j < m; j++) {
            // float d = abs(sr_sel[i] - sr_sorted_before[j]);
            float d = full_graph[sr_sel[i-1]][sr_sorted_before[j-1]];
            cost[i][j] = d + min(cost[i-1][j], min(cost[i][j-1], cost[i-1][j-1]));
        }
    }
    // Return the DTW distance as a similarity measure
    float similarity = exp(-cost[n-1][m-1] / (n+m));
    return similarity;
}

vector<int> sortSubRgi_bfs(vector<vector<float>> full_graph,vector<int> sr_sel,vector<int> sr_sorted_before,int sr_bot)
{
    float best_score = -1e5;
    vector<int> best_path;
    sort(sr_sel.begin(), sr_sel.end());
    do{
        float dis_to_ini = abs(odom.pose.pose.position.x - center_points[sr_sel[0]].x) + abs(odom.pose.pose.position.y - center_points[sr_sel[0]].y);
        // float score = 
        //     7 * similarity_dtw(full_graph, sr_sel,sr_sorted_before) - route_length(full_graph, sr_sel)/(sr_sel.size()+1) 
        //     - 4 * full_graph[sr_bot][sr_sel[0]];
        float score = 
            7 * similarity_dtw(full_graph, sr_sel,sr_sorted_before) - route_length(full_graph, sr_sel)/(sr_sel.size()+1) 
            - 2 * dis_to_ini;
        if (score > best_score) {
            best_score = score;
            best_path = sr_sel;
        }
    } while (next_permutation(sr_sel.begin(), sr_sel.end()));
    return best_path;
}

vector<int> sortSubRgi_asa(vector<vector<float>> full_graph,vector<int> sr_sel,vector<int> sr_sorted_before,int sr_bot)
{
    float init_temp = 1e3;
    float final_temp = 0.1;
    float temp = init_temp;
    float eta = 1;
    vector<int> best_path = sr_sel;
    float best_score = -1e3;
    int count = 0;
    int n_ite = 2e4;
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0, 1);
    while (temp > 1e5 && count < n_ite) 
    {
        //generate new path
        vector<int> new_path = sr_sel;
        int index1 = round(dis(gen) * (new_path.size()-1));
        int index2 = round(dis(gen) * (new_path.size()-1));
        swap(new_path[index1], new_path[index2]);

        //calculate score difference
        float score = 
            6 * similarity_dtw(full_graph, new_path,sr_sorted_before) 
            - route_length(full_graph, new_path)/(new_path.size()+1) 
            - 4 * full_graph[sr_bot][new_path[0]];
        float delta = score - best_score;
        // update best score and path
        if (delta > 0) {
            best_score = score;
            best_path = new_path;
        } else {
            float prob = exp(delta / temp);
            if (prob > (float)dis(gen)) {
                best_score = score;
                best_path = new_path;
            }
        }
        eta = min( exp(0.002 * ( float(count)/n_ite -1 )), 0.999 );
        temp *= eta;
        count++;
    }
    return best_path;
}

// Classify Frontier Points ==============================================
vector<vector<geometry_msgs::Point>> classifyFpoints(vector<int> sr_sorted)
{
	vector<vector<geometry_msgs::Point>> fpoint_sorted;
	double grid_width = map_width / 3;
	double grid_height = map_height / 3;

	for(int i=0; i<sr_sorted.size(); i++)
	{
		vector<geometry_msgs::Point> fpoints_in_grid;
		int grid_num = sr_sorted[i];
		double x = map_origin.x + (grid_num % 3) * grid_width + grid_width / 2;
		double y = map_origin.y + (grid_num / 3) * grid_height + grid_height / 2;
		geometry_msgs::Point center;
		center.x = x;
		center.y = y;
		center.z = 0.0;
		for(int j=0; j<filtered_points.size(); j++)
		{
			if(isInside(center, filtered_points[j])) 
				fpoints_in_grid.push_back(filtered_points[j]);
		}
		if(fpoints_in_grid.size() == 0)
			fpoints_in_grid = {};
		fpoint_sorted.push_back(fpoints_in_grid);
	}

	return fpoint_sorted;
}

// Main Function ==========================================================
int main(int argc, char **argv)
{
	ros::init(argc, argv, "subregion_arrangement");
	ros::NodeHandle nh;

	string map_topic, odom_topic;

	ros::param::param<std::string>("/map_topic", map_topic, "/map"); 
	ros::param::param<std::string>("/odom_topic", odom_topic, "/odom"); 
	ros::Subscriber mapsub = nh.subscribe(map_topic, 10, mapCallBack);	
	ros::Subscriber odomsub = nh.subscribe(odom_topic, 10, odomCallBack);
	ros::Subscriber fpoint_sub = nh.subscribe("/filtered_points", 10, fpCallBack);
	ros::Publisher subregion_pub = nh.advertise<visualization_msgs::Marker>("/subregion_marker", 10);
	ros::Publisher grid_pub = nh.advertise<visualization_msgs::Marker>("/grid_marker", 10);
	ros::Publisher cand_fp_pub = nh.advertise<tdle::PointArray>("/candidate_fpoint", 10);

	// ros::Timer updatesr_timer_ = nh.createTimer(ros::Duration(1), updatesrCallback);

	ros::Rate rate(10); 
	
	// wait until map is received
	while (mapData.header.seq<1 or mapData.data.size()<100)  {ros::spinOnce();  rate.sleep();}

	// initialize the subregion distance matrix
	vector<vector<float>> full_graph;
    for(int i=0;i<9;i++){
        vector<float> one_line;
        for(int j=0;j<9;j++){
            float dx = pow((i % 3 - j % 3),2);
            float dy = pow((i / 3 - j / 3),2);
            one_line.push_back(dx + dy);
        }
        full_graph.push_back(one_line);
    } 

	vector<int> sr_sorted_before(3);

	// Main loop
	while (ros::ok())
	{	
        ros::Time start = ros::Time::now();

        vector<int> sr_sel = selectSubRegi();
        
        vector<int> sr_sorted;
        if(sr_sel.size()<7)
            sr_sorted = sortSubRgi_bfs(full_graph,sr_sel,sr_sorted_before,sr_bot);
        else
            sr_sorted = sortSubRgi_asa(full_graph,sr_sel,sr_sorted_before,sr_bot);
        sr_sorted_before = sr_sorted;
        
        vector<vector<geometry_msgs::Point>> fpoint_sorted = classifyFpoints(sr_sorted);
        
        ros::Duration duration = ros::Time::now() - start;
        ROS_INFO_THROTTLE(3, "Time for subregion arrangement: %f", duration.toSec());

        // publish frontier point in the first subregion
        for(int i=0; i<fpoint_sorted.size(); i++)
        {
            if(fpoint_sorted[i].size() == 0) continue;
            
            if(i<fpoint_sorted.size()-1)
                fpoint_sorted[i].push_back(center_points[sr_sorted[i+1]]);

            tdle::PointArray cand_fp;
            cand_fp.points = fpoint_sorted[i];
            cand_fp_pub.publish(cand_fp);
            break;
        }

        drawGrid(grid_pub);
        drawSubRegi(sr_sorted, subregion_pub);

        ros::spinOnce();
        rate.sleep();
	}

	return 0;
}