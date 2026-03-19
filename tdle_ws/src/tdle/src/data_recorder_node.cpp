#include <tdle/common_func.h>
#include <tdle/data_recorder_node.h>
#include <cstring>

using namespace std;

nav_msgs::msg::Odometry odom;
nav_msgs::msg::OccupancyGrid mapData;
geometry_msgs::msg::Point map_origin, next_center;
float map_width, map_height;

void mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  mapData = *msg;
  map_width = msg->info.width * msg->info.resolution;
  map_height = msg->info.height * msg->info.resolution;
  map_origin = msg->info.origin.position;
}

void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom = *msg;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("data_recorder");

  string map_topic, odom_topic, output_dir;

  node->declare_parameter<string>("map_topic", "/map");
  node->declare_parameter<string>("odom_topic", "/odom");
  node->declare_parameter<string>("output_dir", "/home/sean/tdle_data");
  if (!node->has_parameter("use_sim_time")) {
    node->declare_parameter<bool>("use_sim_time", false);
  }
  node->get_parameter("map_topic", map_topic);
  node->get_parameter("odom_topic", odom_topic);
  node->get_parameter("output_dir", output_dir);

  auto mapsub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(10), mapCallBack);
  auto odomsub = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10), odomCallBack);

  rclcpp::Rate rate(1); // run at 1Hz

  // wait until map is received
  while (rclcpp::ok() && mapData.data.size() < 1) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  // Get current time
  time_t currentTime;
  time(&currentTime);

  // Convert current time to string
  char *timeString = ctime(&currentTime);
  timeString[strlen(timeString) - 1] = '\0';
  // Create filename with current time
  string filename = output_dir + "/" + string(timeString) + ".txt";

  // save robot position and map size to a txt file, name with current time
  ofstream outputFile(filename);

  // Main loop
  while (rclcpp::ok()) {
    outputFile << odom.pose.pose.position.x << " " << odom.pose.pose.position.y << " "
               << map_width * map_height << endl;

    rclcpp::spin_some(node);
    rate.sleep();
  }
  outputFile.close();
  rclcpp::shutdown();
  return 0;
}
