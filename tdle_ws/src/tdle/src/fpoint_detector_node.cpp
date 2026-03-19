#include <tdle/common_func.h>
#include <tdle/msg/point_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <deque>

using namespace std;

namespace tdle
{

class FPointDetectorNode : public rclcpp::Node
{
public:
  explicit FPointDetectorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("fpoint_detector",
             rclcpp::NodeOptions(options).parameter_overrides({{"use_sim_time", true}})),
        gen_(rd_())
  {
    // Declare and get parameters
    declare_parameter<float>("stride", 0.5);
    declare_parameter<string>("map_topic", "/map");
    declare_parameter<string>("odom_topic", "/odom");
    declare_parameter<int>("max_tree_nodes", 1000);

    get_parameter("stride", stride_);
    get_parameter("map_topic", map_topic_);
    get_parameter("odom_topic", odom_topic_);
    int max_nodes;
    get_parameter("max_tree_nodes", max_nodes);
    max_tree_nodes_ = static_cast<size_t>(max_nodes);

    // Initialize robot position
    x_bot_ = {0.0f, 0.0f};

    // Initialize trees with starting position
    global_tree_.push_back(x_bot_);
    local_tree_.push_back(x_bot_);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(10),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          map_data_ = msg;
          map_width_ = msg->info.width * msg->info.resolution;
          map_height_ = msg->info.height * msg->info.resolution;
          map_origin_ = msg->info.origin.position;
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_ = msg;
          x_bot_ = {static_cast<float>(msg->pose.pose.position.x),
                    static_cast<float>(msg->pose.pose.position.y)};
        });

    // Create publishers
    detected_points_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/detected_points", 10);

    // Visualization publishers with transient_local QoS
    rclcpp::QoS vis_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    vis_qos.transient_local().keep_last(1);

    global_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("/global_shapes", vis_qos);
    local_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("/local_shapes", vis_qos);

    // Initialize visualization
    initializeVisualization();

    // Create timers
    detection_timer_ = create_wall_timer(
        chrono::milliseconds(7),
        [this]() {
          if (!map_data_) return;
          detectionLoop();
        });

    vis_timer_ = create_wall_timer(
        chrono::milliseconds(33),
        [this]() {
          if (!map_data_) return;
          visualizationLoop();
        });

    RCLCPP_INFO(get_logger(), "FPointDetector initialized with stride=%.2f", stride_);
  }

private:
  // Parameters
  float stride_;
  string map_topic_;
  string odom_topic_;
  size_t max_tree_nodes_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_data_;
  geometry_msgs::msg::Point map_origin_;
  float map_width_{0.0f};
  float map_height_{0.0f};

  nav_msgs::msg::Odometry::SharedPtr odom_;
  vector<float> x_bot_;

  // RRT trees
  deque<vector<float>> global_tree_;
  deque<vector<float>> local_tree_;

  // Random number generator
  random_device rd_;
  mt19937 gen_;

  // Visualization markers
  visualization_msgs::msg::Marker global_tree_vis_;
  visualization_msgs::msg::Marker local_tree_vis_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr detected_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_vis_pub_;
  rclcpp::TimerBase::SharedPtr detection_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;

  void initializeVisualization()
  {
    string frame_id = "map";
    if (map_data_) {
      frame_id = map_data_->header.frame_id;
    }

    // Initialize global tree visualization
    global_tree_vis_.header.frame_id = frame_id;
    global_tree_vis_.type = visualization_msgs::msg::Marker::LINE_LIST;
    global_tree_vis_.action = visualization_msgs::msg::Marker::ADD;
    global_tree_vis_.scale.x = 0.03;
    global_tree_vis_.color.r = 0.0f;
    global_tree_vis_.color.g = 0.5f;
    global_tree_vis_.color.b = 0.5f;
    global_tree_vis_.color.a = 0.15f;

    // Initialize local tree visualization
    local_tree_vis_.header.frame_id = frame_id;
    local_tree_vis_.type = visualization_msgs::msg::Marker::LINE_LIST;
    local_tree_vis_.action = visualization_msgs::msg::Marker::ADD;
    local_tree_vis_.scale.x = 0.03;
    local_tree_vis_.color.r = 1.0f;
    local_tree_vis_.color.g = 0.1f;
    local_tree_vis_.color.b = 0.2f;
    local_tree_vis_.color.a = 0.2f;
  }

  int rrtExpand(deque<vector<float>> &tree_nodes,
                geometry_msgs::msg::PointStamped &detected_point)
  {
    if (!map_data_)
      return 0;

    auto map = map_data_;
    auto origin = map_origin_;
    float width = map_width_;
    float height = map_height_;

    // Generate random point
    uniform_real_distribution<> dis(0, 1);
    float xr = dis(gen_) * width + origin.x;
    float yr = dis(gen_) * height + origin.y;
    vector<float> x_rand = {xr, yr};

    // Find nearest node
    vector<float> x_nearest = Nearest(vector<vector<float>>(tree_nodes.begin(), tree_nodes.end()), x_rand);
    vector<float> x_new = Steer(x_nearest, x_rand, stride_);

    // Check obstacle
    int det_check = ObstacleFree(x_nearest, x_new, *map);

    if (det_check == -1) {
      // Frontier found
      detected_point.header.stamp = now();
      detected_point.header.frame_id = map->header.frame_id;
      detected_point.point.x = x_new[0];
      detected_point.point.y = x_new[1];
      detected_point.point.z = 0.0;
    } else if (det_check == 1) {
      // Free space, add to tree
      tree_nodes.push_back(x_new);

      // Maintain tree size limit
      size_t limit = calculateMaxTreeNodes();
      while (tree_nodes.size() > limit) {
        tree_nodes.pop_front();
      }

      // Update visualization
      if (&tree_nodes == &global_tree_) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = x_new[0];
        p1.y = x_new[1];
        p1.z = 0.0;
        p2.x = x_nearest[0];
        p2.y = x_nearest[1];
        p2.z = 0.0;
        global_tree_vis_.points.push_back(p1);
        global_tree_vis_.points.push_back(p2);

        // Maintain visualization size
        while (global_tree_vis_.points.size() > limit * 2) {
          global_tree_vis_.points.erase(global_tree_vis_.points.begin(),
                                        global_tree_vis_.points.begin() + 2);
        }
      } else {
        geometry_msgs::msg::Point p1, p2;
        p1.x = x_new[0];
        p1.y = x_new[1];
        p1.z = 0.0;
        p2.x = x_nearest[0];
        p2.y = x_nearest[1];
        p2.z = 0.0;
        local_tree_vis_.points.push_back(p1);
        local_tree_vis_.points.push_back(p2);

        while (local_tree_vis_.points.size() > limit * 2) {
          local_tree_vis_.points.erase(local_tree_vis_.points.begin(),
                                        local_tree_vis_.points.begin() + 2);
        }
      }
    }

    return det_check;
  }

  size_t calculateMaxTreeNodes() const
  {
    if (map_width_ <= 0 || map_height_ <= 0 || stride_ <= 0)
      return max_tree_nodes_;
    return std::min(static_cast<size_t>(2 * map_height_ * map_width_ / stride_ / stride_),
                    max_tree_nodes_);
  }

  void detectionLoop()
  {
    geometry_msgs::msg::PointStamped detected_point;

    // Local tree expansion
    int det_check = rrtExpand(local_tree_, detected_point);
    if (det_check == -1) {
      detected_points_pub_->publish(detected_point);

      // Reset local tree to robot position
      local_tree_.clear();
      local_tree_.push_back(x_bot_);
      local_tree_vis_.points.clear();
    }

    // Global tree expansion
    rrtExpand(global_tree_, detected_point);
    if (det_check != -1 && detected_point.point.x != 0.0) {
      detected_points_pub_->publish(detected_point);
    }
  }

  void visualizationLoop()
  {
    // Update timestamps
    global_tree_vis_.header.stamp = now();
    local_tree_vis_.header.stamp = now();

    global_vis_pub_->publish(global_tree_vis_);
    local_vis_pub_->publish(local_tree_vis_);
  }
};

} // namespace tdle

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tdle::FPointDetectorNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
