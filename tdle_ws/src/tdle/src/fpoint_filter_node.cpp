#include <tdle/common_func.h>
#include <tdle/msg/point_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

using namespace std;

namespace tdle
{

// Spatial index for efficient point lookup
class SpatialIndex
{
public:
  void build(const vector<geometry_msgs::msg::Point> &points)
  {
    points_ = points;
  }

  bool hasNeighborWithin(const geometry_msgs::msg::Point &point, float radius) const
  {
    float radius_sq = radius * radius;
    for (const auto &p : points_) {
      float dx = point.x - p.x;
      float dy = point.y - p.y;
      if (dx * dx + dy * dy < radius_sq) {
        return true;
      }
    }
    return false;
  }

  void clear() { points_.clear(); }

private:
  vector<geometry_msgs::msg::Point> points_;
};

class FPointFilterNode : public rclcpp::Node
{
public:
  explicit FPointFilterNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("fpoint_filter",
             rclcpp::NodeOptions(options).parameter_overrides({{"use_sim_time", true}}))
  {
    // Declare and get parameters
    declare_parameter<string>("map_topic", "/map");
    declare_parameter<string>("costmap_topic", "/global_costmap/costmap");
    declare_parameter<int>("max_frontier_points", 200);
    declare_parameter<int>("costmap_threshold", 30);
    declare_parameter<float>("min_point_distance", 0.5f);
    declare_parameter<int>("unknown_threshold", 5);
    declare_parameter<float>("publish_rate", 10.0f);

    get_parameter("map_topic", map_topic_);
    get_parameter("costmap_topic", costmap_topic_);
    get_parameter("max_frontier_points", params_.max_frontier_points);
    get_parameter("costmap_threshold", params_.costmap_threshold);
    get_parameter("min_point_distance", params_.min_point_distance);
    get_parameter("unknown_threshold", params_.unknown_threshold);
    get_parameter("publish_rate", params_.publish_rate);

    // Reserve capacity to avoid reallocations
    frontier_points_.reserve(params_.max_frontier_points);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(10),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          map_data_ = msg;
          map_width_ = msg->info.width * msg->info.resolution;
          map_height_ = msg->info.height * msg->info.resolution;
          map_origin_ = msg->info.origin.position;
        });

    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic_, rclcpp::QoS(10),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          costmap_data_ = msg;
        });

    fpoint_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/detected_points", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          // Check if queue is full
          if (frontier_points_.size() >= static_cast<size_t>(params_.max_frontier_points)) {
            // Remove oldest point (FIFO)
            frontier_points_.erase(frontier_points_.begin());
          }

          // Distance filter - check if point is far enough from existing points
          if (!spatial_index_.hasNeighborWithin(msg->point, params_.min_point_distance)) {
            frontier_points_.push_back(msg->point);
            spatial_index_.build(frontier_points_);
          }
        });

    // Create publishers
    filtered_points_pub_ = create_publisher<tdle::msg::PointArray>("/filtered_points", 10);

    // Visualization with transient_local QoS
    rclcpp::QoS vis_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    vis_qos.transient_local().keep_last(1);
    vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("/candidate_points", vis_qos);

    // Initialize visualization
    string frame_id = "map";
    if (map_data_) {
      frame_id = map_data_->header.frame_id;
    }
    visMarkerInit(frontier_vis_, frame_id, 8, 0, 0.2, 29.0 / 255.0, 111.0 / 255.0,
                  210.0 / 255.0, 0.5);

    // Create timers
    int period_ms = static_cast<int>(1000.0 / params_.publish_rate);
    filter_timer_ = create_wall_timer(
        chrono::milliseconds(period_ms),
        [this]() {
          if (!map_data_ || !costmap_data_) return;
          filterLoop();
        });

    // Visualization at 10Hz
    vis_timer_ = create_wall_timer(
        chrono::milliseconds(100),
        [this]() {
          if (!map_data_) return;
          visualizationLoop();
        });

    RCLCPP_INFO(get_logger(), "FPointFilter initialized with max_points=%d, min_distance=%.2f",
                params_.max_frontier_points, params_.min_point_distance);
  }

private:
  struct FilterParams {
    int max_frontier_points = 200;
    int costmap_threshold = 30;
    float min_point_distance = 0.5f;
    int unknown_threshold = 5;
    float publish_rate = 10.0f;
  } params_;

  string map_topic_;
  string costmap_topic_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_data_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
  geometry_msgs::msg::Point map_origin_;
  float map_width_{0.0f};
  float map_height_{0.0f};

  vector<geometry_msgs::msg::Point> frontier_points_;
  SpatialIndex spatial_index_;

  // Visualization
  visualization_msgs::msg::Marker frontier_vis_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr fpoint_sub_;
  rclcpp::Publisher<tdle::msg::PointArray>::SharedPtr filtered_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_pub_;
  rclcpp::TimerBase::SharedPtr filter_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;

  bool isInBounds(const nav_msgs::msg::OccupancyGrid &grid, float x, float y) const
  {
    float resolution = grid.info.resolution;
    float origin_x = grid.info.origin.position.x;
    float origin_y = grid.info.origin.position.y;
    int width = grid.info.width;
    int height = grid.info.height;

    int grid_x = static_cast<int>((x - origin_x) / resolution);
    int grid_y = static_cast<int>((y - origin_y) / resolution);

    return grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height;
  }

  int getGridValue(const nav_msgs::msg::OccupancyGrid &grid, float x, float y) const
  {
    if (!isInBounds(grid, x, y)) {
      return -1; // Unknown
    }

    float resolution = grid.info.resolution;
    float origin_x = grid.info.origin.position.x;
    float origin_y = grid.info.origin.position.y;
    int width = grid.info.width;

    int grid_x = static_cast<int>((x - origin_x) / resolution);
    int grid_y = static_cast<int>((y - origin_y) / resolution);

    return grid.data[grid_y * width + grid_x];
  }

  bool isCostmapValid(const geometry_msgs::msg::Point &point) const
  {
    if (!costmap_data_)
      return true;

    int value = getGridValue(*costmap_data_, point.x, point.y);
    return value <= params_.costmap_threshold;
  }

  bool hasEnoughUnknown(const geometry_msgs::msg::Point &point) const
  {
    if (!map_data_)
      return true;

    float resolution = map_data_->info.resolution;
    int unknown_count = 0;

    // Check 5x5 grid around point
    for (int dy = -2; dy <= 2; dy++) {
      for (int dx = -2; dx <= 2; dx++) {
        float check_x = point.x + dx * 2 * resolution;
        float check_y = point.y + dy * 2 * resolution;

        if (getGridValue(*map_data_, check_x, check_y) == -1) {
          unknown_count++;
        }
      }
    }

    return unknown_count >= params_.unknown_threshold;
  }

  void removeInvalidPoints()
  {
    // Filter out invalid points in-place
    size_t write_idx = 0;
    for (size_t read_idx = 0; read_idx < frontier_points_.size(); read_idx++) {
      const auto &point = frontier_points_[read_idx];

      // Check costmap
      if (!isCostmapValid(point)) {
        continue;
      }

      // Check unknown neighbors
      if (!hasEnoughUnknown(point)) {
        continue;
      }

      // Keep valid point
      frontier_points_[write_idx] = point;
      write_idx++;
    }

    frontier_points_.resize(write_idx);

    // Rebuild spatial index if points were removed
    if (write_idx < frontier_points_.size()) {
      spatial_index_.build(frontier_points_);
    }
  }

  void filterLoop()
  {
    // Remove invalid points
    removeInvalidPoints();

    // Create output message
    tdle::msg::PointArray candi_points;
    candi_points.points = frontier_points_;

    // Update visualization
    frontier_vis_.points.clear();
    frontier_vis_.points.insert(frontier_vis_.points.end(),
                                candi_points.points.begin(),
                                candi_points.points.end());

    // Publish
    filtered_points_pub_->publish(candi_points);
  }

  void visualizationLoop()
  {
    frontier_vis_.header.stamp = now();
    vis_pub_->publish(frontier_vis_);
  }
};

} // namespace tdle

// Main function - use SingleThreadedExecutor
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tdle::FPointFilterNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
