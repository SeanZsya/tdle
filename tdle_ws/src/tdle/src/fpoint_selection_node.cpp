#include <tdle/common_func.h>
#include <tdle/msg/point_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <deque>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>

using namespace std;
using namespace std::chrono_literals;

namespace tdle
{

enum class ExpState {
  NORMAL = 0,
  STUCKED = 1,
  FINISHED = 2
};

class FPointSelectionNode : public rclcpp::Node
{
public:
  explicit FPointSelectionNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("fpoint_selection",
             rclcpp::NodeOptions(options).parameter_overrides({{"use_sim_time", true}})),
        state_(ExpState::NORMAL),
        goal_active_(false)
  {
    // Declare and get parameters
    declare_parameter<string>("map_topic", "/map");
    declare_parameter<string>("odom_topic", "/odom");
    declare_parameter<string>("local_plan", "/local_plan");
    declare_parameter<float>("goal_send_interval_ms", 700.0f);
    declare_parameter<int>("stuck_waypoint_threshold", 15);
    declare_parameter<float>("position_tolerance", 0.1f);
    declare_parameter<float>("goal_reached_tolerance", 0.3f);
    declare_parameter<float>("global_compatibility_weight", 0.5f);
    declare_parameter<float>("info_gain_weight", 0.5f);
    declare_parameter<float>("motion_consistency_weight", 1.0f);
    declare_parameter<float>("distance_to_robot_weight", 0.7f);
    declare_parameter<float>("loop_rate", 10.0f);

    get_parameter("map_topic", map_topic_);
    get_parameter("odom_topic", odom_topic_);
    get_parameter("local_plan", local_plan_topic_);
    get_parameter("goal_send_interval_ms", params_.goal_send_interval_ms);
    get_parameter("stuck_waypoint_threshold", params_.stuck_waypoint_threshold);
    get_parameter("position_tolerance", params_.position_tolerance);
    get_parameter("goal_reached_tolerance", params_.goal_reached_tolerance);
    get_parameter("global_compatibility_weight", params_.global_compatibility_weight);
    get_parameter("info_gain_weight", params_.info_gain_weight);
    get_parameter("motion_consistency_weight", params_.motion_consistency_weight);
    get_parameter("distance_to_robot_weight", params_.distance_to_robot_weight);
    get_parameter("loop_rate", params_.loop_rate);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(10),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          map_data_ = msg;
          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "[Map] size=%ux%u cells, resolution=%.3f, origin=(%.2f, %.2f)",
              msg->info.width, msg->info.height, msg->info.resolution,
              msg->info.origin.position.x, msg->info.origin.position.y);
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_ = msg;

          // Update waypoints for stuck detection
          waypoints_.push_back(msg->pose.pose.position);
          while (waypoints_.size() > static_cast<size_t>(params_.stuck_waypoint_threshold)) {
            waypoints_.pop_front();
          }
        });

    fp_sub_ = create_subscription<tdle::msg::PointArray>(
        "/filtered_points", rclcpp::QoS(10),
        [this](const tdle::msg::PointArray::SharedPtr msg) {
          filtered_points_ = msg->points;
          (void)msg;
        });

    cand_fp_sub_ = create_subscription<tdle::msg::PointArray>(
        "/candidate_fpoint", rclcpp::QoS(10),
        [this](const tdle::msg::PointArray::SharedPtr msg) {
          if (msg->points.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "candidate_fpoint is empty");
            candidate_fpoints_.clear();
            return;
          }

          // Last point is next center
          next_center_ = msg->points.back();
          candidate_fpoints_ = msg->points;

          // Don't pop if only one point - keep it as candidate
          if (candidate_fpoints_.size() > 1) {
            candidate_fpoints_.pop_back();
          }

          RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "[Receive] candidate_fpoint raw=%zu usable=%zu",
              msg->points.size(), candidate_fpoints_.size());
        });

    // Create action client
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");

    RCLCPP_INFO(get_logger(), "FPointSelection initialized, waiting for action server...");

    // Wait for action server in a timer instead of blocking
    init_timer_ = create_wall_timer(
        100ms,
        [this]() {
          if (nav_client_->wait_for_action_server(0s)) {
            RCLCPP_INFO(get_logger(), "Connected to navigate_to_pose action server");
            init_timer_->cancel();

            // Create main selection timer
            int period_ms = static_cast<int>(1000.0 / params_.loop_rate);
            selection_timer_ = create_wall_timer(
                chrono::milliseconds(period_ms),
                [this]() {
                  if (!map_data_) return;
                  selectionLoop();
                });
          }
        });
  }

private:
  struct SelectionParams {
    float goal_send_interval_ms = 700.0f;
    int stuck_waypoint_threshold = 15;
    float position_tolerance = 0.1f;
    float goal_reached_tolerance = 0.3f;
    float global_compatibility_weight = 0.5f;
    float info_gain_weight = 0.5f;
    float motion_consistency_weight = 1.0f;
    float distance_to_robot_weight = 0.7f;
    float loop_rate = 10.0f;
  } params_;

  string map_topic_;
  string odom_topic_;
  string local_plan_topic_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_data_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  vector<geometry_msgs::msg::Point> filtered_points_;
  vector<geometry_msgs::msg::Point> candidate_fpoints_;
  geometry_msgs::msg::Point next_center_;
  geometry_msgs::msg::Point last_sent_target_;
  bool has_last_sent_target_{false};
  rclcpp::Time last_goal_send_time_{0, 0, RCL_ROS_TIME};

  // Waypoint history for stuck detection
  deque<geometry_msgs::msg::Point> waypoints_;

  // Exploration state
  ExpState state_;
  bool goal_active_;

  // Action client
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tdle::msg::PointArray>::SharedPtr fp_sub_;
  rclcpp::Subscription<tdle::msg::PointArray>::SharedPtr cand_fp_sub_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr selection_timer_;

  string formatPoint(const geometry_msgs::msg::Point &point) const
  {
    ostringstream oss;
    oss << fixed << setprecision(2)
        << "(" << point.x << ", " << point.y << ", " << point.z << ")";
    return oss.str();
  }

  double distanceBetween(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) const
  {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
  }

  bool isNearPoint(const geometry_msgs::msg::Point &target, double tolerance) const
  {
    if (!odom_) {
      return false;
    }
    return distanceBetween(odom_->pose.pose.position, target) <= tolerance;
  }

  bool containsNearbyPoint(
      const vector<geometry_msgs::msg::Point> &points,
      const geometry_msgs::msg::Point &target,
      double tolerance = 0.05) const
  {
    for (const auto &point : points) {
      if (distanceBetween(point, target) <= tolerance) {
        return true;
      }
    }
    return false;
  }

  float calculateGlobalCompatibility(const geometry_msgs::msg::Point &fpoint) const
  {
    float map_size = 10.0f; // Default
    if (map_data_) {
      map_size = max(map_data_->info.width * map_data_->info.resolution,
                     map_data_->info.height * map_data_->info.resolution);
    }

    float cost = max(abs(fpoint.x - next_center_.x), abs(fpoint.y - next_center_.y)) -
                 map_size / 6.0f;
    return cost;
  }

  float calculateMotionConsistency(const geometry_msgs::msg::Point &fpoint) const
  {
    if (!odom_)
      return 0.0f;

    auto r_position = odom_->pose.pose.position;
    auto r_orient = odom_->pose.pose.orientation;

    float angle = atan2(fpoint.y - r_position.y, fpoint.x - r_position.x);
    float yaw = atan2(2 * (r_orient.w * r_orient.z + r_orient.x * r_orient.y),
                      1 - 2 * (r_orient.y * r_orient.y + r_orient.z * r_orient.z));

    float cost = exp(2 * (0.637 * abs(angle - yaw) - 1));
    return cost;
  }

  float calculateInfoGain(const geometry_msgs::msg::Point &fpoint) const
  {
    float cost = 0.0f;
    for (const auto &fp : filtered_points_) {
      if (abs(fp.x - fpoint.x) < 1.0f && abs(fp.y - fpoint.y) < 1.0f) {
        cost += 1.0f;
      }
    }
    return cost;
  }

  float calculateDistanceToRobot(const geometry_msgs::msg::Point &fpoint) const
  {
    if (!odom_) {
      return 0.0f;
    }
    return static_cast<float>(distanceBetween(odom_->pose.pose.position, fpoint));
  }

  vector<float> zscoreNormalize(const vector<float> &data) const
  {
    if (data.empty())
      return {};

    float sum = 0.0f;
    for (float val : data)
      sum += val;
    float mean = sum / data.size();

    float std_dev = 0.0f;
    for (float val : data)
      std_dev += pow(val - mean, 2);
    std_dev = sqrt(std_dev / data.size());

    vector<float> norm_data;
    norm_data.reserve(data.size());

    for (float val : data) {
      if (std_dev == 0)
        norm_data.push_back(0.0f);
      else
        norm_data.push_back((val - mean) / std_dev);
    }

    return norm_data;
  }

  geometry_msgs::msg::Point selectTarget()
  {
    vector<geometry_msgs::msg::Point> effective_candidates;
    effective_candidates.reserve(candidate_fpoints_.size());
    for (const auto &candidate : candidate_fpoints_) {
      if (!containsNearbyPoint(filtered_points_, candidate)) {
        continue;
      }
      if (isNearPoint(candidate, params_.goal_reached_tolerance)) {
        continue;
      }
      effective_candidates.push_back(candidate);
    }

    if (effective_candidates.empty()) {
      // Return to origin or last waypoint
      if (!waypoints_.empty()) {
        if (state_ == ExpState::FINISHED) {
          return waypoints_.front(); // Return to origin
        } else if (state_ == ExpState::STUCKED && waypoints_.size() > 15) {
          auto it = waypoints_.begin();
          advance(it, waypoints_.size() - 15);
          return *it;
        }
      }
      return geometry_msgs::msg::Point(); // Empty point
    }

    // Calculate scores for all candidates
    vector<float> global_compat, motion_consist, info_gain, robot_distance;
    global_compat.reserve(effective_candidates.size());
    motion_consist.reserve(effective_candidates.size());
    info_gain.reserve(effective_candidates.size());
    robot_distance.reserve(effective_candidates.size());

    for (const auto &point : effective_candidates) {
      global_compat.push_back(calculateGlobalCompatibility(point));
      motion_consist.push_back(calculateMotionConsistency(point));
      info_gain.push_back(calculateInfoGain(point));
      robot_distance.push_back(calculateDistanceToRobot(point));
    }

    // Normalize
    vector<float> norm_global = zscoreNormalize(global_compat);
    vector<float> norm_info = zscoreNormalize(info_gain);
    vector<float> norm_distance = zscoreNormalize(robot_distance);

    // Find best score
    float best_score = -numeric_limits<float>::max();
    float best_distance = numeric_limits<float>::max();
    geometry_msgs::msg::Point best_point;
    const float score_eps = 1e-4f;

    for (size_t i = 0; i < effective_candidates.size(); i++) {
      float score = params_.global_compatibility_weight * norm_global[i] +
                    params_.info_gain_weight * norm_info[i] -
                    params_.motion_consistency_weight * motion_consist[i] -
                    params_.distance_to_robot_weight * norm_distance[i];

      if (score > best_score + score_eps ||
          (abs(score - best_score) <= score_eps && robot_distance[i] < best_distance)) {
        best_score = score;
        best_distance = robot_distance[i];
        best_point = effective_candidates[i];
      }
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[Select] best_target=%s best_score=%.3f candidate_count=%zu",
        formatPoint(best_point).c_str(), best_score, effective_candidates.size());
    return best_point;
  }

  bool isStuck() const
  {
    if (waypoints_.size() < static_cast<size_t>(params_.stuck_waypoint_threshold) || !odom_)
      return false;

    auto current_pos = odom_->pose.pose.position;
    int count = 0;

    for (const auto &wp : waypoints_) {
      if (abs(wp.x - current_pos.x) < params_.position_tolerance &&
          abs(wp.y - current_pos.y) < params_.position_tolerance) {
        count++;
      }
    }

    return count > 5;
  }

  bool isFinished() const
  {
    return candidate_fpoints_.empty() && waypoints_.size() > 5;
  }

  geometry_msgs::msg::Pose createGoalPose(const geometry_msgs::msg::Point &target) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position = target;
    pose.orientation.w = 1.0; // Default orientation
    return pose;
  }

  void cancelCurrentGoal()
  {
    if (goal_active_) {
      goal_active_ = false;
    }
  }

  void sendNavigationGoal(const geometry_msgs::msg::Point &target)
  {
    // Create goal
    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose = createGoalPose(target);

    // Create options with result callback
    auto options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    options.result_callback = [this](const auto &result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(this->get_logger(), "Navigation aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "Navigation canceled");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Navigation failed with unknown result");
          break;
      }
      goal_active_ = false;
    };

    double dist_to_robot = -1.0;
    bool same_as_last = false;
    if (odom_) {
      dist_to_robot = distanceBetween(odom_->pose.pose.position, target);
    }
    if (has_last_sent_target_) {
      same_as_last = distanceBetween(last_sent_target_, target) < 1e-3;
    }

    RCLCPP_INFO(
        get_logger(),
        "[SendGoal] target=%s dist_to_robot=%.3f same_as_last=%s",
        formatPoint(target).c_str(), dist_to_robot,
        same_as_last ? "true" : "false");

    goal_active_ = true;
    last_sent_target_ = target;
    has_last_sent_target_ = true;
    last_goal_send_time_ = now();

    // Send goal
    nav_client_->async_send_goal(goal, options);
  }

  void selectionLoop()
  {
    geometry_msgs::msg::Point robot_pos;
    if (odom_) {
      robot_pos = odom_->pose.pose.position;
    }

    // Check state
    if (isStuck()) {
      if (state_ != ExpState::STUCKED) {
        RCLCPP_WARN(get_logger(), "Robot appears stuck!");
        state_ = ExpState::STUCKED;
      }
    } else if (isFinished()) {
      if (state_ != ExpState::FINISHED) {
        RCLCPP_INFO(get_logger(), "Exploration finished!");
        state_ = ExpState::FINISHED;
      }
    } else {
      state_ = ExpState::NORMAL;
    }

    // Select target
    (void)robot_pos;
    auto target = selectTarget();

    // Check for empty target before sending navigation goal
    if (target.x == 0.0f && target.y == 0.0f && target.z == 0.0f) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "No valid target point available, skipping navigation goal");
      return;
    }

    double elapsed_since_send_ms = std::numeric_limits<double>::infinity();
    if (last_goal_send_time_.nanoseconds() > 0) {
      elapsed_since_send_ms = (now() - last_goal_send_time_).seconds() * 1000.0;
    }

    bool same_as_last = has_last_sent_target_ &&
                        distanceBetween(last_sent_target_, target) < 1e-3;

    if (same_as_last && isNearPoint(target, params_.goal_reached_tolerance)) {
      goal_active_ = false;
      return;
    }

    if (goal_active_ && same_as_last) {
      return;
    }

    if (same_as_last && elapsed_since_send_ms < params_.goal_send_interval_ms) {
      return;
    }

    // Send navigation goal
    sendNavigationGoal(target);
  }
};

} // namespace tdle

// Main function - use SingleThreadedExecutor
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tdle::FPointSelectionNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
