#include <tdle/common_func.h>
#include <tdle/msg/point_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <limits>

using namespace std;

namespace tdle
{

class SubregionArrangementNode : public rclcpp::Node
{
public:
  explicit SubregionArrangementNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("subregion_arrangement",
             rclcpp::NodeOptions(options).parameter_overrides({{"use_sim_time", true}})),
        gen_(42)
  {
    // Parameter declaration
    declare_parameter<string>("map_topic", "/map");
    declare_parameter<string>("odom_topic", "/odom");
    declare_parameter<float>("max_subregion_size", 5.0f);
    declare_parameter<int>("min_points_per_region", 2);
    declare_parameter<int>("unknown_threshold", 80);
    declare_parameter<float>("unknown_sample_step", 10.0f);
    declare_parameter<float>("publish_rate", 10.0f);
    declare_parameter<float>("ana_initial_temp", 1.0f);
    declare_parameter<float>("ana_stop_temp", 1e-3f);
    declare_parameter<int>("ana_max_iterations", 120);
    declare_parameter<float>("ana_mu", 2.0f);
    declare_parameter<float>("ana_lambda_similarity", 1.0f);
    declare_parameter<float>("ana_lambda_end_to_initial", 0.01f);
    declare_parameter<float>("ana_lambda_path_length", 0.65f);
    declare_parameter<int>("ana_seed", 42);

    get_parameter("map_topic", map_topic_);
    get_parameter("odom_topic", odom_topic_);
    get_parameter("max_subregion_size", params_.max_subregion_size);
    get_parameter("min_points_per_region", params_.min_points_per_region);
    get_parameter("unknown_threshold", params_.unknown_threshold);
    get_parameter("unknown_sample_step", params_.unknown_sample_step);
    get_parameter("publish_rate", params_.publish_rate);
    get_parameter("ana_initial_temp", params_.ana_initial_temp);
    get_parameter("ana_stop_temp", params_.ana_stop_temp);
    get_parameter("ana_max_iterations", params_.ana_max_iterations);
    get_parameter("ana_mu", params_.ana_mu);
    get_parameter("ana_lambda_similarity", params_.ana_lambda_similarity);
    get_parameter("ana_lambda_end_to_initial", params_.ana_lambda_end_to_initial);
    get_parameter("ana_lambda_path_length", params_.ana_lambda_path_length);
    get_parameter("ana_seed", params_.ana_seed);

    gen_.seed(static_cast<uint32_t>(params_.ana_seed));

    // Initialize dynamic divisions (at least 3x3)
    params_.divisions_x = 3;
    params_.divisions_y = 3;

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(10),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          map_data_ = msg;
          map_width_ = msg->info.width * msg->info.resolution;
          map_height_ = msg->info.height * msg->info.resolution;
          map_origin_ = msg->info.origin.position;

          // Recompute divisions from current map size
          int old_divisions_x = params_.divisions_x;
          int old_divisions_y = params_.divisions_y;
          updateDivisions();

          // First map initialization
          static bool initialized = false;
          if (!initialized) {
            initialized = true;
            initializeDistanceGraph();
            initializeVisualization();
            map_initialized_ = true;
            RCLCPP_INFO(
                get_logger(),
                "Map initialized: %ux%u cells, resolution=%.3f, size=(%.2fm, %.2fm), origin=(%.2f, %.2f), divisions=%dx%d",
                msg->info.width, msg->info.height, msg->info.resolution,
                map_width_, map_height_, map_origin_.x, map_origin_.y,
                params_.divisions_x, params_.divisions_y);
          } else if (old_divisions_x != params_.divisions_x || old_divisions_y != params_.divisions_y) {
            // Divisions changed: rebuild distance graph and clear historical sort
            initializeDistanceGraph();
            previous_sorted_subregions_.clear();
            RCLCPP_INFO(get_logger(), "Grid divisions updated: %dx%d -> %dx%d (map size: %.1fm x %.1fm)",
                        old_divisions_x, old_divisions_y, params_.divisions_x, params_.divisions_y,
                        map_width_, map_height_);
          }
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_ = msg;
          if (!has_initial_robot_position_) {
            initial_robot_position_ = msg->pose.pose.position;
            has_initial_robot_position_ = true;
          }
        });

    fpoint_sub_ = create_subscription<tdle::msg::PointArray>(
        "/filtered_points", rclcpp::QoS(10),
        [this](const tdle::msg::PointArray::SharedPtr msg) {
          filtered_points_ = msg->points;
          (void)msg;
        });

    // Publisher
    candidate_fp_pub_ = create_publisher<tdle::msg::PointArray>("/candidate_fpoint", 10);

    rclcpp::QoS vis_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    vis_qos.transient_local().keep_last(1);
    subregion_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("/subregion_marker", vis_qos);
    grid_vis_pub_ = create_publisher<visualization_msgs::msg::Marker>("/grid_marker", vis_qos);

    // Timers
    int period_ms = static_cast<int>(1000.0 / params_.publish_rate);
    arrangement_timer_ = create_wall_timer(
        chrono::milliseconds(period_ms),
        [this]() {
          if (!map_initialized_) return;
          arrangementLoop();
        });

    vis_timer_ = create_wall_timer(
        chrono::milliseconds(100),
        [this]() {
          if (!map_initialized_) return;
          drawGrid();
          if (!previous_sorted_subregions_.empty()) {
            drawSubregions(previous_sorted_subregions_);
          }
        });

    RCLCPP_INFO(get_logger(), "SubregionArrangement initialized");
    RCLCPP_INFO(
        get_logger(),
        "Selection params: min_points_per_region=%d, unknown_threshold=%d, unknown_sample_step=%.1f (%d samples)",
        params_.min_points_per_region, params_.unknown_threshold,
        params_.unknown_sample_step, unknownSampleCount());
    RCLCPP_INFO(
        get_logger(),
        "Sort params: frontier=ANA, unknown=greedy, ana_temp=[%.3f->%.4f], ana_iter=%d, ana_mu=%.2f, ana_lambda={sim=%.2f,end=%.3f,len=%.2f}, ana_seed=%d",
        params_.ana_initial_temp, params_.ana_stop_temp, params_.ana_max_iterations,
        params_.ana_mu, params_.ana_lambda_similarity,
        params_.ana_lambda_end_to_initial, params_.ana_lambda_path_length,
        params_.ana_seed);
    if (params_.unknown_threshold >= unknownSampleCount()) {
      RCLCPP_WARN(
          get_logger(),
          "unknown_threshold=%d but unknown sampler only checks %d points per region; has_unknown may never become true",
          params_.unknown_threshold, unknownSampleCount());
    }
  }

private:
  struct Params {
    float max_subregion_size = 5.0f;  // Maximum side length per subregion (meters)
    int divisions_x = 3;              // Number of x-axis (column) divisions, computed dynamically
    int divisions_y = 3;              // Number of y-axis (row) divisions, computed dynamically
    int min_points_per_region = 2;
    int unknown_threshold = 80;
    float publish_rate = 10.0f;
    float unknown_sample_step = 10.0f;
    float ana_initial_temp = 1.0f;
    float ana_stop_temp = 1e-3f;
    int ana_max_iterations = 120;
    float ana_mu = 2.0f;
    float ana_lambda_similarity = 1.0f;
    float ana_lambda_end_to_initial = 0.01f;
    float ana_lambda_path_length = 0.65f;
    int ana_seed = 42;
  } params_;

  struct RegionSelectionInfo {
    int frontier_count = 0;
    int unknown_count = 0;
    bool has_frontier = false;
    bool has_unknown = false;
    bool selected = false;
  };

  nav_msgs::msg::OccupancyGrid::SharedPtr map_data_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  vector<geometry_msgs::msg::Point> filtered_points_;
  vector<geometry_msgs::msg::Point> center_points_;

  double map_width_ = 0.0;
  double map_height_ = 0.0;
  geometry_msgs::msg::Point map_origin_;

  vector<vector<float>> distance_graph_;
  vector<int> previous_sorted_subregions_;
  vector<RegionSelectionInfo> last_region_selection_;
  int robot_subregion_ = -1;
  geometry_msgs::msg::Point initial_robot_position_;
  bool has_initial_robot_position_ = false;
  int last_published_subregion_ = -1;
  atomic<bool> map_initialized_{false};

  mt19937 gen_;

  string map_topic_, odom_topic_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tdle::msg::PointArray>::SharedPtr fpoint_sub_;
  rclcpp::Publisher<tdle::msg::PointArray>::SharedPtr candidate_fp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr subregion_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr grid_vis_pub_;
  rclcpp::TimerBase::SharedPtr arrangement_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;

  visualization_msgs::msg::Marker subregion_vis_;
  visualization_msgs::msg::Marker grid_vis_;

  string formatPoint(const geometry_msgs::msg::Point &point) const
  {
    ostringstream oss;
    oss << fixed << setprecision(2)
        << "(" << point.x << ", " << point.y << ", " << point.z << ")";
    return oss.str();
  }

  string formatRegionList(const vector<int> &regions) const
  {
    ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < regions.size(); i++) {
      if (i > 0) {
        oss << ", ";
      }
      oss << regions[i];
    }
    oss << "]";
    return oss.str();
  }

  double regionDistanceToPoint(int subregion_idx, const geometry_msgs::msg::Point &point) const
  {
    if (subregion_idx < 0 || subregion_idx >= params_.divisions_x * params_.divisions_y) {
      return numeric_limits<double>::max();
    }
    auto center = getSubregionCenter(subregion_idx);
    double dx = center.x - point.x;
    double dy = center.y - point.y;
    return sqrt(dx * dx + dy * dy);
  }

  int getSubregionIndex(const geometry_msgs::msg::Point &point) const
  {
    if (params_.divisions_x <= 0 || params_.divisions_y <= 0 ||
        map_width_ <= 0.0 || map_height_ <= 0.0) {
      return -1;
    }

    const double eps = 1e-6;
    double rel_x = point.x - map_origin_.x;
    double rel_y = point.y - map_origin_.y;

    if (rel_x < -eps || rel_y < -eps || rel_x > map_width_ + eps || rel_y > map_height_ + eps) {
      return -1;
    }

    double grid_width = map_width_ / params_.divisions_x;
    double grid_height = map_height_ / params_.divisions_y;
    if (grid_width <= 0.0 || grid_height <= 0.0) {
      return -1;
    }

    rel_x = min(max(rel_x, 0.0), map_width_ - eps);
    rel_y = min(max(rel_y, 0.0), map_height_ - eps);

    int col = static_cast<int>(rel_x / grid_width);
    int row = static_cast<int>(rel_y / grid_height);

    col = min(max(col, 0), params_.divisions_x - 1);
    row = min(max(row, 0), params_.divisions_y - 1);
    return row * params_.divisions_x + col;
  }

  int unknownSampleCount() const
  {
    int samples_per_axis = static_cast<int>(params_.unknown_sample_step);
    return samples_per_axis * samples_per_axis;
  }

  int countSubregionUnknownSamples(int subregion_idx) const
  {
    auto center = getSubregionCenter(subregion_idx);

    double grid_width = map_width_ / params_.divisions_x;
    double grid_height = map_height_ / params_.divisions_y;

    double x1 = center.x - grid_width / 2;
    double x2 = center.x + grid_width / 2;
    double y1 = center.y - grid_height / 2;
    double y2 = center.y + grid_height / 2;

    double x_step = (x2 - x1) / params_.unknown_sample_step;
    double y_step = (y2 - y1) / params_.unknown_sample_step;

    int unknown_count = 0;
    for (int i = 0; i < static_cast<int>(params_.unknown_sample_step); i++) {
      for (int j = 0; j < static_cast<int>(params_.unknown_sample_step); j++) {
        float x = x1 + i * x_step;
        float y = y1 + j * y_step;
        if (getGridValue(x, y) == -1) {
          unknown_count++;
        }
      }
    }

    return unknown_count;
  }

  void updateDivisions()
  {
    // Compute divisions from map size while keeping each subregion below max_subregion_size.
    // Always keep at least 3x3.
    params_.divisions_x = std::max(3, static_cast<int>(std::ceil(map_width_ / params_.max_subregion_size)));
    params_.divisions_y = std::max(3, static_cast<int>(std::ceil(map_height_ / params_.max_subregion_size)));
  }

  void initializeDistanceGraph()
  {
    int num_regions = params_.divisions_x * params_.divisions_y;
    // Rebuild a fully square matrix whenever divisions change.
    // `resize()` on the outer vector keeps existing row lengths unchanged,
    // which can leave old rows shorter than `num_regions` and cause OOB writes.
    distance_graph_.assign(num_regions, vector<float>(num_regions, 0.0f));

    for (int i = 0; i < num_regions; i++) {
      for (int j = 0; j < num_regions; j++) {
        // i / divisions_x = row (y-axis), i % divisions_x = column (x-axis)
        float dx = pow((i % params_.divisions_x - j % params_.divisions_x), 2);
        float dy = pow((i / params_.divisions_x - j / params_.divisions_x), 2);
        distance_graph_[i][j] = sqrt(dx + dy);
      }
    }
  }

  void initializeVisualization()
  {
    string frame_id = map_data_ ? map_data_->header.frame_id : "map";

    subregion_vis_.header.frame_id = frame_id;
    subregion_vis_.ns = "SubRegion";
    subregion_vis_.type = visualization_msgs::msg::Marker::CUBE_LIST;
    subregion_vis_.action = visualization_msgs::msg::Marker::ADD;
    subregion_vis_.pose.orientation.w = 1.0;
    subregion_vis_.scale.z = 0.01;

    grid_vis_.header.frame_id = frame_id;
    grid_vis_.ns = "grid";
    grid_vis_.type = visualization_msgs::msg::Marker::LINE_LIST;
    grid_vis_.action = visualization_msgs::msg::Marker::ADD;
    grid_vis_.pose.orientation.w = 1.0;
    grid_vis_.scale.x = 0.05;
    grid_vis_.color.r = 0.4;
    grid_vis_.color.g = 1.0;
    grid_vis_.color.b = 0.8;
    grid_vis_.color.a = 0.6;
  }

  geometry_msgs::msg::Point getSubregionCenter(int subregion_idx) const
  {
    double grid_width = map_width_ / params_.divisions_x;
    double grid_height = map_height_ / params_.divisions_y;

    geometry_msgs::msg::Point center;
    // subregion_idx / divisions_x = row (y-axis)
    // subregion_idx % divisions_x = column (x-axis)
    center.x = map_origin_.x + (subregion_idx % params_.divisions_x) * grid_width + grid_width / 2;
    center.y = map_origin_.y + (subregion_idx / params_.divisions_x) * grid_height + grid_height / 2;
    center.z = 0.0;

    return center;
  }

  bool isPointInSubregion(int subregion_idx, const geometry_msgs::msg::Point &point) const
  {
    return getSubregionIndex(point) == subregion_idx;
  }

  float getGridValue(float x, float y) const
  {
    if (!map_data_) return -1;

    float resolution = map_data_->info.resolution;
    float origin_x = map_data_->info.origin.position.x;
    float origin_y = map_data_->info.origin.position.y;
    int width = map_data_->info.width;
    int height = map_data_->info.height;

    int grid_x = static_cast<int>((x - origin_x) / resolution);
    int grid_y = static_cast<int>((y - origin_y) / resolution);

    if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height)
      return -1;

    return map_data_->data[grid_y * width + grid_x];
  }

  bool isSubregionUnknown(int subregion_idx) const
  {
    int unknown_count = countSubregionUnknownSamples(subregion_idx);
    return unknown_count > params_.unknown_threshold;
  }

  vector<int> selectSubregions()
  {
    vector<int> selected;
    int num_subregions = params_.divisions_x * params_.divisions_y;
    last_region_selection_.assign(num_subregions, RegionSelectionInfo{});

    vector<geometry_msgs::msg::Point> points = filtered_points_;
    geometry_msgs::msg::Point robot_pos;
    if (odom_) {
      robot_pos = odom_->pose.pose.position;
      robot_subregion_ = getSubregionIndex(robot_pos);
    } else {
      robot_subregion_ = -1;
    }

    center_points_.clear();

    for (int i = 0; i < num_subregions; i++) {
      auto center = getSubregionCenter(i);
      center_points_.push_back(center);

      int fpoint_count = 0;
      for (const auto &fp : points) {
        if (isPointInSubregion(i, fp))
          fpoint_count++;
      }

      bool fpoint_inside = fpoint_count >= params_.min_points_per_region;
      int unknown_count = countSubregionUnknownSamples(i);
      bool has_unknown = unknown_count > params_.unknown_threshold;

      last_region_selection_[i].frontier_count = fpoint_count;
      last_region_selection_[i].unknown_count = unknown_count;
      last_region_selection_[i].has_frontier = fpoint_inside;
      last_region_selection_[i].has_unknown = has_unknown;

      if (has_unknown || fpoint_inside) {
        selected.push_back(i);
        last_region_selection_[i].selected = true;
        const char *reason =
            (has_unknown && fpoint_inside) ? "frontier+unknown" :
            (fpoint_inside ? "frontier" : "unknown");
        (void)reason;
      }

    }

    return selected;
  }

  vector<vector<geometry_msgs::msg::Point>> classifyPoints(const vector<int> &sorted)
  {
    vector<vector<geometry_msgs::msg::Point>> classified;
    vector<geometry_msgs::msg::Point> points = filtered_points_;

    for (size_t i = 0; i < sorted.size(); i++) {
      vector<geometry_msgs::msg::Point> points_in_region;
      for (const auto &fp : points) {
        if (isPointInSubregion(sorted[i], fp)) {
          points_in_region.push_back(fp);
        }
      }
      classified.push_back(points_in_region);
    }

    return classified;
  }

  struct AnaScoreComponents
  {
    double similarity = 0.0;
    double end_distance_norm = 0.0;
    double path_length_norm = 0.0;
    double score = -numeric_limits<double>::infinity();
  };

  double regionDistanceBetween(int region_a, int region_b) const
  {
    if (region_a >= 0 && region_a < static_cast<int>(distance_graph_.size()) &&
        region_b >= 0 && region_b < static_cast<int>(distance_graph_[region_a].size())) {
      return distance_graph_[region_a][region_b];
    }
    auto a = getSubregionCenter(region_a);
    auto b = getSubregionCenter(region_b);
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
  }

  double computeRouteLengthNorm(const vector<int> &route, double map_diag) const
  {
    if (route.size() < 2 || map_diag <= 1e-6) {
      return 0.0;
    }
    double length = 0.0;
    for (size_t i = 1; i < route.size(); i++) {
      length += regionDistanceBetween(route[i - 1], route[i]);
    }
    double norm_base = map_diag * static_cast<double>(max<size_t>(1, route.size() - 1));
    return length / norm_base;
  }

  double computeEndDistanceNorm(
      const vector<int> &route,
      int initial_region,
      double map_diag) const
  {
    if (route.empty() || initial_region < 0 || map_diag <= 1e-6) {
      return 0.0;
    }
    return regionDistanceBetween(route.back(), initial_region) / map_diag;
  }

  double computeDtwDistanceNorm(
      const vector<int> &route,
      const vector<int> &reference_route,
      double map_diag) const
  {
    if (route.empty() || reference_route.empty() || map_diag <= 1e-6) {
      return 0.0;
    }

    size_t n = route.size();
    size_t m = reference_route.size();
    double inf = numeric_limits<double>::infinity();
    vector<vector<double>> dtw(n + 1, vector<double>(m + 1, inf));
    dtw[0][0] = 0.0;

    for (size_t i = 1; i <= n; i++) {
      for (size_t j = 1; j <= m; j++) {
        double step_cost = regionDistanceBetween(route[i - 1], reference_route[j - 1]);
        dtw[i][j] = step_cost + min({dtw[i - 1][j], dtw[i][j - 1], dtw[i - 1][j - 1]});
      }
    }

    if (!isfinite(dtw[n][m])) {
      return numeric_limits<double>::infinity();
    }
    double norm_base = map_diag * static_cast<double>(max(n, m));
    return dtw[n][m] / norm_base;
  }

  vector<int> buildReferenceRoute(const vector<int> &selected, int start_region) const
  {
    vector<int> reference;
    reference.reserve(selected.size());

    auto is_selected = [&selected](int region) {
      return find(selected.begin(), selected.end(), region) != selected.end();
    };

    if (is_selected(start_region)) {
      reference.push_back(start_region);
    }

    for (int region : previous_sorted_subregions_) {
      if (!is_selected(region)) {
        continue;
      }
      if (find(reference.begin(), reference.end(), region) == reference.end()) {
        reference.push_back(region);
      }
    }

    for (int region : selected) {
      if (find(reference.begin(), reference.end(), region) == reference.end()) {
        reference.push_back(region);
      }
    }

    return reference;
  }

  vector<int> buildInitialAnaRoute(
      const vector<int> &selected,
      int start_region,
      const vector<int> &reference_route)
  {
    vector<int> route;
    route.reserve(selected.size());

    auto is_selected = [&selected](int region) {
      return find(selected.begin(), selected.end(), region) != selected.end();
    };
    auto is_used = [&route](int region) {
      return find(route.begin(), route.end(), region) != route.end();
    };

    if (is_selected(start_region)) {
      route.push_back(start_region);
    } else if (!selected.empty()) {
      route.push_back(selected.front());
    }

    for (int region : reference_route) {
      if (!is_selected(region) || is_used(region)) {
        continue;
      }
      route.push_back(region);
    }

    vector<int> remaining;
    for (int region : selected) {
      if (!is_used(region)) {
        remaining.push_back(region);
      }
    }

    shuffle(remaining.begin(), remaining.end(), gen_);
    route.insert(route.end(), remaining.begin(), remaining.end());
    return route;
  }

  bool evaluateAnaScore(
      const vector<int> &route,
      const vector<int> &reference_route,
      int initial_region,
      AnaScoreComponents &out) const
  {
    double map_diag = hypot(map_width_, map_height_);
    if (route.empty() || map_diag <= 1e-6) {
      return false;
    }

    out.path_length_norm = computeRouteLengthNorm(route, map_diag);
    out.end_distance_norm = computeEndDistanceNorm(route, initial_region, map_diag);

    double dtw_norm = computeDtwDistanceNorm(route, reference_route, map_diag);
    if (!isfinite(dtw_norm)) {
      return false;
    }
    out.similarity = 1.0 / (1.0 + max(0.0, dtw_norm));
    out.score = params_.ana_lambda_similarity * out.similarity -
                params_.ana_lambda_end_to_initial * out.end_distance_norm -
                params_.ana_lambda_path_length * out.path_length_norm;
    return isfinite(out.score);
  }

  int resolveStartSubregion(
      const vector<int> &sr_sel,
      int preferred_start,
      const geometry_msgs::msg::Point &robot_pos) const
  {
    if (sr_sel.empty()) {
      return -1;
    }

    if (find(sr_sel.begin(), sr_sel.end(), preferred_start) != sr_sel.end()) {
      return preferred_start;
    }

    double best_dist = numeric_limits<double>::max();
    int best_region = sr_sel.front();
    for (int region : sr_sel) {
      double dist = regionDistanceToPoint(region, robot_pos);
      if (dist < best_dist) {
        best_dist = dist;
        best_region = region;
      }
    }
    return best_region;
  }

  vector<int> sortSubregionsAna(const vector<int> &sr_sel, int start_region)
  {
    if (sr_sel.size() < 3) {
      return sortSubregionsGreedy(sr_sel, start_region);
    }

    int initial_region = has_initial_robot_position_ ? getSubregionIndex(initial_robot_position_) : -1;
    if (initial_region < 0) {
      initial_region = start_region;
    }

    vector<int> reference_route = buildReferenceRoute(sr_sel, start_region);
    vector<int> best_route = buildInitialAnaRoute(sr_sel, start_region, reference_route);
    if (best_route.size() != sr_sel.size()) {
      return sortSubregionsGreedy(sr_sel, start_region);
    }
    if (reference_route.size() < 2) {
      reference_route = best_route;
    }

    vector<int> current_route = best_route;
    AnaScoreComponents best_components;
    if (!evaluateAnaScore(best_route, reference_route, initial_region, best_components)) {
      return sortSubregionsGreedy(sr_sel, start_region);
    }
    AnaScoreComponents current_components = best_components;
    double best_score = best_components.score;
    double current_score = current_components.score;

    const int max_iter = max(1, params_.ana_max_iterations);
    const double temp_stop = max(1e-6f, params_.ana_stop_temp);
    double temperature = max(1e-6f, params_.ana_initial_temp);
    uniform_real_distribution<double> random_01(0.0, 1.0);

    int accepted = 0;
    int iter_done = 0;
    for (int iter = 0; iter < max_iter && temperature > temp_stop; iter++) {
      iter_done = iter + 1;
      vector<int> candidate_route = current_route;
      uniform_int_distribution<size_t> pick_idx(1, candidate_route.size() - 1);
      size_t idx_a = pick_idx(gen_);
      size_t idx_b = pick_idx(gen_);
      int guard = 0;
      while (idx_a == idx_b && guard < 8) {
        idx_b = pick_idx(gen_);
        guard++;
      }
      if (idx_a == idx_b) {
        continue;
      }
      swap(candidate_route[idx_a], candidate_route[idx_b]);

      AnaScoreComponents candidate_components;
      if (!evaluateAnaScore(candidate_route, reference_route, initial_region, candidate_components)) {
        continue;
      }

      double delta = candidate_components.score - current_score;
      bool accept = delta >= 0.0 || random_01(gen_) < exp(delta / temperature);
      if (accept) {
        current_route = candidate_route;
        current_components = candidate_components;
        current_score = candidate_components.score;
        accepted++;
      }

      if (current_score > best_score) {
        best_route = current_route;
        best_components = current_components;
        best_score = current_score;
      }

      double progress = static_cast<double>(iter + 1) / static_cast<double>(max_iter);
      double eta = exp(params_.ana_mu * (progress - 1.0));
      temperature *= eta;
    }

    double accept_ratio = iter_done > 0 ? static_cast<double>(accepted) / static_cast<double>(iter_done) : 0.0;
    RCLCPP_DEBUG(
        get_logger(),
        "[SortANA] start=%d initial=%d ref_len=%zu iter=%d/%d accept_ratio=%.2f score=%.4f sim=%.4f end=%.4f len=%.4f",
        start_region, initial_region, reference_route.size(), iter_done, max_iter, accept_ratio,
        best_components.score, best_components.similarity,
        best_components.end_distance_norm, best_components.path_length_norm);
    return best_route;
  }

  vector<int> sortSubregionsGreedy(const vector<int> &sr_sel, int start_region)
  {
    if (sr_sel.empty()) return {};

    vector<int> sorted;
    vector<bool> visited(sr_sel.size(), false);

    int current = start_region;
    if (find(sr_sel.begin(), sr_sel.end(), current) == sr_sel.end()) {
      current = sr_sel.front();
    }

    sorted.push_back(current);
    auto it = find(sr_sel.begin(), sr_sel.end(), current);
    if (it != sr_sel.end()) {
      visited[it - sr_sel.begin()] = true;
    }

    while (sorted.size() < sr_sel.size()) {
      float min_dist = numeric_limits<float>::max();
      int next_idx = -1;

      for (size_t i = 0; i < sr_sel.size(); i++) {
        if (visited[i]) continue;
        if (current < 0 || current >= static_cast<int>(distance_graph_.size()) ||
            sr_sel[i] < 0 || sr_sel[i] >= static_cast<int>(distance_graph_[current].size())) {
          RCLCPP_WARN(
              get_logger(),
              "Distance graph index out of range: current=%d target=%d graph_size=%zu row_size=%zu",
              current, sr_sel[i], distance_graph_.size(),
              (current >= 0 && current < static_cast<int>(distance_graph_.size())) ? distance_graph_[current].size() : 0u);
          continue;
        }
        float dist = distance_graph_[current][sr_sel[i]];
        if (dist < min_dist) {
          min_dist = dist;
          next_idx = i;
        }
      }

      if (next_idx < 0) break;

      current = sr_sel[next_idx];
      sorted.push_back(current);
      visited[next_idx] = true;
    }

    return sorted;
  }

  vector<int> sortSubregions(
      const vector<int> &selected,
      int preferred_start,
      const geometry_msgs::msg::Point &robot_pos,
      bool allow_ana = true)
  {
    if (selected.empty()) return {};
    int start_region = resolveStartSubregion(selected, preferred_start, robot_pos);
    if (selected.size() < 2) return {start_region};
    if (allow_ana) {
      return sortSubregionsAna(selected, start_region);
    }
    return sortSubregionsGreedy(selected, start_region);
  }

  void arrangementLoop()
  {
    if (!odom_) return;

    auto robot_pos = odom_->pose.pose.position;
    (void)robot_pos;

    vector<int> selected = selectSubregions();
    if (selected.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No subregions selected");
      return;
    }

    vector<int> frontier_selected;
    vector<int> unknown_only_selected;
    frontier_selected.reserve(selected.size());
    unknown_only_selected.reserve(selected.size());
    for (int region : selected) {
      if (region >= 0 && region < static_cast<int>(last_region_selection_.size()) &&
          last_region_selection_[region].has_frontier) {
        frontier_selected.push_back(region);
      } else {
        unknown_only_selected.push_back(region);
      }
    }

    vector<int> sorted_frontier = sortSubregions(frontier_selected, robot_subregion_, robot_pos, true);
    int unknown_start = sorted_frontier.empty() ? robot_subregion_ : sorted_frontier.back();
    vector<int> sorted_unknown = sortSubregions(unknown_only_selected, unknown_start, robot_pos, false);

    vector<int> sorted = sorted_frontier;
    sorted.insert(sorted.end(), sorted_unknown.begin(), sorted_unknown.end());
    previous_sorted_subregions_ = sorted;

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "[Sort] robot_subregion=%d selected_total=%zu frontier_selected=%zu unknown_selected=%zu sorted_frontier=%s sorted_unknown=%s final=%s",
        robot_subregion_, selected.size(), frontier_selected.size(), unknown_only_selected.size(),
        formatRegionList(sorted_frontier).c_str(),
        formatRegionList(sorted_unknown).c_str(),
        formatRegionList(sorted).c_str());

    vector<vector<geometry_msgs::msg::Point>> classified = classifyPoints(sorted);
    last_published_subregion_ = -1;

    auto publish_first_match = [this, &sorted, &classified](bool frontier_only) -> bool {
      for (size_t i = 0; i < classified.size(); i++) {
        if (classified[i].empty()) {
          continue;
        }

        int region_idx = sorted[i];
        if (region_idx < 0 || region_idx >= static_cast<int>(last_region_selection_.size())) {
          continue;
        }

        const auto &region_info = last_region_selection_[region_idx];
        if (frontier_only && !region_info.has_frontier) {
          continue;
        }

        auto publish_points = classified[i];
        if (i + 1 < sorted.size() && sorted[i + 1] < static_cast<int>(center_points_.size())) {
          publish_points.push_back(center_points_[sorted[i + 1]]);
        }

        tdle::msg::PointArray cand_fp;
        cand_fp.points = publish_points;
        last_published_subregion_ = region_idx;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[Publish] current_subregion=%d sorted_idx=%zu points=%zu reason=%s",
            region_idx, i, cand_fp.points.size(),
            frontier_only ? "frontier-priority" : "fallback");
        candidate_fp_pub_->publish(cand_fp);
        return true;
      }
      return false;
    };

    if (!publish_first_match(true) && !publish_first_match(false)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Selected subregions contain no publishable frontier points");
    }
  }

  void drawGrid()
  {
    grid_vis_.points.clear();
    grid_vis_.header.stamp = now();

    double grid_width = map_width_ / params_.divisions_x;
    double grid_height = map_height_ / params_.divisions_y;
    double x = map_origin_.x + grid_width;
    double y = map_origin_.y + grid_height;

    // Draw vertical lines (x-axis divisions)
    for (int i = 0; i < params_.divisions_x - 1; i++) {
      geometry_msgs::msg::Point start, end;
      start.x = x; start.y = map_origin_.y; start.z = 0.0;
      end.x = x; end.y = map_origin_.y + map_height_; end.z = 0.0;
      grid_vis_.points.push_back(start);
      grid_vis_.points.push_back(end);
      x += grid_width;
    }

    // Draw horizontal lines (y-axis divisions)
    for (int i = 0; i < params_.divisions_y - 1; i++) {
      geometry_msgs::msg::Point start, end;
      start.x = map_origin_.x; start.y = y; start.z = 0.0;
      end.x = map_origin_.x + map_width_; end.y = y; end.z = 0.0;
      grid_vis_.points.push_back(start);
      grid_vis_.points.push_back(end);
      y += grid_height;
    }

    grid_vis_pub_->publish(grid_vis_);
  }

  void drawSubregions(const vector<int> &sorted)
  {
    subregion_vis_.points.clear();
    subregion_vis_.colors.clear();
    subregion_vis_.header.stamp = now();

    double grid_width = map_width_ / params_.divisions_x;
    double grid_height = map_height_ / params_.divisions_y;
    subregion_vis_.scale.x = grid_width;
    subregion_vis_.scale.y = grid_height;

    for (size_t i = 0; i < sorted.size(); i++) {
      int sr_idx = sorted[i];
      auto center = getSubregionCenter(sr_idx);

      geometry_msgs::msg::Point position;
      position.x = center.x;
      position.y = center.y;
      position.z = 0.0;
      subregion_vis_.points.push_back(position);

      float color_factor = static_cast<float>(i) / sorted.size();
      std_msgs::msg::ColorRGBA color;
      color.r = 0.2 * log10(10 * color_factor + 1) + 0.1;
      color.g = 0.6 * log10(10 * color_factor + 1) + 0.3;
      color.b = 0.3 * log10(10 * color_factor + 1) + 0.15;
      color.a = 1 + 0.3 * log10(10 * color_factor + 10);
      subregion_vis_.colors.push_back(color);
    }

    subregion_vis_pub_->publish(subregion_vis_);
  }
};

} // namespace tdle

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tdle::SubregionArrangementNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
