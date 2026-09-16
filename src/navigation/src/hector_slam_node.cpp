#include "c1_slam/hector_slam_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <numeric>
#include <tuple>

namespace c1_slam
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaxTrajectorySize = 50000U;
constexpr std::size_t kMaxPoseBufferSize = 2U;
constexpr std::size_t kMaxDeltaHistorySize = 5U;
}

HectorSLAMNode::HectorSLAMNode(const rclcpp::NodeOptions & options)
: Node("hector_slam_node", options)
{
  declare_parameters();
  load_parameters();

  grid_width_ = std::max(1, static_cast<int>(map_width_m_ / resolution_));
  grid_height_ = std::max(1, static_cast<int>(map_height_m_ / resolution_));
  origin_x_ = -map_width_m_ / 2.0;
  origin_y_ = -map_height_m_ / 2.0;
  log_odds_ = Eigen::MatrixXf::Zero(grid_height_, grid_width_);
  observed_ = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Constant(
    grid_height_, grid_width_, false);

  auto map_qos = rclcpp::QoS(1).reliable().transient_local();
  auto scan_qos = rclcpp::SensorDataQoS().keep_last(10);
  if (publish_map_topic_) {
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
  }
  if (publish_pose_topic_) {
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
  }
  traj_pub_ = create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  pose_text_pub_ = create_publisher<std_msgs::msg::String>("/robot_pose_text", 10);
  quality_pub_ = create_publisher<std_msgs::msg::Float32>("/scan_match_quality", 10);
  if (publish_map_odom_tf_ || publish_odom_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, scan_qos, std::bind(&HectorSLAMNode::scan_callback, this, std::placeholders::_1));
  // BAB 4.2 pure LiDAR mode must not consume IMU at all.  Create the IMU
  // subscription only when its rotation plausibility gate is explicitly enabled.
  if (use_imu_rotation_gate_) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS().keep_last(20),
      std::bind(&HectorSLAMNode::imu_callback, this, std::placeholders::_1));
  }

  const double map_period = get_parameter("map_pub_period").as_double();
  const double pose_period = std::max(
    get_parameter("pose_pub_period").as_double(), get_parameter("tf_pub_period").as_double());
  map_timer_ = create_wall_timer(std::chrono::duration<double>(map_period),
    std::bind(&HectorSLAMNode::publish_map_callback, this));
  pose_timer_ = create_wall_timer(std::chrono::duration<double>(pose_period),
    std::bind(&HectorSLAMNode::publish_pose_tf_callback, this));
  traj_timer_ = create_wall_timer(std::chrono::milliseconds(500),
    std::bind(&HectorSLAMNode::publish_trajectory_callback, this));
  stats_timer_ = create_wall_timer(std::chrono::seconds(5),
    std::bind(&HectorSLAMNode::log_stats_callback, this));

  publish_map();
  RCLCPP_INFO(get_logger(), "Hector SLAM C++17 initialized: %.1fx%.1fm, resolution %.3fm (%dx%d), scan=%s",
    map_width_m_, map_height_m_, resolution_, grid_width_, grid_height_, scan_topic_.c_str());
  RCLCPP_INFO(get_logger(),
    "[ANTI-STARBURST-V7] active: min_points=%d overlap=%d motion_confirm=%d max_gap=%.2fs median=%.3fm changed=%.3fm/%.0f%%",
    min_valid_scan_points_, motion_min_overlap_, motion_confirm_threshold_, max_scan_gap_,
    motion_median_threshold_, motion_changed_range_threshold_, motion_changed_ratio_threshold_ * 100.0);
  RCLCPP_INFO(get_logger(),
    "[IMU-ROT-GATE] %s: timeout=%.2fs stationary_gyro<%.4frad/s margin=%.2fdeg scale=%.2f",
    use_imu_rotation_gate_ ? "enabled" : "disabled", imu_rotation_gate_timeout_,
    imu_stationary_gyro_threshold_, imu_rotation_margin_ * 180.0 / kPi, imu_rotation_scale_);
}

void HectorSLAMNode::declare_parameters()
{
  declare_parameter("map_width", 40.0); declare_parameter("map_height", 40.0);
  declare_parameter("map_resolution", 0.025); declare_parameter("max_range", 8.0);
  declare_parameter("min_range", 0.15); declare_parameter("scan_filtering", true);
  declare_parameter("scan_filter_window", 7); declare_parameter("update_factor_free", 0.15);
  declare_parameter("update_factor_occupied", 0.35);
  declare_parameter("map_update_distance_threshold", 0.05);
  declare_parameter("map_update_angle_threshold", 0.035);
  declare_parameter("min_scan_match_quality", 0.30);
  declare_parameter("map_pub_period", 0.1); declare_parameter("pose_pub_period", 0.033);
  declare_parameter("tf_pub_period", 0.033); declare_parameter("publish_odom_tf", false);
  declare_parameter("odom_child_frame", std::string("base_footprint"));
  declare_parameter("scan_topic", std::string("/scan_nav"));
  declare_parameter("publish_odom_topic", true); declare_parameter("publish_map_odom_tf", false);
  declare_parameter("publish_map_topic", true); declare_parameter("publish_pose_topic", true);
  declare_parameter("robot_initial_x", 0.0);
  declare_parameter("robot_initial_y", 0.0); declare_parameter("robot_initial_yaw", 0.0);
  declare_parameter("delta_ema_alpha", 0.4); declare_parameter("stationary_dist_threshold", 0.015);
  declare_parameter("stationary_confirm_threshold", 8); declare_parameter("deadband_dist", 0.01);
  declare_parameter("deadband_angle_deg", 1.0); declare_parameter("quality_gate_threshold", 0.45);
  declare_parameter("quality_gate_significant_dist", 0.02);
  declare_parameter("quality_gate_significant_angle_deg", 3.0);
  declare_parameter("max_vel_trans", 0.5); declare_parameter("max_vel_rot", 1.0);
  declare_parameter("coarse_search_x", 0.15); declare_parameter("coarse_search_y", 0.15);
  declare_parameter("coarse_search_theta_deg", 4.0); declare_parameter("max_single_trans", 0.8);
  declare_parameter("max_single_rot_deg", 15.0); declare_parameter("deviation_threshold", 0.25);
  declare_parameter("deviation_threshold_init", 0.5);
  declare_parameter("min_valid_scan_points", 50);
  declare_parameter("motion_confirm_threshold", 2);
  declare_parameter("motion_min_overlap", 35);
  declare_parameter("motion_median_threshold", 0.020);
  declare_parameter("motion_changed_range_threshold", 0.060);
  declare_parameter("motion_changed_ratio_threshold", 0.22);
  declare_parameter("motion_reference_max_scans", 12);
  declare_parameter("max_scan_gap", 0.30);
  declare_parameter("search_tie_epsilon", 1.0e-6);
  declare_parameter("use_imu_rotation_gate", true);
  declare_parameter("imu_rotation_gate_timeout", 0.30);
  declare_parameter("imu_stationary_gyro_threshold", 0.035);
  declare_parameter("imu_rotation_margin_deg", 0.8);
  declare_parameter("imu_rotation_scale", 1.8);
}

void HectorSLAMNode::load_parameters()
{
  map_width_m_ = get_parameter("map_width").as_double(); map_height_m_ = get_parameter("map_height").as_double();
  resolution_ = get_parameter("map_resolution").as_double(); max_range_ = get_parameter("max_range").as_double();
  min_range_ = get_parameter("min_range").as_double(); scan_filtering_ = get_parameter("scan_filtering").as_bool();
  scan_filter_window_ = get_parameter("scan_filter_window").as_int();
  log_odds_free_ = -get_parameter("update_factor_free").as_double();
  log_odds_occ_ = get_parameter("update_factor_occupied").as_double();
  map_update_distance_threshold_ = get_parameter("map_update_distance_threshold").as_double();
  map_update_angle_threshold_ = get_parameter("map_update_angle_threshold").as_double();
  min_match_improvement_ = get_parameter("min_scan_match_quality").as_double();
  publish_odom_tf_ = get_parameter("publish_odom_tf").as_bool();
  odom_child_frame_ = get_parameter("odom_child_frame").as_string();
  scan_topic_ = get_parameter("scan_topic").as_string();
  publish_odom_topic_ = get_parameter("publish_odom_topic").as_bool();
  publish_map_odom_tf_ = get_parameter("publish_map_odom_tf").as_bool();
  publish_map_topic_ = get_parameter("publish_map_topic").as_bool();
  publish_pose_topic_ = get_parameter("publish_pose_topic").as_bool();
  robot_x_ = get_parameter("robot_initial_x").as_double(); robot_y_ = get_parameter("robot_initial_y").as_double();
  robot_theta_ = get_parameter("robot_initial_yaw").as_double(); delta_ema_alpha_ = get_parameter("delta_ema_alpha").as_double();
  stationary_dist_threshold_ = get_parameter("stationary_dist_threshold").as_double();
  stationary_confirm_threshold_ = get_parameter("stationary_confirm_threshold").as_int();
  deadband_dist_ = get_parameter("deadband_dist").as_double(); deadband_angle_ = get_parameter("deadband_angle_deg").as_double() * kPi / 180.0;
  quality_gate_threshold_ = get_parameter("quality_gate_threshold").as_double();
  quality_gate_significant_dist_ = get_parameter("quality_gate_significant_dist").as_double();
  quality_gate_significant_angle_ = get_parameter("quality_gate_significant_angle_deg").as_double() * kPi / 180.0;
  max_vel_trans_ = get_parameter("max_vel_trans").as_double(); max_vel_rot_ = get_parameter("max_vel_rot").as_double();
  coarse_search_x_ = get_parameter("coarse_search_x").as_double(); coarse_search_y_ = get_parameter("coarse_search_y").as_double();
  coarse_search_theta_ = get_parameter("coarse_search_theta_deg").as_double() * kPi / 180.0;
  max_single_trans_ = get_parameter("max_single_trans").as_double();
  max_single_rot_ = get_parameter("max_single_rot_deg").as_double() * kPi / 180.0;
  deviation_threshold_ = get_parameter("deviation_threshold").as_double();
  deviation_threshold_init_ = get_parameter("deviation_threshold_init").as_double();
  min_valid_scan_points_ = std::max(15, static_cast<int>(get_parameter("min_valid_scan_points").as_int()));
  motion_confirm_threshold_ = std::max(1, static_cast<int>(get_parameter("motion_confirm_threshold").as_int()));
  motion_min_overlap_ = std::max(10, static_cast<int>(get_parameter("motion_min_overlap").as_int()));
  motion_median_threshold_ = std::max(0.001, get_parameter("motion_median_threshold").as_double());
  motion_changed_range_threshold_ = std::max(0.001, get_parameter("motion_changed_range_threshold").as_double());
  motion_changed_ratio_threshold_ = std::clamp(get_parameter("motion_changed_ratio_threshold").as_double(), 0.01, 1.0);
  motion_reference_max_scans_ = std::max(3, static_cast<int>(get_parameter("motion_reference_max_scans").as_int()));
  max_scan_gap_ = std::max(0.10, get_parameter("max_scan_gap").as_double());
  search_tie_epsilon_ = std::max(0.0, get_parameter("search_tie_epsilon").as_double());
  use_imu_rotation_gate_ = get_parameter("use_imu_rotation_gate").as_bool();
  imu_rotation_gate_timeout_ = std::max(0.05, get_parameter("imu_rotation_gate_timeout").as_double());
  imu_stationary_gyro_threshold_ = std::max(0.0, get_parameter("imu_stationary_gyro_threshold").as_double());
  imu_rotation_margin_ = std::max(0.0, get_parameter("imu_rotation_margin_deg").as_double()) * kPi / 180.0;
  imu_rotation_scale_ = std::max(1.0, get_parameter("imu_rotation_scale").as_double());
}

void HectorSLAMNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (!msg || !std::isfinite(msg->angular_velocity.z)) {
    return;
  }
  last_imu_gyro_z_ = msg->angular_velocity.z;
  const rclcpp::Time stamp(msg->header.stamp);
  last_imu_stamp_ = stamp.nanoseconds() > 0 ? stamp : now();
  imu_received_ = true;
}

std::vector<ScanPoint> HectorSLAMNode::scan_to_points(const sensor_msgs::msg::LaserScan & msg)
{
  std::vector<float> ranges(msg.ranges.begin(), msg.ranges.end());
  if (scan_filtering_ && ranges.size() >= 3U) {
    const int half = std::max(1, scan_filter_window_ / 2);
    const auto original = ranges;
    std::vector<float> window;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      window.clear();
      for (int j = -half; j <= half; ++j) {
        const auto index = (static_cast<long>(i) + j + static_cast<long>(ranges.size())) % static_cast<long>(ranges.size());
        if (std::isfinite(original[static_cast<std::size_t>(index)])) window.push_back(original[static_cast<std::size_t>(index)]);
      }
      // Never synthesize a return into a beam that was +inf/NaN in the raw scan.
      // Filling empty beams from neighbours creates fake walls and radial map noise.
      if (std::isfinite(original[i]) && !window.empty()) {
        const auto middle = window.begin() + static_cast<long>(window.size() / 2U);
        std::nth_element(window.begin(), middle, window.end());
        ranges[i] = *middle;
      } else if (!std::isfinite(original[i])) {
        ranges[i] = original[i];
      }
    }
  }
  std::vector<ScanPoint> points; points.reserve(ranges.size());
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r < std::max(min_range_, static_cast<double>(msg.range_min)) ||
      r > std::min(max_range_, static_cast<double>(msg.range_max))) continue;
    const double angle = msg.angle_min + static_cast<double>(i) * msg.angle_increment;
    points.push_back({r * std::cos(angle), r * std::sin(angle)});
  }
  return points;
}

bool HectorSLAMNode::detect_motion(const std::vector<ScanPoint> & prev, const std::vector<ScanPoint> & curr)
{
  if (prev.size() < static_cast<std::size_t>(min_valid_scan_points_) ||
    curr.size() < static_cast<std::size_t>(min_valid_scan_points_))
  {
    return false;  // low-information scans must never be allowed to move odometry
  }

  // Re-bin compacted Cartesian points back to fixed 1-degree bearings. The old
  // index-to-index comparison was unsafe because invalid LaserScan beams are
  // removed, so prev[i] and curr[i] often represented different angles.
  constexpr std::size_t kBins = 360U;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::array<double, kBins> prev_bins{};
  std::array<double, kBins> curr_bins{};
  prev_bins.fill(nan);
  curr_bins.fill(nan);

  auto rasterize = [&](const std::vector<ScanPoint> & points, std::array<double, kBins> & bins) {
      for (const auto & p : points) {
        const double range = std::hypot(p.x, p.y);
        if (!std::isfinite(range) || range < min_range_ || range > max_range_) {
          continue;
        }
        double deg = std::atan2(p.y, p.x) * 180.0 / kPi;
        if (deg < 0.0) deg += 360.0;
        int idx = static_cast<int>(std::lround(deg)) % 360;
        if (idx < 0) idx += 360;
        auto & slot = bins[static_cast<std::size_t>(idx)];
        if (!std::isfinite(slot) || range < slot) {
          slot = range;
        }
      }
    };

  rasterize(prev, prev_bins);
  rasterize(curr, curr_bins);

  std::vector<double> diffs;
  diffs.reserve(kBins);
  std::size_t changed = 0U;
  for (std::size_t i = 0; i < kBins; ++i) {
    if (!std::isfinite(prev_bins[i]) || !std::isfinite(curr_bins[i])) {
      continue;
    }
    const double diff = std::abs(curr_bins[i] - prev_bins[i]);
    if (diff > 1.0) {
      continue;  // reject gross outliers from the motion statistic
    }
    diffs.push_back(diff);
    if (diff > motion_changed_range_threshold_) {
      ++changed;
    }
  }

  if (diffs.size() < static_cast<std::size_t>(motion_min_overlap_)) {
    return false;  // insufficient common geometry -> hold pose instead of guessing
  }

  const auto mid = diffs.begin() + static_cast<long>(diffs.size() / 2U);
  std::nth_element(diffs.begin(), mid, diffs.end());
  const double median_diff = *mid;
  const double changed_ratio = static_cast<double>(changed) / static_cast<double>(diffs.size());

  return median_diff > motion_median_threshold_ ||
         changed_ratio > motion_changed_ratio_threshold_;
}

double HectorSLAMNode::compute_map_score(const std::vector<ScanPoint> & points, double dx, double dy, double dt)
{
  const double x = robot_x_ + dx, y = robot_y_ + dy, theta = robot_theta_ + dt;
  const double c = std::cos(theta), s = std::sin(theta); double score = 0.0; std::size_t count = 0U;
  for (std::size_t i = 0; i < points.size(); i += 3U) {
    const double wx = x + points[i].x * c - points[i].y * s;
    const double wy = y + points[i].x * s + points[i].y * c;
    const int gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    const int gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    if (gx >= 0 && gx < grid_width_ && gy >= 0 && gy < grid_height_ && observed_(gy, gx)) {
      score += log_odds_(gy, gx);
      ++count;
    }
  }
  return count == 0U ? 0.0 : score / static_cast<double>(count);
}

std::tuple<double, double, double> HectorSLAMNode::search_level(const std::vector<ScanPoint> & points,
  double xmin, double xmax, double ymin, double ymax, double tmin, double tmax, int steps)
{
  double best = -std::numeric_limits<double>::max();
  double best_motion_cost = std::numeric_limits<double>::max();
  double bx = 0.0, by = 0.0, bt = 0.0;
  for (int ix = 0; ix < steps; ++ix) {
    for (int iy = 0; iy < steps; ++iy) {
      for (int it = 0; it < steps; ++it) {
        const double f_x = steps == 1 ? 0.5 : static_cast<double>(ix) / (steps - 1);
        const double f_y = steps == 1 ? 0.5 : static_cast<double>(iy) / (steps - 1);
        const double f_t = steps == 1 ? 0.5 : static_cast<double>(it) / (steps - 1);
        const double dx = xmin + f_x * (xmax - xmin);
        const double dy = ymin + f_y * (ymax - ymin);
        const double dt = tmin + f_t * (tmax - tmin);
        const double score = compute_map_score(points, dx, dy, dt);
        const double motion_cost = std::hypot(dx, dy) + 0.25 * std::abs(dt);
        if (score > best + search_tie_epsilon_ ||
          (std::abs(score - best) <= search_tie_epsilon_ && motion_cost < best_motion_cost))
        {
          best = score;
          best_motion_cost = motion_cost;
          bx = dx;
          by = dy;
          bt = dt;
        }
      }
    }
  }
  return {bx, by, bt};
}

std::optional<Pose2D> HectorSLAMNode::multi_res_match(const std::vector<ScanPoint> & points)
{
  const double null_score = compute_map_score(points, 0.0, 0.0, 0.0);
  // Fine-grained hierarchy: the old 3x3x3 search quantized valid motion to the
  // edges of a coarse cube.  With sparse/noisy geometry this could look like a
  // repeated ~0.1 m / multi-degree jump.  Seven samples per axis plus two local
  // refinements keep the solution continuous while the tie-break still prefers
  // the smallest motion for equivalent scores.
  auto [dx, dy, dt] = search_level(
    points, -coarse_search_x_, coarse_search_x_,
    -coarse_search_y_, coarse_search_y_,
    -coarse_search_theta_, coarse_search_theta_, 7);
  const double refine_x = coarse_search_x_ / 3.0;
  const double refine_y = coarse_search_y_ / 3.0;
  const double refine_theta = coarse_search_theta_ / 3.0;
  std::tie(dx, dy, dt) = search_level(
    points, dx-refine_x, dx+refine_x, dy-refine_y, dy+refine_y,
    dt-refine_theta, dt+refine_theta, 7);
  std::tie(dx, dy, dt) = search_level(
    points, dx-0.012, dx+0.012, dy-0.012, dy+0.012,
    dt-0.00524, dt+0.00524, 7);
  const double best_score = compute_map_score(points, dx, dy, dt);
  const double magnitude = std::max(std::abs(null_score), std::abs(best_score));
  last_match_quality_ = magnitude > 0.001 ? std::clamp(1.0 + (best_score-null_score)/magnitude, 0.1, 5.0) : 0.5;
  if (last_match_quality_ < min_match_improvement_) return std::nullopt;
  if (std::hypot(dx, dy) > max_single_trans_ || std::abs(dt) > max_single_rot_) { ++jump_reject_count_; return std::nullopt; }
  if (delta_history_.size() >= 3U && std::hypot(dx, dy) > 0.02) {
    const double ax = std::accumulate(delta_history_.begin(), delta_history_.end(), 0.0, [](double v, const Pose2D & p){return v+p.x;}) / delta_history_.size();
    const double ay = std::accumulate(delta_history_.begin(), delta_history_.end(), 0.0, [](double v, const Pose2D & p){return v+p.y;}) / delta_history_.size();
    const double threshold = scan_count_ < 50 ? deviation_threshold_init_ : deviation_threshold_;
    if (std::hypot(dx-ax, dy-ay) > threshold && last_match_quality_ < 2.0) { ++jump_reject_count_; return std::nullopt; }
  }
  delta_history_.push_back({dx,dy,dt}); if (delta_history_.size() > kMaxDeltaHistorySize) delta_history_.pop_front();
  return Pose2D{dx,dy,dt};
}

std::vector<std::pair<int,int>> HectorSLAMNode::bresenham(int x0, int y0, int x1, int y1)
{
  std::vector<std::pair<int,int>> result; const int dx=std::abs(x1-x0), sx=x0<x1?1:-1, dy=-std::abs(y1-y0), sy=y0<y1?1:-1; int error=dx+dy;
  while (true) { result.emplace_back(x0,y0); if (x0==x1 && y0==y1) break; const int e2=2*error; if(e2>=dy){error+=dy;x0+=sx;} if(e2<=dx){error+=dx;y0+=sy;} }
  return result;
}

int HectorSLAMNode::update_map(const std::vector<ScanPoint> & points)
{
  const double c=std::cos(robot_theta_), s=std::sin(robot_theta_); const int rgx=static_cast<int>((robot_x_-origin_x_)/resolution_), rgy=static_cast<int>((robot_y_-origin_y_)/resolution_); int updated=0;
  for (const auto & p: points) {
    const int gx=static_cast<int>((robot_x_+p.x*c-p.y*s-origin_x_)/resolution_), gy=static_cast<int>((robot_y_+p.x*s+p.y*c-origin_y_)/resolution_);
    if(gx<0||gx>=grid_width_||gy<0||gy>=grid_height_) continue;
    auto ray=bresenham(rgx,rgy,gx,gy); if(!ray.empty()) ray.pop_back();
    for(const auto &[x,y]:ray) if(x>=0&&x<grid_width_&&y>=0&&y<grid_height_){log_odds_(y,x)=std::max(static_cast<float>(log_odds_min_),log_odds_(y,x)+static_cast<float>(log_odds_free_));observed_(y,x)=true;++updated;}
    log_odds_(gy,gx)=std::min(static_cast<float>(log_odds_max_),log_odds_(gy,gx)+static_cast<float>(log_odds_occ_)); observed_(gy,gx)=true; ++updated;
  }
  map_ready_=true; return updated;
}

void HectorSLAMNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const auto points = scan_to_points(*msg);
  if (points.size() < static_cast<std::size_t>(min_valid_scan_points_)) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[ANTI-STARBURST] Holding odometry: only %zu valid scan points (< %d)",
      points.size(), min_valid_scan_points_);
    smooth_dx_ = 0.0;
    smooth_dy_ = 0.0;
    smooth_dtheta_ = 0.0;
    motion_counter_ = 0;
    return;
  }

  const rclcpp::Time scan_stamp(msg->header.stamp);
  if (!first_scan_received_) {
    first_scan_received_ = true;
    first_scan_stamp_ = scan_stamp.nanoseconds() > 0 ? scan_stamp : now();
    prev_scan_cart_ = points;
    motion_reference_scan_ = points;
    motion_reference_stamp_ = first_scan_stamp_;
    motion_reference_age_ = 0;
    robot_x_ = get_parameter("robot_initial_x").as_double();
    robot_y_ = get_parameter("robot_initial_y").as_double();
    robot_theta_ = get_parameter("robot_initial_yaw").as_double();
    smooth_dx_ = 0.0;
    smooth_dy_ = 0.0;
    smooth_dtheta_ = 0.0;
    motion_counter_ = 0;
    stationary_counter_ = stationary_confirm_threshold_;
    confirmed_stationary_ = true;
    is_stationary_ = true;
    update_map(points);
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    last_update_theta_ = robot_theta_;
    publish_odometry_measurement(first_scan_stamp_, false);
    RCLCPP_INFO(get_logger(),
      "First scan accepted as zero-motion reference; LiDAR odometry pose reset to (%.3f, %.3f, %.1fdeg)",
      robot_x_, robot_y_, robot_theta_ * 180.0 / kPi);
    return;
  }

  const rclcpp::Time effective_stamp = scan_stamp.nanoseconds() > 0 ? scan_stamp : now();
  const double dt_scan = (effective_stamp - first_scan_stamp_).seconds();
  first_scan_stamp_ = effective_stamp;
  if (dt_scan <= 0.0) {
    ++jump_reject_count_;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "Rejecting non-monotonic LiDAR odometry scan timestamp (dt=%.6f)", dt_scan);
    return;
  }

  if (dt_scan > max_scan_gap_) {
    // Do not try to match a post-reconnect / delayed scan against stale state.
    // Re-seed from the new scan while keeping the current pose continuous.
    prev_scan_cart_ = points;
    motion_reference_scan_ = points;
    motion_reference_stamp_ = effective_stamp;
    motion_reference_age_ = 0;
    smooth_dx_ = 0.0;
    smooth_dy_ = 0.0;
    smooth_dtheta_ = 0.0;
    motion_counter_ = 0;
    stationary_counter_ = stationary_confirm_threshold_;
    confirmed_stationary_ = true;
    is_stationary_ = true;
    ++jump_reject_count_;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[ANTI-STARBURST] scan gap %.3f s > %.3f s: pose held and matcher re-seeded",
      dt_scan, max_scan_gap_);
    publish_odometry_measurement(effective_stamp, false);
    return;
  }

  ++scan_count_;
  if (scan_count_ > min_scans_before_matching_ && map_ready_) {
    if (motion_reference_scan_.empty()) {
      motion_reference_scan_ = points;
      motion_reference_stamp_ = effective_stamp;
      motion_reference_age_ = 0;
    }
    ++motion_reference_age_;
    const bool moved = detect_motion(motion_reference_scan_, points);

    if (moved) {
      motion_counter_ = std::min(motion_counter_ + 1, motion_confirm_threshold_);
      stationary_counter_ = 0;
      confirmed_stationary_ = false;
      is_stationary_ = false;
    } else {
      motion_counter_ = 0;
      ++stationary_counter_;
      confirmed_stationary_ = stationary_counter_ >= stationary_confirm_threshold_;
      is_stationary_ = confirmed_stationary_;
    }

    // A scan-matcher quality score alone is never permission to move the pose.
    // Pose updates require physical scan change for consecutive scans.
    if (!moved || motion_counter_ < motion_confirm_threshold_) {
      smooth_dx_ = 0.0;
      smooth_dy_ = 0.0;
      smooth_dtheta_ = 0.0;
      prev_scan_cart_ = points;
      // Keep the reference long enough for slow motion to accumulate. If the
      // scene stays effectively stationary, refresh it periodically to avoid
      // very old geometry/noise becoming a false movement trigger.
      if (!moved && motion_reference_age_ >= motion_reference_max_scans_) {
        motion_reference_scan_ = points;
        motion_reference_stamp_ = effective_stamp;
        motion_reference_age_ = 0;
      }
      publish_odometry_measurement(effective_stamp, false);
      return;
    }

    auto delta = multi_res_match(points);
    if (!delta) {
      smooth_dx_ = 0.0;
      smooth_dy_ = 0.0;
      smooth_dtheta_ = 0.0;
      prev_scan_cart_ = points;
      if (motion_reference_age_ >= motion_reference_max_scans_) {
        motion_reference_scan_ = points;
        motion_reference_stamp_ = effective_stamp;
        motion_reference_age_ = 0;
        motion_counter_ = 0;
      }
      publish_odometry_measurement(effective_stamp, false);
      return;
    }

    double dx = delta->x;
    double dy = delta->y;
    double dtheta = delta->theta;

    // A bad scan can look like a plausible small yaw at each frame and evade a
    // simple max-delta test, then accumulate into the classic radial/starburst
    // map. When the IMU is fresh we use gyro magnitude as a hard plausibility
    // gate. This does not integrate IMU orientation and therefore does not add
    // IMU yaw drift to the LiDAR odometry.
    if (use_imu_rotation_gate_ && imu_received_) {
      const double imu_age = std::abs((effective_stamp - last_imu_stamp_).seconds());
      if (imu_age <= imu_rotation_gate_timeout_) {
        const double gyro_abs = std::abs(last_imu_gyro_z_);
        if (gyro_abs < imu_stationary_gyro_threshold_) {
          if (std::abs(dtheta) > 1.0e-6) {
            ++imu_rotation_reject_count_;
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "[IMU-ROT-GATE] suppressing LiDAR yaw %.2fdeg while IMU gyro=%.4frad/s (stationary)",
              dtheta * 180.0 / kPi, last_imu_gyro_z_);
          }
          dtheta = 0.0;
        } else {
          const double max_imu_consistent_yaw =
            imu_rotation_margin_ + imu_rotation_scale_ * gyro_abs * dt_scan;
          if (std::abs(dtheta) > max_imu_consistent_yaw) {
            ++imu_rotation_reject_count_;
            RCLCPP_INFO_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "[IMU-ROT-GATE] clamping LiDAR yaw %.2fdeg -> %.2fdeg (gyro=%.3frad/s dt=%.3fs)",
              dtheta * 180.0 / kPi,
              std::copysign(max_imu_consistent_yaw, dtheta) * 180.0 / kPi,
              last_imu_gyro_z_, dt_scan);
            dtheta = std::copysign(max_imu_consistent_yaw, dtheta);
          }
        }
      }
    }

    const double delta_trans = std::hypot(dx, dy);
    const double delta_rot = std::abs(dtheta);
    // The candidate motion is measured since the held motion reference, so its
    // physical-rate check must use the same accumulated time window.
    double motion_dt = dt_scan;
    if (motion_reference_stamp_.nanoseconds() > 0) {
      motion_dt = std::max(dt_scan, (effective_stamp - motion_reference_stamp_).seconds());
    }

    if ((delta_trans / motion_dt) > max_vel_trans_ || (delta_rot / motion_dt) > max_vel_rot_) {
      ++jump_reject_count_;
      smooth_dx_ = 0.0;
      smooth_dy_ = 0.0;
      smooth_dtheta_ = 0.0;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Rejecting LiDAR odometry delta beyond physical limits: dxy=%.3f m, dyaw=%.2f deg, dt=%.3f s",
        delta_trans, delta_rot * 180.0 / kPi, motion_dt);
      prev_scan_cart_ = points;
      motion_reference_scan_ = points;
      motion_reference_stamp_ = effective_stamp;
      motion_reference_age_ = 0;
      motion_counter_ = 0;
      publish_odometry_measurement(effective_stamp, false);
      return;
    }

    const bool delta_small = delta_trans < deadband_dist_ && delta_rot < deadband_angle_;
    if (delta_small) {
      smooth_dx_ = 0.0;
      smooth_dy_ = 0.0;
      smooth_dtheta_ = 0.0;
    } else {
      smooth_dx_ = delta_ema_alpha_ * dx + (1.0 - delta_ema_alpha_) * smooth_dx_;
      smooth_dy_ = delta_ema_alpha_ * dy + (1.0 - delta_ema_alpha_) * smooth_dy_;
      smooth_dtheta_ = delta_ema_alpha_ * dtheta + (1.0 - delta_ema_alpha_) * smooth_dtheta_;
      robot_x_ += smooth_dx_;
      robot_y_ += smooth_dy_;
      robot_theta_ = normalize_angle(robot_theta_ + smooth_dtheta_);
      trajectory_.push_back({robot_x_, robot_y_, robot_theta_});
      if (trajectory_.size() > kMaxTrajectorySize) {
        trajectory_.pop_front();
      }
    }
    // A confirmed/processed motion establishes the next cumulative reference.
    motion_reference_scan_ = points;
    motion_reference_stamp_ = effective_stamp;
    motion_reference_age_ = 0;
    motion_counter_ = 0;
  }

  const double distance = std::hypot(robot_x_ - last_update_x_, robot_y_ - last_update_y_);
  const double angle = std::abs(normalize_angle(robot_theta_ - last_update_theta_));
  if (!map_ready_ || distance > map_update_distance_threshold_ || angle > map_update_angle_threshold_) {
    update_map(points);
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    last_update_theta_ = robot_theta_;
  }
  prev_scan_cart_ = points;
  publish_odometry_measurement(effective_stamp, scan_count_ > min_scans_before_matching_);
}

void HectorSLAMNode::publish_map()
{
  if (!publish_map_topic_) return;
  nav_msgs::msg::OccupancyGrid msg; msg.header.stamp=now();msg.header.frame_id="map";msg.info.resolution=resolution_;msg.info.width=grid_width_;msg.info.height=grid_height_;msg.info.origin.position.x=origin_x_;msg.info.origin.position.y=origin_y_;msg.info.origin.orientation.w=1.0;msg.data.resize(static_cast<std::size_t>(grid_width_*grid_height_),-1);
  if(map_ready_) for(int y=0;y<grid_height_;++y)for(int x=0;x<grid_width_;++x)if(observed_(y,x)){const double p=1.0/(1.0+std::exp(-log_odds_(y,x)));msg.data[static_cast<std::size_t>(y*grid_width_+x)]=static_cast<int8_t>(std::clamp(p*100.0,0.0,100.0));}
  map_pub_->publish(msg);
}

void HectorSLAMNode::publish_pose_tf()
{
  // This timer exists only for legacy visualization/TF modes.  Production
  // mapping and autonomous launches disable Hector TF ownership; EKF is the
  // sole owner of odom -> base_footprint.  /lidar/odom itself is published
  // synchronously from scan_callback() with the scan measurement timestamp.
  const auto stamp = now();
  const double qz = std::sin(robot_theta_ / 2.0);
  const double qw = std::cos(robot_theta_ / 2.0);

  if (publish_map_odom_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped map_odom;
    map_odom.header.stamp = stamp;
    map_odom.header.frame_id = "map";
    map_odom.child_frame_id = "odom";
    map_odom.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(map_odom);
  }
  if (publish_odom_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped odom_base;
    odom_base.header.stamp = stamp;
    odom_base.header.frame_id = "odom";
    odom_base.child_frame_id = odom_child_frame_;
    odom_base.transform.translation.x = robot_x_;
    odom_base.transform.translation.y = robot_y_;
    odom_base.transform.rotation.z = qz;
    odom_base.transform.rotation.w = qw;
    tf_broadcaster_->sendTransform(odom_base);
  }

  if (publish_pose_topic_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = robot_x_;
    pose.pose.position.y = robot_y_;
    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;
    pose_pub_->publish(pose);
  }
  publish_pose_text_and_quality();
}

void HectorSLAMNode::publish_odometry_measurement(
  const rclcpp::Time & stamp, bool scan_match_accepted)
{
  if (!publish_odom_topic_) {
    return;
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = odom_child_frame_;
  odom.pose.pose.position.x = robot_x_;
  odom.pose.pose.position.y = robot_y_;
  odom.pose.pose.orientation.z = std::sin(robot_theta_ / 2.0);
  odom.pose.pose.orientation.w = std::cos(robot_theta_ / 2.0);

  // Unobserved axes must not look perfectly known to downstream filters.
  odom.pose.covariance.fill(1.0e6);
  odom.twist.covariance.fill(1.0e6);

  // Convert the scan-matcher score into conservative measurement covariance.
  // A held/rejected match remains publishable for timestamp continuity, but it
  // receives deliberately weak pose covariance so EKF will prefer wheel/IMU.
  const double q_span = std::max(0.10, 2.0 - min_match_improvement_);
  const double confidence = scan_match_accepted
    ? std::clamp((last_match_quality_ - min_match_improvement_) / q_span, 0.0, 1.0)
    : 0.0;
  const double xy_var = scan_match_accepted
    ? (0.040 * (1.0 - confidence) + 0.0025 * confidence)
    : 0.25;
  const double yaw_var = scan_match_accepted
    ? (0.030 * (1.0 - confidence) + 0.0012 * confidence)
    : 0.12;
  const double linear_var = scan_match_accepted
    ? (0.090 * (1.0 - confidence) + 0.010 * confidence)
    : 0.50;
  const double angular_var = scan_match_accepted
    ? (0.100 * (1.0 - confidence) + 0.015 * confidence)
    : 0.50;

  odom.pose.covariance[0] = xy_var;
  odom.pose.covariance[7] = xy_var;
  odom.pose.covariance[35] = yaw_var;
  odom.twist.covariance[0] = linear_var;
  odom.twist.covariance[7] = linear_var * 2.0;
  odom.twist.covariance[35] = angular_var;

  if (odom_state_initialized_) {
    const double dt = (stamp - last_odom_stamp_).seconds();
    if (dt > 1.0e-4) {
      const double dx_world = robot_x_ - last_odom_x_;
      const double dy_world = robot_y_ - last_odom_y_;
      const double dyaw = normalize_angle(robot_theta_ - last_odom_theta_);
      const bool delta_small =
        std::hypot(dx_world, dy_world) < quality_gate_significant_dist_ &&
        std::abs(dyaw) < quality_gate_significant_angle_;

      if (!scan_match_accepted || confirmed_stationary_ || is_stationary_ || delta_small) {
        odom.twist.twist.linear.x = 0.0;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0.0;
      } else {
        const double vx_world = dx_world / dt;
        const double vy_world = dy_world / dt;
        const double c = std::cos(robot_theta_);
        const double s = std::sin(robot_theta_);
        // nav_msgs/Odometry twist is expressed in child_frame_id.
        odom.twist.twist.linear.x = c * vx_world + s * vy_world;
        odom.twist.twist.linear.y = -s * vx_world + c * vy_world;
        odom.twist.twist.angular.z = dyaw / dt;
      }
    }
  }

  last_odom_stamp_ = stamp;
  last_odom_x_ = robot_x_;
  last_odom_y_ = robot_y_;
  last_odom_theta_ = robot_theta_;
  odom_state_initialized_ = true;
  odom_pub_->publish(odom);
}

void HectorSLAMNode::publish_trajectory(){nav_msgs::msg::Path msg;msg.header.stamp=now();msg.header.frame_id="map";msg.poses.reserve(trajectory_.size());for(const auto&p:trajectory_){geometry_msgs::msg::PoseStamped pose;pose.header=msg.header;pose.pose.position.x=p.x;pose.pose.position.y=p.y;pose.pose.orientation.z=std::sin(p.theta/2.0);pose.pose.orientation.w=std::cos(p.theta/2.0);msg.poses.push_back(pose);}traj_pub_->publish(msg);}
void HectorSLAMNode::publish_pose_text_and_quality(){std_msgs::msg::String text;text.data=std::string(confirmed_stationary_?"STATIONARY":"MOVING")+" | X:"+std::to_string(robot_x_)+" Y:"+std::to_string(robot_y_)+" yaw:"+std::to_string(robot_theta_*180.0/kPi);pose_text_pub_->publish(text);std_msgs::msg::Float32 q;q.data=static_cast<float>(last_match_quality_);quality_pub_->publish(q);}
void HectorSLAMNode::publish_map_callback(){if(map_ready_)publish_map();}void HectorSLAMNode::publish_pose_tf_callback(){publish_pose_tf();}void HectorSLAMNode::publish_trajectory_callback(){publish_trajectory();}
void HectorSLAMNode::log_stats_callback(){RCLCPP_INFO(get_logger(),"[%s] scans=%d quality=%.2f pose=(%.3f,%.3f,%.1fdeg) jumps=%d imu_yaw_reject=%d",confirmed_stationary_?"STATIONARY":"MOVING",scan_count_,last_match_quality_,robot_x_,robot_y_,robot_theta_*180.0/kPi,jump_reject_count_,imu_rotation_reject_count_);}
double HectorSLAMNode::normalize_angle(double a){return std::atan2(std::sin(a),std::cos(a));}
std::array<double,4> HectorSLAMNode::euler_to_quaternion(double r,double p,double y){const double cy=std::cos(y/2),sy=std::sin(y/2),cp=std::cos(p/2),sp=std::sin(p/2),cr=std::cos(r/2),sr=std::sin(r/2);return{sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy};}
}  // namespace c1_slam

int main(int argc,char **argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<c1_slam::HectorSLAMNode>());rclcpp::shutdown();return 0;}
