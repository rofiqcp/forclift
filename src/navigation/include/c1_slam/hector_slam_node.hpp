#ifndef C1_SLAM__HECTOR_SLAM_NODE_HPP_
#define C1_SLAM__HECTOR_SLAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace c1_slam
{

struct ScanPoint {
  double x{0.0};
  double y{0.0};
};

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

class HectorSLAMNode : public rclcpp::Node
{
public:
  explicit HectorSLAMNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Parameters
  void declare_parameters();
  void load_parameters();

  // Callbacks
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publish_map_callback();
  void publish_pose_tf_callback();
  void publish_trajectory_callback();
  void log_stats_callback();

  // Core SLAM
  std::vector<ScanPoint> scan_to_points(const sensor_msgs::msg::LaserScan & msg);
  bool detect_motion(const std::vector<ScanPoint> & prev, const std::vector<ScanPoint> & curr);
  std::optional<Pose2D> multi_res_match(const std::vector<ScanPoint> & scan_points);
  double compute_map_score(const std::vector<ScanPoint> & scan_points, double dx, double dy, double dtheta);
  std::tuple<double, double, double> search_level(
    const std::vector<ScanPoint> & scan_points,
    double x_min, double x_max,
    double y_min, double y_max,
    double t_min, double t_max,
    int steps);

  int update_map(const std::vector<ScanPoint> & scan_points);
  std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1);

  void publish_map();
  void publish_pose_tf();
  void publish_odometry_measurement(const rclcpp::Time & stamp, bool scan_match_accepted);
  void publish_trajectory();
  void publish_pose_text_and_quality();

  static double normalize_angle(double angle);
  static std::array<double, 4> euler_to_quaternion(double roll, double pitch, double yaw);

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr quality_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_text_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timers
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  rclcpp::TimerBase::SharedPtr traj_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // Map state
  double map_width_m_{40.0};
  double map_height_m_{40.0};
  double resolution_{0.025};
  double max_range_{8.0};
  double min_range_{0.15};
  bool scan_filtering_{true};
  int scan_filter_window_{7};
  double log_odds_free_{-0.15};
  double log_odds_occ_{0.35};
  double map_update_distance_threshold_{0.05};
  double map_update_angle_threshold_{0.035};
  double min_match_improvement_{0.30};
  bool publish_odom_tf_{false};
  bool publish_odom_topic_{true};
  bool publish_map_odom_tf_{false};
  std::string odom_child_frame_{"base_footprint"};
  std::string scan_topic_{"/scan_nav"};
  bool publish_map_topic_{true};
  bool publish_pose_topic_{true};

  int grid_width_{0};
  int grid_height_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double log_odds_max_{10.0};
  double log_odds_min_{-8.0};

  Eigen::MatrixXf log_odds_;
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> observed_;
  bool map_ready_{false};

  // Robot pose
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_theta_{0.0};

  // Scan matching state
  std::vector<ScanPoint> prev_scan_cart_;
  // Motion reference is intentionally held across several scans so slow AGV
  // motion can accumulate above the motion gate instead of being reset every frame.
  std::vector<ScanPoint> motion_reference_scan_;
  rclcpp::Time motion_reference_stamp_{0, 0, RCL_ROS_TIME};
  int motion_reference_age_{0};
  int motion_reference_max_scans_{12};
  int scan_count_{0};
  int min_scans_before_matching_{3};
  std::deque<Pose2D> trajectory_;
  double last_match_quality_{0.0};

  // Drift reduction parameters
  double delta_ema_alpha_{0.4};
  double stationary_dist_threshold_{0.015};
  int stationary_confirm_threshold_{8};
  double deadband_dist_{0.01};
  double deadband_angle_{0.0174533};  // 1 deg in rad
  double quality_gate_threshold_{0.45};
  double quality_gate_significant_dist_{0.02};
  double quality_gate_significant_angle_{0.0523599};  // 3 deg in rad
  double max_vel_trans_{0.5};
  double max_vel_rot_{1.0};
  double coarse_search_x_{0.15};
  double coarse_search_y_{0.15};
  double coarse_search_theta_{0.0698132};  // 4 deg in rad
  double max_single_trans_{0.8};
  double max_single_rot_{0.261799};  // 15 deg in rad
  double deviation_threshold_{0.25};
  double deviation_threshold_init_{0.5};

  // Anti-starburst scan validation / motion gating
  int min_valid_scan_points_{50};
  int motion_confirm_threshold_{2};
  int motion_min_overlap_{35};
  double motion_median_threshold_{0.020};
  double motion_changed_range_threshold_{0.060};
  double motion_changed_ratio_threshold_{0.22};
  double max_scan_gap_{0.30};
  double search_tie_epsilon_{1.0e-6};

  // Cross-sensor rotation sanity gate. LiDAR scan matching supplies X/Y, but a
  // false rotational match is the main mechanism that turns one bad scan into
  // a radial/starburst map. A fresh IMU gyro sample is therefore used only as
  // a plausibility bound for per-scan yaw; IMU yaw is not integrated here.
  bool use_imu_rotation_gate_{true};
  double imu_rotation_gate_timeout_{0.30};
  double imu_stationary_gyro_threshold_{0.035};
  double imu_rotation_margin_{0.0139626};  // 0.8 deg
  double imu_rotation_scale_{1.8};
  double last_imu_gyro_z_{0.0};
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
  bool imu_received_{false};
  int imu_rotation_reject_count_{0};

  // Drift reduction state
  int stationary_counter_{0};
  int motion_counter_{0};
  bool confirmed_stationary_{false};
  bool is_stationary_{false};
  double smooth_dx_{0.0};
  double smooth_dy_{0.0};
  double smooth_dtheta_{0.0};
  double last_update_x_{0.0};
  double last_update_y_{0.0};
  double last_update_theta_{0.0};
  double last_update_time_{0.0};
  std::deque<Pose2D> pose_buffer_;
  double last_pose_time_{0.0};
  std::deque<Pose2D> delta_history_;
  int jump_reject_count_{0};

  // First scan handling
  bool first_scan_received_{false};
  rclcpp::Time first_scan_stamp_{0, 0, RCL_ROS_TIME};

  // Scan-synchronous odometry state.  /lidar/odom is stamped with the
  // LaserScan measurement time and its twist is computed only from successive
  // scan measurements, never from the wall timer used for visualization.
  bool odom_state_initialized_{false};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  double last_odom_x_{0.0};
  double last_odom_y_{0.0};
  double last_odom_theta_{0.0};
};

}  // namespace c1_slam

#endif  // C1_SLAM__HECTOR_SLAM_NODE_HPP_