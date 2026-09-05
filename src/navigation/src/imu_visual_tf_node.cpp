#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.hpp>

class ImuVisualTfNode : public rclcpp::Node
{
public:
  ImuVisualTfNode()
  : Node("imu_visual_tf_node")
  {
    imu_stationary_enter_rad_s_ = declare_parameter("imu_stationary_enter_rad_s", 0.020);
    imu_stationary_exit_rad_s_ = declare_parameter("imu_stationary_exit_rad_s", 0.060);
    stationary_confirm_s_ = declare_parameter("stationary_confirm_s", 0.20);
    esc_stationary_linear_mps_ = declare_parameter("esc_stationary_linear_mps", 0.030);
    esc_stationary_angular_rad_s_ = declare_parameter("esc_stationary_angular_rad_s", 0.050);
    esc_timeout_s_ = declare_parameter("esc_timeout_s", 0.60);
    lidar_stationary_linear_mps_ = declare_parameter("lidar_stationary_linear_mps", 0.025);
    lidar_stationary_angular_rad_s_ = declare_parameter("lidar_stationary_angular_rad_s", 0.050);
    position_filter_alpha_ = std::clamp(declare_parameter("position_filter_alpha", 0.20), 0.05, 1.0);
    yaw_filter_alpha_ = std::clamp(declare_parameter("yaw_filter_alpha", 0.25), 0.05, 1.0);
    visual_position_deadband_m_ = declare_parameter("visual_position_deadband_m", 0.0080);
    visual_yaw_deadband_rad_ = declare_parameter("visual_yaw_deadband_rad", 0.0040);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/lidar/odom");
    visual_parent_frame_ = declare_parameter<std::string>("visual_parent_frame", "odom");
    visual_child_frame_ = declare_parameter<std::string>(
      "visual_child_frame", "visual_/base_footprint");
    direct_imu_yaw_ = declare_parameter("direct_imu_yaw", false);
    visual_model_yaw_offset_rad_ = declare_parameter("visual_model_yaw_offset_rad", 0.0);

    const auto qos = rclcpp::SensorDataQoS();
    lidar_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) { handle_lidar(message); });
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", qos,
      [this](const sensor_msgs::msg::Imu::SharedPtr message) { handle_imu(message); });
    esc_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/esc/odom", qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) { handle_esc(message); });

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(std::chrono::milliseconds(33), [this]() { publish_transform(); });

    RCLCPP_INFO(
      get_logger(),
      "Visual TF active: odom=%s parent=%s child=%s IMU_yaw=%s hold=%.3f/%.3f rad/s",
      odom_topic_.c_str(), visual_parent_frame_.c_str(), visual_child_frame_.c_str(),
      direct_imu_yaw_ ? "DIRECT" : "ANTI-JITTER",
      imu_stationary_enter_rad_s_, imu_stationary_exit_rad_s_);
  }

private:
  static bool valid_quaternion(const double x, const double y, const double z, const double w)
  {
    const double norm_squared = x * x + y * y + z * z + w * w;
    return std::isfinite(norm_squared) && norm_squared > 1.0e-12;
  }

  static double normalize_angle(const double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double yaw(const tf2::Quaternion & orientation)
  {
    return std::atan2(
      2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
      1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));
  }

  static double blend_angle(const double from, const double to, const double alpha)
  {
    return normalize_angle(from + alpha * normalize_angle(to - from));
  }

  void handle_lidar(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    const auto & orientation = message->pose.pose.orientation;
    if (!valid_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)) {
      return;
    }

    lidar_x_ = message->pose.pose.position.x;
    lidar_y_ = message->pose.pose.position.y;
    lidar_vx_ = message->twist.twist.linear.x;
    lidar_vy_ = message->twist.twist.linear.y;
    lidar_wz_ = message->twist.twist.angular.z;

    if (!lidar_valid_) {
      lidar_initial_ = tf2::Quaternion(
        orientation.x, orientation.y, orientation.z, orientation.w);
      lidar_initial_.normalize();
      lidar_valid_ = true;
      align_if_ready();
    }
  }

  void handle_imu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    const auto & orientation = message->orientation;
    if (!valid_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)) {
      return;
    }

    imu_current_ = tf2::Quaternion(
      orientation.x, orientation.y, orientation.z, orientation.w);
    imu_current_.normalize();
    imu_yaw_ = yaw(imu_current_);

    const double gx = message->angular_velocity.x;
    const double gy = message->angular_velocity.y;
    const double gz = message->angular_velocity.z;
    const double gyro_norm = std::sqrt(gx * gx + gy * gy + gz * gz);
    update_imu_stationary_state(gyro_norm, now());

    if (!imu_valid_) {
      imu_initial_ = imu_current_;
      imu_valid_ = true;
      align_if_ready();
    }
  }

  void handle_esc(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    esc_vx_ = message->twist.twist.linear.x;
    esc_wz_ = message->twist.twist.angular.z;
    last_esc_stamp_ = now();
    esc_valid_ = true;
  }

  void update_imu_stationary_state(const double gyro_norm, const rclcpp::Time & stamp)
  {
    if (!std::isfinite(gyro_norm)) {
      return;
    }

    if (imu_stationary_) {
      if (gyro_norm > imu_stationary_exit_rad_s_) {
        imu_stationary_ = false;
        quiet_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      }
      return;
    }

    if (gyro_norm <= imu_stationary_enter_rad_s_) {
      if (quiet_since_.nanoseconds() == 0) {
        quiet_since_ = stamp;
      } else if ((stamp - quiet_since_).seconds() >= stationary_confirm_s_) {
        imu_stationary_ = true;
      }
    } else {
      quiet_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
  }

  void align_if_ready()
  {
    if (aligned_ || !lidar_valid_ || !imu_valid_) {
      return;
    }

    yaw_offset_ = normalize_angle(yaw(lidar_initial_) - yaw(imu_initial_));
    visual_yaw_ = normalize_angle(yaw_offset_ + imu_yaw_);
    visual_x_ = lidar_x_;
    visual_y_ = lidar_y_;
    lidar_offset_x_ = visual_x_ - lidar_x_;
    lidar_offset_y_ = visual_y_ - lidar_y_;
    visual_initialized_ = true;
    aligned_ = true;
  }

  bool chassis_stationary() const
  {
    const auto current = now();
    if (esc_valid_ && last_esc_stamp_.nanoseconds() > 0 &&
      (current - last_esc_stamp_).seconds() <= esc_timeout_s_)
    {
      return std::abs(esc_vx_) <= esc_stationary_linear_mps_ &&
             std::abs(esc_wz_) <= esc_stationary_angular_rad_s_;
    }

    // Safe fallback when ESC odometry is unavailable. Require both LiDAR twist
    // and IMU rotation to be quiet so straight-line motion is not frozen merely
    // because the IMU angular velocity is near zero.
    return imu_stationary_ &&
           std::hypot(lidar_vx_, lidar_vy_) <= lidar_stationary_linear_mps_ &&
           std::abs(lidar_wz_) <= lidar_stationary_angular_rad_s_;
  }

  void publish_transform()
  {
    if (!visual_initialized_) {
      return;
    }

    // Yaw ownership is always the hardware IMU.  Autonomous V21 uses DIRECT
    // mode so the visual URDF follows every real IMU yaw change immediately.
    // Mapping may keep anti-jitter mode, where stationary AHRS drift is absorbed
    // into the offset instead of visibly shaking a parked RobotModel.
    if (direct_imu_yaw_) {
      const double target_yaw = normalize_angle(yaw_offset_ + imu_yaw_);
      if (std::abs(normalize_angle(target_yaw - visual_yaw_)) > visual_yaw_deadband_rad_) {
        visual_yaw_ = blend_angle(visual_yaw_, target_yaw, yaw_filter_alpha_);
      }
    } else if (imu_stationary_) {
      yaw_offset_ = normalize_angle(visual_yaw_ - imu_yaw_);
    } else {
      const double target_yaw = normalize_angle(yaw_offset_ + imu_yaw_);
      if (std::abs(normalize_angle(target_yaw - visual_yaw_)) > visual_yaw_deadband_rad_) {
        visual_yaw_ = blend_angle(visual_yaw_, target_yaw, yaw_filter_alpha_);
      }
    }

    // Translation still follows LiDAR odometry during real chassis motion, but
    // when ESC feedback says the vehicle is stopped we absorb LiDAR scan-matcher
    // wander into an offset. Therefore hundreds of tiny LiDAR pose corrections
    // cannot make a stationary URDF shake or slide on the RViz grid.
    if (chassis_stationary()) {
      lidar_offset_x_ = visual_x_ - lidar_x_;
      lidar_offset_y_ = visual_y_ - lidar_y_;
    } else {
      const double target_x = lidar_x_ + lidar_offset_x_;
      const double target_y = lidar_y_ + lidar_offset_y_;
      const double dx = target_x - visual_x_;
      const double dy = target_y - visual_y_;
      if (std::hypot(dx, dy) > visual_position_deadband_m_) {
        visual_x_ += position_filter_alpha_ * dx;
        visual_y_ += position_filter_alpha_ * dy;
      }
    }

    tf2::Quaternion visual_orientation;
    visual_orientation.setRPY(
      0.0, 0.0, normalize_angle(visual_yaw_ + visual_model_yaw_offset_rad_));
    visual_orientation.normalize();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = visual_parent_frame_;
    transform.child_frame_id = visual_child_frame_;
    transform.transform.translation.x = visual_x_;
    transform.transform.translation.y = visual_y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = visual_orientation.x();
    transform.transform.rotation.y = visual_orientation.y();
    transform.transform.rotation.z = visual_orientation.z();
    transform.transform.rotation.w = visual_orientation.w();
    broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr esc_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2::Quaternion lidar_initial_;
  tf2::Quaternion imu_initial_;
  tf2::Quaternion imu_current_;

  double imu_yaw_{0.0};
  double yaw_offset_{0.0};
  double visual_yaw_{0.0};

  double lidar_x_{0.0};
  double lidar_y_{0.0};
  double lidar_vx_{0.0};
  double lidar_vy_{0.0};
  double lidar_wz_{0.0};
  double lidar_offset_x_{0.0};
  double lidar_offset_y_{0.0};
  double visual_x_{0.0};
  double visual_y_{0.0};

  double esc_vx_{0.0};
  double esc_wz_{0.0};

  double imu_stationary_enter_rad_s_{0.020};
  double imu_stationary_exit_rad_s_{0.060};
  double stationary_confirm_s_{0.20};
  double esc_stationary_linear_mps_{0.030};
  double esc_stationary_angular_rad_s_{0.050};
  double esc_timeout_s_{0.60};
  double lidar_stationary_linear_mps_{0.025};
  double lidar_stationary_angular_rad_s_{0.050};
  double position_filter_alpha_{0.20};
  double yaw_filter_alpha_{0.25};
  double visual_position_deadband_m_{0.0080};
  double visual_yaw_deadband_rad_{0.0040};
  double visual_model_yaw_offset_rad_{0.0};

  std::string odom_topic_{"/lidar/odom"};
  std::string visual_parent_frame_{"odom"};
  std::string visual_child_frame_{"visual_/base_footprint"};
  bool direct_imu_yaw_{false};

  rclcpp::Time quiet_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_esc_stamp_{0, 0, RCL_ROS_TIME};

  bool lidar_valid_{false};
  bool imu_valid_{false};
  bool esc_valid_{false};
  bool aligned_{false};
  bool visual_initialized_{false};
  bool imu_stationary_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuVisualTfNode>());
  rclcpp::shutdown();
  return 0;
}
