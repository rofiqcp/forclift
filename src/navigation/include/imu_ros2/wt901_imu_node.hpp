#pragma once

#include "imu_ros2/imu_driver.hpp"
#include "imu_ros2/imu_parser.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <mutex>
#include <thread>
#include <vector>

namespace imu_ros2
{

class WT901IMUNode : public rclcpp::Node
{
public:
  WT901IMUNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~WT901IMUNode() override;

protected:
  void load_parameters();
  void declare_parameters();
  void initialize_components();
  bool detect_imu_port();
  bool try_connect();
  void reconnect();
  bool read_and_parse();
  void handle_parsed_packet(PacketType type, const uint8_t * payload);
  void publish_imu_messages();
  void publish_marker(const rclcpp::Time & stamp,
                       const std::array<double, 4> & q);
  void publish_callback();

private:
  // State
  std::unique_ptr<IMUDriver> driver_;
  IMUParser parser_;
  IMUFilter filter_;

  // Parameters
  std::string port_;
  int baudrate_{115200};
  std::string frame_id_{"imu_link"};
  int publish_rate_{50};
  bool use_ahrs_{true};
  double gyro_range_{2000.0};
  double accel_range_{16.0};
  std::string calibration_file_;  // deprecated compatibility field; biases below are authoritative
  std::vector<double> accel_bias_mps2_{0.0, 0.0, 0.0};
  std::vector<double> gyro_bias_rps_{0.0, 0.0, 0.0};
  std::vector<double> mag_bias_t_{0.0, 0.0, 0.0};
  std::vector<double> mag_soft_iron_{1.0, 0.0, 0.0,
                                     0.0, 1.0, 0.0,
                                     0.0, 0.0, 1.0};
  double component_timeout_sec_{0.25};
  double component_sync_tolerance_sec_{0.04};
  std::vector<double> orientation_cov_diag_{0.01, 0.01, 0.02};
  std::vector<double> angular_vel_cov_diag_{0.001, 0.001, 0.001};
  std::vector<double> linear_acc_cov_diag_{0.01, 0.01, 0.01};

  // Sensor data
  AccelData accel_;
  GyroData gyro_;
  AngleData angle_;
  MagData mag_;
  double temperature_{25.0};

  // Per-packet freshness. WT901 sends accel/gyro/angle/mag as independent
  // frames; a composite /imu/data message is valid only when required
  // components are recent.
  std::chrono::steady_clock::time_point last_accel_packet_{};
  std::chrono::steady_clock::time_point last_gyro_packet_{};
  std::chrono::steady_clock::time_point last_angle_packet_{};
  std::chrono::steady_clock::time_point last_mag_packet_{};
  rclcpp::Time last_accel_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gyro_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_angle_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mag_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_composite_stamp_{0, 0, RCL_ROS_TIME};

  // Covariance
  std::array<double, 9> orientation_cov_{0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02};
  std::array<double, 9> angular_vel_cov_{0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001};
  std::array<double, 9> linear_acc_cov_{0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

  // Reconnect logic
  int reconnect_attempts_{0};
  double reconnect_interval_{0.5};
  double stream_timeout_sec_{0.8};
  double first_packet_timeout_sec_{2.0};
  std::chrono::steady_clock::time_point last_valid_packet_{};
  std::chrono::steady_clock::time_point connected_since_{};
  std::chrono::steady_clock::time_point last_reconnect_attempt_{};
  std::vector<uint8_t> rx_buffer_;

  // Pose for marker
  double robot_x_{0.0};
  double robot_y_{0.0};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gyro_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_field_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<bool> connected_{false};

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

}  // namespace imu_ros2