#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <serial/serial.h>

#include <memory>
#include <vector>
#include <string>
#include <chrono>

class ImuNode : public rclcpp::Node
{
public:
  explicit ImuNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ImuNode() override;

private:
  void openSerial(bool initial = false);
  void closeSerial();
  void pollSerial();
  void publishImu();
  void publishMag();
  void publishRaw(const std::vector<uint8_t> & packet);
  bool parsePacket(const std::vector<uint8_t> & packet);
  bool configureSensorOutput(bool persistent = false);
  bool writeSensorRegister(uint8_t address, uint16_t value);
  bool logRateLimited(const std::string & msg, const std::string & level = "warn");
  std::vector<std::string> detectPort();
  static void quatFromEuler(double roll, double pitch, double yaw,
                            double & qx, double & qy, double & qz, double & qw);

  // Parameters
  std::string port_;
  int baudrate_ = 115200;  // fixed runtime baudrate
  std::string frame_id_ = "imu_link";
  bool debug_ = false;
  bool auto_detect_ = false;
  bool publish_raw_ = false;
  int poll_interval_ms_ = 10;
  int publish_rate_hz_ = 50;
  // Self-heal stream Yahboom/WitMotion. Konfigurasi hanya memastikan konten
  // output/rate; tidak mengubah kalibrasi, axis, atau baud sensor.
  bool configure_output_on_connect_ = true;
  bool persist_output_config_ = false;
  int output_content_mask_ = 0x001E;  // ACC + GYRO + ANGLE + MAG
  int output_rate_code_ = 0x08;       // 50 Hz stream profile at 115200 baud
  double orientation_packet_timeout_sec_ = 2.0;
  double sensor_config_retry_sec_ = 30.0;
  double orientation_reopen_sec_ = 20.0;
  // Konversi sumbu sensor ke REP-103/ENU. yaw_sign mengubah orientasi dan
  // angular_velocity.z secara bersamaan agar EKF tidak menerima dua tanda yang
  // saling bertentangan. invert_yaw dipertahankan hanya untuk kompatibilitas.
  bool publish_orientation_ = true;
  bool invert_roll_ = false;
  bool invert_pitch_ = false;
  bool invert_yaw_ = false;
  double yaw_sign_ = 1.0;
  double roll_offset_rad_ = 0.0;
  double pitch_offset_rad_ = 0.0;
  double yaw_offset_rad_ = 0.0;
  double magnetic_declination_rad_ = 0.0;
  double mag_scale_tesla_per_lsb_ = 1e-7;
  std::vector<double> accel_bias_{0.0, 0.0, 0.0};
  std::vector<double> gyro_bias_{0.0, 0.0, 0.0};
  std::vector<double> orientation_covariance_{0.001, 0.001, 0.003};
  std::vector<double> angular_velocity_covariance_{0.005, 0.005, 0.005};
  std::vector<double> linear_acceleration_covariance_{0.01, 0.01, 0.01};

  // Serial
  std::unique_ptr<serial::Serial> ser_;
  std::string active_port_;
  serial::Timeout ser_timeout_;
  std::vector<uint8_t> buf_;
  // Cross-process lock shared with lidar_node. This closes the tiny race where
  // two processes open the same tty before either TIOCEXCL has taken effect.
  int port_lock_fd_ = -1;

  // IMU state
  double roll_ = 0.0, pitch_ = 0.0, yaw_ = 0.0;
  double ax_ = 0.0, ay_ = 0.0, az_ = 0.0;
  double gx_ = 0.0, gy_ = 0.0, gz_ = 0.0;
  double mx_ = 0.0, my_ = 0.0, mz_ = 0.0;
  bool has_angle_ = false;
  bool has_acc_ = false;
  bool has_gyro_ = false;
  bool has_mag_ = false;
  std::chrono::steady_clock::time_point last_publish_time_{};

  // Stats / stream health
  size_t bytes_received_ = 0;
  size_t packets_parsed_ = 0;
  size_t packets_acc_ = 0;
  size_t packets_gyro_ = 0;
  size_t packets_angle_ = 0;
  size_t packets_mag_ = 0;
  size_t packets_quat_ = 0;
  double last_orientation_packet_time_ = 0.0;
  double last_sensor_config_try_ = 0.0;
  bool stream_announced_ = false;
  bool orientation_recovery_attempted_ = false;
  double orientation_recovery_started_time_ = 0.0;
  double last_status_time_ = 0.0;
  double last_reconnect_try_ = 0.0;
  double last_data_time_ = 0.0;
  double data_timeout_sec_ = 6.0;
  const double reconnect_interval_sec_ = 2.0;
  double error_suppress_until_ = 0.0;
  const double error_suppress_window_ = 3.0;
  int consecutive_serial_errors_ = 0;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_gyro_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_accel_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr pub_raw_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_connected_;
  rclcpp::TimerBase::SharedPtr timer_;

};

#endif // IMU_NODE_HPP
