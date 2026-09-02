#include "imu_ros2/wt901_imu_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <climits>
#include <thread>
#include <fstream>
#include <sstream>

// === AGV HARDWARE CONSTANTS ===
// Physical USB paths (by-path) for deterministic device binding.
// Both use CP2102 (10c4:ea60) with duplicate serial "0001" — by-id is ambiguous.
//
// ACTUAL topology (Aug 2026): Bus001/Hub(1-2)/Hub(1-2.1)/IMU(1-2.1.1)/LiDAR(1-2.1.2)
// Corrected from 2.3.x -> 2.1.x after sysfs audit confirmed devpath=2.1.1 and 2.1.2.
static constexpr const char * IMU_PHYSICAL_PATH =
    "/dev/serial/by-path/platform-3610000.usb-usb-0:2.1.1:1.0-port0";
static constexpr const char * LIDAR_PHYSICAL_PATH =
    "/dev/serial/by-path/platform-3610000.usb-usb-0:2.1.2:1.0-port0";

namespace imu_ros2
{

WT901IMUNode::WT901IMUNode(const rclcpp::NodeOptions & options)
: Node("imu_node", options)
{
  declare_parameters();
  load_parameters();

  // === AGV HARDWARE COLLISION CHECK ===
  // Verify IMU and LiDAR by-path symlinks resolve to different devices.
  // Both CP2102 adapters carry the same serial "0001" making by-id ambiguous.
  char imu_resolved[PATH_MAX], lidar_resolved[PATH_MAX];
  bool imu_exists = (realpath(IMU_PHYSICAL_PATH, imu_resolved) != nullptr);
  bool lidar_exists = (realpath(LIDAR_PHYSICAL_PATH, lidar_resolved) != nullptr);

  if (imu_exists && lidar_exists) {
    if (std::string(imu_resolved) == std::string(lidar_resolved)) {
      RCLCPP_FATAL(get_logger(),
        "FATAL: IMU and LiDAR resolved to the same device %s — refusing to start",
        imu_resolved);
      throw std::runtime_error("Serial port collision detected between IMU and LiDAR");
    }
    RCLCPP_INFO(get_logger(),
      "Collision check passed: IMU=%s, LiDAR=%s (different devices)",
      imu_resolved, lidar_resolved);
  } else {
    RCLCPP_WARN(get_logger(),
      "Collision check skipped (one or both paths missing): IMU=%s, LiDAR=%s",
      imu_exists ? imu_resolved : "(missing)",
      lidar_exists ? lidar_resolved : "(missing)");
  }

  // === AGV USB DEVICE MAP LOGGING ===
  RCLCPP_INFO(get_logger(), "========== AGV USB DEVICE MAP ==========");
  std::ifstream by_path_index("/dev/serial/by-path");
  if (by_path_index.is_open()) {
    std::string line;
    while (std::getline(by_path_index, line)) {
      if (!line.empty()) {
        char resolved[PATH_MAX];
        if (realpath(line.c_str(), resolved)) {
          RCLCPP_INFO(get_logger(), "  %s -> %s", line.c_str(), resolved);
        } else {
          RCLCPP_INFO(get_logger(), "  %s -> (broken symlink)", line.c_str());
        }
      }
    }
    by_path_index.close();
  } else {
    RCLCPP_WARN(get_logger(), "  /dev/serial/by-path not accessible");
  }
  RCLCPP_INFO(get_logger(), "==========================================");

  initialize_components();
  if (!try_connect()) {
    RCLCPP_INFO(get_logger(), "[IMU-RECOVERY] initial connect pending; automatic reconnect armed");
  }
}

WT901IMUNode::~WT901IMUNode()
{
  if (driver_) {
    driver_->disconnect();
  }
}

void WT901IMUNode::declare_parameters()
{
  declare_parameter("port", std::string(IMU_PHYSICAL_PATH));
  declare_parameter("baudrate", 115200);
  declare_parameter("frame_id", std::string("imu_link"));
  declare_parameter("publish_rate", 50);
  declare_parameter("use_ahrs_euler", true);
  declare_parameter("gyro_range", 2000.0);
  declare_parameter("accel_range", 16.0);
  declare_parameter("calibration_file", std::string(""));
}

void WT901IMUNode::load_parameters()
{
  port_ = get_parameter("port").as_string();
  baudrate_ = get_parameter("baudrate").as_int();
  frame_id_ = get_parameter("frame_id").as_string();
  publish_rate_ = get_parameter("publish_rate").as_int();
  use_ahrs_ = get_parameter("use_ahrs_euler").as_bool();
  gyro_range_ = get_parameter("gyro_range").as_double();
  accel_range_ = get_parameter("accel_range").as_double();
  calibration_file_ = get_parameter("calibration_file").as_string();

  RCLCPP_INFO(get_logger(),
              "IMU Node initialized: port=%s, rate=%dHz, frame=%s",
              port_.c_str(), publish_rate_, frame_id_.c_str());
}

void WT901IMUNode::initialize_components()
{
  parser_ = IMUParser(accel_range_, gyro_range_);
  filter_ = IMUFilter();

  // QoS
  auto qos = rclcpp::QoS(10).reliable();

  // Publishers
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", qos);
  gyro_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/gyro", qos);
  accel_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/accel", qos);
  mag_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/mag", qos);
  euler_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/euler", qos);
  status_pub_ = create_publisher<std_msgs::msg::String>("/imu/status", qos);
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/imu_pose_marker", 10);

  // Subscribe to /pose for marker position
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        robot_x_ = msg->pose.position.x;
        robot_y_ = msg->pose.position.y;
      });

  // Timer
  const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = create_wall_timer(period, [this] { publish_callback(); });
}

bool WT901IMUNode::detect_imu_port()
{
  // No longer performs aggressive auto-detection across all CP2102 ports.
  // Port is hardwired via IMU_PHYSICAL_PATH / declare_parameter default.
  // This function is kept only to satisfy the existing call site in try_connect()
  // when the parameter is explicitly set to "auto" (should not happen in normal use).
  if (port_.empty()) {
    RCLCPP_ERROR(get_logger(), "Port is empty — IMU_PHYSICAL_PATH not set or not found");
    return false;
  }
  if (!SerialPort::path_exists(port_)) {
    RCLCPP_ERROR(get_logger(), "Configured IMU port does not exist: %s", port_.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "IMU port verified: %s", port_.c_str());
  return true;
}

bool WT901IMUNode::try_connect()
{
  // === HARDWIRED BY-PATH MODE ===
  // IMU port is configured via IMU_PHYSICAL_PATH (default parameter).
  // No aggressive auto-detect. We verify the path exists and log resolution.
  if (port_.empty() || port_ == "auto") {
    RCLCPP_ERROR(get_logger(), "Port is 'auto' — auto-detection has been disabled. "
      "Set port parameter to a by-path value.");
    return false;
  }

  // Check if by-path symlink exists
  if (!SerialPort::path_exists(port_)) {
    RCLCPP_INFO(get_logger(),
      "[IMU-WAIT] IMU by-path not ready: %s — waiting for device to appear",
      port_.c_str());
    return false;
  }

  // Resolve and log the actual tty device
  char resolved_port[PATH_MAX];
  if (realpath(port_.c_str(), resolved_port)) {
    // Symlink resolves: check if the underlying tty actually exists
    if (SerialPort::path_exists(resolved_port)) {
      RCLCPP_INFO(get_logger(),
        "[IMU-PORT] by-path %s -> %s [OK]", port_.c_str(), resolved_port);
    } else {
      // Symlink exists but points to a ttyUSB that no longer exists (stale symlink).
      // This happens when ttyUSB numbers shift after USB re-enumeration.
      // Clear stale state and report waiting.
      RCLCPP_WARN(get_logger(),
        "[IMU-STALE] by-path %s -> %s [BROKEN — tty no longer exists]",
        port_.c_str(), resolved_port);
      return false;
    }
  } else {
    // realpath() failed — by-path symlink itself is broken (target gone)
    RCLCPP_WARN(get_logger(),
      "[IMU-STALE] by-path realpath failed for %s: %s — waiting for device",
      port_.c_str(), std::strerror(errno));
    return false;
  }

  RCLCPP_INFO(get_logger(), "[IMU-CONNECT] Opening IMU on %s @ %d...", resolved_port, baudrate_);
  driver_ = std::make_unique<IMUDriver>(port_, baudrate_, 0.1);
  if (driver_->connect()) {
    connected_.store(true);
    reconnect_attempts_ = 0;
    RCLCPP_INFO(get_logger(), "[IMU-OK] IMU connected on %s", resolved_port);
    return true;
  }

  RCLCPP_ERROR(get_logger(), "[IMU-FAIL] Failed to connect to IMU on %s", resolved_port);
  connected_.store(false);
  return false;
}

void WT901IMUNode::reconnect()
{
  if (connected_.load()) return;

  ++reconnect_attempts_;
  RCLCPP_INFO(get_logger(), "[IMU-RECONNECT] attempt %d", reconnect_attempts_);

  if (driver_) {
    driver_->disconnect();
    driver_.reset();
  }

  try_connect();
}

bool WT901IMUNode::read_and_parse()
{
  if (!connected_.load() || !driver_) {
    return false;
  }

  bool processed = false;

  // Read all available bytes into persistent buffer
  int waiting = driver_->in_waiting();
  if (waiting > 0) {
    auto data = driver_->read_raw(waiting);
    if (!data.empty()) {
      rx_buffer_.insert(rx_buffer_.end(), data.begin(), data.end());
    } else if (!driver_->is_connected()) {
      // USB unplugged / fatal IO
      connected_.store(false);
      return false;
    }
  }

  // Parse buffer for complete WT901 packets
  while (rx_buffer_.size() >= 11) {
    // Find sync byte 0x55
    size_t sync_pos = 0;
    while (sync_pos < rx_buffer_.size() && rx_buffer_[sync_pos] != 0x55) {
      ++sync_pos;
    }

    if (sync_pos > 0) {
      // Remove garbage before sync
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sync_pos);
      continue;
    }

    if (rx_buffer_.size() < 11) {
      break;  // Wait for more data
    }

    // Check if we have a valid packet type
    uint8_t type = rx_buffer_[1];
    if (type != 0x51 && type != 0x52 && type != 0x53 && type != 0x54) {
      // Invalid type - drop 1 byte and resync
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    // Verify checksum
    uint8_t expected = (0x55 + type);
    for (size_t i = 0; i < 8; ++i) {
      expected = (expected + rx_buffer_[2 + i]) & 0xFF;
    }
    if (expected != rx_buffer_[10]) {
      // Bad checksum - drop 1 byte and resync
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    // Valid packet - extract and process
    std::array<uint8_t, IMUParser::kPacketSize> packet{};
    std::copy(rx_buffer_.begin(), rx_buffer_.begin() + 11, packet.begin());
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 11);

    auto pkt_type = parser_.parse_packet(packet);
    if (pkt_type) {
      processed = true;
      const uint8_t * payload_ptr = packet.data() + 2;
      handle_parsed_packet(*pkt_type, payload_ptr);
      last_valid_packet_ = std::chrono::steady_clock::now();
    }
  }

  // Prevent unbounded buffer growth on corrupted stream
  if (rx_buffer_.size() > 4096) {
    rx_buffer_.clear();
  }

  return processed;
}

void WT901IMUNode::handle_parsed_packet(PacketType type, const uint8_t * payload)
{
  switch (type) {
    case PacketType::Accel: {
      auto data = parser_.parse_accel(payload, accel_range_);
      accel_.x = data.x; accel_.y = data.y; accel_.z = data.z;
      temperature_ = data.temperature;
      break;
    }
    case PacketType::Gyro: {
      auto data = parser_.parse_gyro(payload, gyro_range_);
      gyro_.x = data.x; gyro_.y = data.y; gyro_.z = data.z;
      break;
    }
    case PacketType::Angle: {
      angle_ = parser_.parse_angle(payload);
      break;
    }
    case PacketType::Mag: {
      auto data = parser_.parse_mag(payload);
      mag_.x = data.x; mag_.y = data.y; mag_.z = data.z;
      break;
    }
  }
}

void WT901IMUNode::publish_imu_messages()
{
  auto now = this->get_clock()->now();

  // Apply calibration
  Vector3 accel_cal, gyro_cal, mag_cal;
  filter_.apply(accel_, gyro_, mag_, accel_cal, gyro_cal, mag_cal);

  // Main IMU message
  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
  imu_msg->header.stamp = now;
  imu_msg->header.frame_id = frame_id_;

  auto q = IMUParser::euler_to_quaternion(angle_.roll, angle_.pitch, angle_.yaw);
  imu_msg->orientation.x = q[0];
  imu_msg->orientation.y = q[1];
  imu_msg->orientation.z = q[2];
  imu_msg->orientation.w = q[3];
  imu_msg->orientation_covariance = orientation_cov_;

  imu_msg->angular_velocity.x = gyro_cal.x;
  imu_msg->angular_velocity.y = gyro_cal.y;
  imu_msg->angular_velocity.z = gyro_cal.z;
  imu_msg->angular_velocity_covariance = angular_vel_cov_;

  imu_msg->linear_acceleration.x = accel_cal.x;
  imu_msg->linear_acceleration.y = accel_cal.y;
  imu_msg->linear_acceleration.z = accel_cal.z;
  imu_msg->linear_acceleration_covariance = linear_acc_cov_;

  imu_pub_->publish(std::move(imu_msg));

  // Euler
  auto euler_msg = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
  euler_msg->header.stamp = now;
  euler_msg->header.frame_id = frame_id_;
  euler_msg->vector.x = angle_.roll;
  euler_msg->vector.y = angle_.pitch;
  euler_msg->vector.z = angle_.yaw;
  euler_pub_->publish(std::move(euler_msg));

  // Magnetometer
  auto mag_msg = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
  mag_msg->header.stamp = now;
  mag_msg->header.frame_id = frame_id_;
  mag_msg->vector.x = mag_cal.x;
  mag_msg->vector.y = mag_cal.y;
  mag_msg->vector.z = mag_cal.z;
  mag_pub_->publish(std::move(mag_msg));

  // Gyro only
  auto gyro_msg = std::make_unique<sensor_msgs::msg::Imu>();
  gyro_msg->header.stamp = now;
  gyro_msg->header.frame_id = frame_id_;
  gyro_msg->angular_velocity.x = gyro_cal.x;
  gyro_msg->angular_velocity.y = gyro_cal.y;
  gyro_msg->angular_velocity.z = gyro_cal.z;
  gyro_msg->angular_velocity_covariance = angular_vel_cov_;
  gyro_pub_->publish(std::move(gyro_msg));

  // Accel only
  auto accel_msg = std::make_unique<sensor_msgs::msg::Imu>();
  accel_msg->header.stamp = now;
  accel_msg->header.frame_id = frame_id_;
  accel_msg->linear_acceleration.x = accel_cal.x;
  accel_msg->linear_acceleration.y = accel_cal.y;
  accel_msg->linear_acceleration.z = accel_cal.z;
  accel_msg->linear_acceleration_covariance = linear_acc_cov_;
  accel_pub_->publish(std::move(accel_msg));

  // Status
  auto status_msg = std::make_unique<std_msgs::msg::String>();
  const char * status_str = connected_.load() ? "CONNECTED" : "DISCONNECTED";
  status_msg->data = std::string(status_str) + " | Port: " + port_ +
                     " | R:" + std::to_string(angle_.roll * 180.0 / M_PI) +
                     " P:" + std::to_string(angle_.pitch * 180.0 / M_PI) +
                     " Y:" + std::to_string(angle_.yaw * 180.0 / M_PI);
  status_pub_->publish(std::move(status_msg));

  // Marker
  publish_marker(now, q);
}

void WT901IMUNode::publish_marker(const rclcpp::Time & now,
                                  const std::array<double, 4> & q)
{
  // Validasi quaternion: harus finite dan non-zero norm
  double q_norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (q_norm < 1e-6 || !std::isfinite(q_norm)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Invalid quaternion, skipping marker publish");
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = now;
  // Publish in base_link so TF places the arrow on the robot.
  // Do NOT use /pose XY (can diverge / be wrong frame).
  marker.header.frame_id = "base_link";
  marker.ns = "wt901_imu_heading";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.15;
  // Normalisasi quaternion
  marker.pose.orientation.x = q[0] / q_norm;
  marker.pose.orientation.y = q[1] / q_norm;
  marker.pose.orientation.z = q[2] / q_norm;
  marker.pose.orientation.w = q[3] / q_norm;
  marker.scale.x = 0.4;
  marker.scale.y = 0.06;
  marker.scale.z = 0.1;
  // Warna CYAN opaque
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);
  marker_pub_->publish(marker);
}

void WT901IMUNode::publish_callback()
{
  try {
    if (connected_.load()) {
      read_and_parse();
    }

    if (connected_.load()) {
      // Stale detection: no valid packet for >1.0s => don't publish stale data
      bool fresh = last_valid_packet_.time_since_epoch().count() == 0 ||
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_valid_packet_).count() < 1.0;

      if (!fresh) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "IMU stream stale: no valid WT901 packet for >1.0s");
        connected_.store(false);
        if (driver_) {
          driver_->disconnect();
          driver_.reset();
        }
        auto status_msg = std::make_unique<std_msgs::msg::String>();
        status_msg->data = "STALE | Re-detecting IMU...";
        status_pub_->publish(std::move(status_msg));
        return;
      }

      publish_imu_messages();
    } else {
      auto status_msg = std::make_unique<std_msgs::msg::String>();
      status_msg->data = "DISCONNECTED | Attempting reconnect...";
      status_pub_->publish(std::move(status_msg));
    }

  // Reconnect logic - time-based (every reconnect_interval_ seconds)
  static auto last_reconnect = std::chrono::steady_clock::now() - std::chrono::seconds(10);
  if (!connected_.load()) {
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_reconnect).count() >= reconnect_interval_) {
      last_reconnect = now;
      reconnect();
    }
  }
} catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Callback error: %s", e.what());
    connected_.store(false);
  }
}

}  // namespace imu_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<imu_ros2::WT901IMUNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}