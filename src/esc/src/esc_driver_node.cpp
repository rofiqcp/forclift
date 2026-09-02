#include "esc/serial_board.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

using std::placeholders::_1;

namespace {
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kRpmToRadPerSec = kTwoPi / 60.0;

rclcpp::QoS state_qos() {
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}
}  // namespace

/*
 * Driver ESC ROS 2 untuk hardware STM32 dual-motor.
 *
 * Kontrak dengan Nav2/navigation:
 *   input  : /cmd_vel/actuator (hasil arbitration/safety gate esc_command_mux)
 *   output : /esc/odom, /esc/speed, /esc/armed, /esc/ready,
 *            /esc/drive/connected, /esc/steer/connected, /esc/status
 *
 * Command dari Nav2 tetap diterima dan diringkas ke /esc/status saat ESC offline.
 * Untuk tahap mapping sebelum ESC terhubung, offline_zero_output=true membuat topic
 * numerik ESC tetap terbit sebagai 0 agar downstream node punya nilai deterministik.
 * Placeholder nol TIDAK dinyatakan sebagai feedback nyata: /esc/feedback_valid,
 * /esc/ready, dan status connected tetap false sampai telemetry hardware valid.
 *
 * Topologi:
 * 1) ackermann_1_board    : motor kiri SPD (drive), motor kanan POS (steering)
 * 2) ackermann_2_board    : board-0 dua SPD, board-1 kiri POS steering
 * 3) differential_1_board : kiri/kanan SPD independen
 */
class EscDriverNode : public rclcpp::Node {
 public:
  EscDriverNode() : Node("esc_driver") {
    vehicle_mode_ = declare_parameter<std::string>("vehicle_mode", "differential_1_board");
    board0_port_ = declare_parameter<std::string>("board0_port", "/dev/esc");
    board1_port_ = declare_parameter<std::string>("board1_port", "");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel/actuator");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/esc/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    wheelbase_ = declare_parameter<double>("wheelbase", 0.70);
    track_width_ = declare_parameter<double>("track_width", 0.55);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.145);
    wheel_ticks_per_revolution_ = declare_parameter<double>("wheel_ticks_per_revolution", 90.0);
    speed_setpoint_rpm_per_mps_ = declare_parameter<double>("speed_setpoint_rpm_per_mps", 0.0);
    max_motor_setpoint_rpm_ = declare_parameter<double>("max_motor_setpoint_rpm", 1000.0);

    steering_ticks_per_rad_ = declare_parameter<double>("steering_ticks_per_rad", 1000.0);
    steering_zero_ticks_ = declare_parameter<double>("steering_zero_ticks", 0.0);
    max_steering_rad_ = declare_parameter<double>("max_steering_rad", 0.34);
    min_speed_for_steering_mps_ = declare_parameter<double>("min_speed_for_steering_mps", 0.08);
    steering_calibrated_ = declare_parameter<bool>("steering_calibrated", false);
    require_steering_calibration_for_motion_ = declare_parameter<bool>(
      "require_steering_calibration_for_motion", true);

    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.25);
    publish_joint_states_ = declare_parameter<bool>("publish_joint_states", false);
    // Bench/mapping mode before ESC hardware is connected: publish explicit zero
    // telemetry while keeping feedback_valid/connected/ready FALSE.
    offline_zero_output_ = declare_parameter<bool>("offline_zero_output", true);
    drive_sign_ = declare_parameter<double>("drive_sign", 1.0);
    left_sign_ = declare_parameter<double>("left_sign", 1.0);
    right_sign_ = declare_parameter<double>("right_sign", 1.0);
    steering_sign_ = declare_parameter<double>("steering_sign", 1.0);

    if (vehicle_mode_ != "ackermann_1_board" &&
        vehicle_mode_ != "ackermann_2_board" &&
        vehicle_mode_ != "differential_1_board") {
      RCLCPP_WARN(get_logger(), "vehicle_mode '%s' tidak dikenal; memakai differential_1_board",
                  vehicle_mode_.c_str());
      vehicle_mode_ = "differential_1_board";
    }

    if (wheelbase_ <= 1e-6) wheelbase_ = 0.70;
    if (track_width_ <= 1e-6) track_width_ = 0.55;
    if (wheel_radius_ <= 1e-6) wheel_radius_ = 0.145;
    if (wheel_ticks_per_revolution_ <= 0.0) wheel_ticks_per_revolution_ = 90.0;
    // motorSpeed firmware adalah RPM mekanik. Bila parameter nol, hitung RPM/(m/s)
    // langsung dari radius roda: rpm = v * 60 / (2*pi*r).
    if (speed_setpoint_rpm_per_mps_ <= 0.0) {
      speed_setpoint_rpm_per_mps_ = 60.0 / (kTwoPi * wheel_radius_);
    }
    if (std::abs(steering_ticks_per_rad_) < 1e-9) steering_ticks_per_rad_ = 1000.0;
    if (std::abs(steering_sign_) < 1e-9) steering_sign_ = 1.0;

    boards_[0] = std::make_unique<esc::SerialBoard>(board0_port_);
    boards_[1] = std::make_unique<esc::SerialBoard>(board1_port_);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(10).reliable(),
        std::bind(&EscDriverNode::on_cmd, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));
    speed_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/speed", rclcpp::QoS(1).best_effort());
    drive_target_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/drive_target_mps", rclcpp::QoS(1).best_effort());
    drive_actual_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/drive_actual_mps", rclcpp::QoS(1).best_effort());
    steering_target_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/steering_target_rad", rclcpp::QoS(1).best_effort());
    steering_actual_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/steering_actual_rad", rclcpp::QoS(1).best_effort());
    steering_raw_ticks_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/steering_raw_ticks", rclcpp::QoS(1).best_effort());
    yaw_rate_actual_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/yaw_rate_actual_rps", rclcpp::QoS(1).best_effort());
    feedback_valid_pub_ = create_publisher<std_msgs::msg::Bool>("/esc/feedback_valid", state_qos());
    drive_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/esc/drive/connected", state_qos());
    steer_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/esc/steer/connected", state_qos());
    status_pub_ = create_publisher<std_msgs::msg::String>("/esc/status", state_qos());
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/esc/ready", state_qos());
    armed_pub_ = create_publisher<std_msgs::msg::Bool>("/esc/armed", state_qos());
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/esc/battery", 10);
    temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>("/esc/temperature", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               std::bind(&EscDriverNode::tick, this));

    RCLCPP_INFO(get_logger(),
                "ESC Protocol v%u | hardware mode=%s port0=%s port1=%s cmd=%s odom=%s; hot-plug aktif, offline-zero=%s",
                static_cast<unsigned>(esc::kVersion),
                vehicle_mode_.c_str(), board0_port_.c_str(),
                board1_port_.empty() ? "-" : board1_port_.c_str(),
                cmd_vel_topic_.c_str(), odom_topic_.c_str(),
                offline_zero_output_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(),
                "ESC calibration: wheel_r=%.3fm hall=%.1f tick/rev speed_gain=%.3f RPM/(m/s) steer=%.1f tick/rad zero=%.1f",
                wheel_radius_, wheel_ticks_per_revolution_, speed_setpoint_rpm_per_mps_,
                steering_ticks_per_rad_, steering_zero_ticks_);
    RCLCPP_INFO(get_logger(), "ESC closed-loop feedback: steering_calibrated=%s min_speed_for_steering=%.3f m/s",
                steering_calibrated_ ? "YES" : "NO", min_speed_for_steering_mps_);
  }

 private:
  struct Desired {
    std::uint8_t mode_left{esc::OPEN};
    std::uint8_t mode_right{esc::OPEN};
    std::int32_t setpoint_left{0};
    std::int32_t setpoint_right{0};
  };

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr message) {
    if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "cmd_vel actuator mengandung NaN/Inf; command diabaikan");
      return;
    }
    last_twist_ = *message;
    last_cmd_ = now();
    received_cmd_ = true;
  }

  double steering_angle() const {
    const double velocity = last_twist_.linear.x;
    const double yaw_rate = last_twist_.angular.z;
    if (std::abs(velocity) < min_speed_for_steering_mps_) {
      // Ackermann tidak dapat pure rotation. Saat v mendekati nol, jangan pernah
      // menafsirkan yaw-rate sebagai sudut steering. Tahan sudut aktual bila
      // feedback nyata tersedia; jika tidak, gunakan center (0 rad).
      double actual = 0.0;
      return steering_feedback_rad(actual) ? actual : 0.0;
    }
    return std::clamp(std::atan(wheelbase_ * yaw_rate / velocity),
                      -max_steering_rad_, max_steering_rad_);
  }

  std::int32_t clamp_motor_setpoint(double value) const {
    const double limit = std::max(0.0, max_motor_setpoint_rpm_);
    return static_cast<std::int32_t>(std::lround(std::clamp(value, -limit, limit)));
  }

  Desired desired_for(int board_index, bool neutral) const {
    Desired desired;
    const double velocity = neutral ? 0.0 : last_twist_.linear.x;
    const double yaw_rate = neutral ? 0.0 : last_twist_.angular.z;
    const std::int32_t drive = clamp_motor_setpoint(
        velocity * speed_setpoint_rpm_per_mps_ * drive_sign_);

    if (vehicle_mode_ == "ackermann_1_board") {
      if (board_index == 0) {
        desired.mode_left = esc::SPD;
        desired.mode_right = esc::POS;
        desired.setpoint_left = drive;
        const double angle = neutral ? 0.0 : steering_angle();
        desired.setpoint_right = static_cast<std::int32_t>(std::lround(
            steering_zero_ticks_ + steering_sign_ * angle * steering_ticks_per_rad_));
      }
    } else if (vehicle_mode_ == "ackermann_2_board") {
      if (board_index == 0) {
        desired.mode_left = esc::SPD;
        desired.mode_right = esc::SPD;
        desired.setpoint_left = drive;
        desired.setpoint_right = drive;
      } else {
        desired.mode_left = esc::POS;
        desired.mode_right = esc::OPEN;
        const double angle = neutral ? 0.0 : steering_angle();
        desired.setpoint_left = static_cast<std::int32_t>(std::lround(
            steering_zero_ticks_ + steering_sign_ * angle * steering_ticks_per_rad_));
      }
    } else if (board_index == 0) {
      const double left_velocity = velocity - yaw_rate * track_width_ / 2.0;
      const double right_velocity = velocity + yaw_rate * track_width_ / 2.0;
      desired.mode_left = esc::SPD;
      desired.mode_right = esc::SPD;
      desired.setpoint_left = clamp_motor_setpoint(
          left_velocity * speed_setpoint_rpm_per_mps_ * left_sign_);
      desired.setpoint_right = clamp_motor_setpoint(
          right_velocity * speed_setpoint_rpm_per_mps_ * right_sign_);
    }
    return desired;
  }

  Desired safe_for(int board_index) const {
    Desired desired = desired_for(board_index, true);
    const auto &board = *boards_[board_index];
    if (board.has_basic()) {
      const auto &telemetry = board.basic();
      if (desired.mode_left == esc::POS) desired.setpoint_left = telemetry.position_left;
      if (desired.mode_right == esc::POS) desired.setpoint_right = telemetry.position_right;
    }
    return desired;
  }

  void send_control(int board_index, const Desired &desired, bool arm) {
    auto frame = esc::command(esc::CONTROL);
    frame.mode_left = desired.mode_left;
    frame.mode_right = desired.mode_right;
    frame.setpoint_left = desired.setpoint_left;
    frame.setpoint_right = desired.setpoint_right;
    frame.flags = arm ? esc::FLAG_ARM : 0;
    (void)boards_[board_index]->send(frame);
  }

  void service_board(int board_index, bool command_fresh) {
    auto &board = *boards_[board_index];
    (void)board.poll();

    if (!board.connected()) {
      arm_stage_[board_index] = 0;
      seen_generation_[board_index] = 0;
      return;
    }

    if (seen_generation_[board_index] != board.connection_generation()) {
      seen_generation_[board_index] = board.connection_generation();
      arm_stage_[board_index] = 0;
    }

    const Desired safe = safe_for(board_index);
    if (!command_fresh) {
      send_control(board_index, safe, false);
      arm_stage_[board_index] = 0;
      return;
    }

    if (!board.has_basic()) {
      send_control(board_index, desired_for(board_index, true), false);
      return;
    }

    if (board.error_left() != 0U || board.error_right() != 0U) {
      auto frame = esc::command(esc::DISARM);
      (void)board.send(frame);
      arm_stage_[board_index] = 0;
      return;
    }

    switch (arm_stage_[board_index]) {
      case 0:
        send_control(board_index, safe, false);
        arm_stage_[board_index] = 1;
        break;
      case 1:
        send_control(board_index, safe, true);
        arm_stage_[board_index] = 2;
        break;
      case 2:
        send_control(board_index, safe, true);
        if ((board.status() & esc::STATUS_ARMED) != 0U) arm_stage_[board_index] = 3;
        break;
      default:
        if ((board.status() & esc::STATUS_ARMED) == 0U) {
          arm_stage_[board_index] = 0;
        } else {
          send_control(board_index, desired_for(board_index, false), true);
        }
        break;
    }
  }

  bool drive_feedback_ready() const {
    return boards_[0]->connected() && boards_[0]->has_basic();
  }

  bool steer_feedback_ready() const {
    if (vehicle_mode_ == "ackermann_2_board") {
      return boards_[1]->connected() && boards_[1]->has_basic();
    }
    return boards_[0]->connected() && boards_[0]->has_basic();
  }

  double wheel_ticks_to_rad(std::int32_t ticks, double sign) const {
    return sign * static_cast<double>(ticks) * kTwoPi / wheel_ticks_per_revolution_;
  }

  double steering_ticks_to_rad(std::int32_t ticks) const {
    return (static_cast<double>(ticks) - steering_zero_ticks_) /
           (steering_sign_ * steering_ticks_per_rad_);
  }

  bool steering_feedback_rad(double & steering_rad) const {
    if (!steer_feedback_ready()) return false;
    if (vehicle_mode_ == "ackermann_1_board") {
      steering_rad = steering_ticks_to_rad(boards_[0]->basic().position_right);
    } else if (vehicle_mode_ == "ackermann_2_board") {
      steering_rad = steering_ticks_to_rad(boards_[1]->basic().position_left);
    } else {
      steering_rad = 0.0;
    }
    steering_rad = std::clamp(steering_rad, -max_steering_rad_, max_steering_rad_);
    return std::isfinite(steering_rad);
  }

  double drive_target_mps(const Desired & target0) const {
    if (speed_setpoint_rpm_per_mps_ <= 1.0e-9) return 0.0;
    if (vehicle_mode_ == "differential_1_board") {
      const double left = static_cast<double>(target0.setpoint_left) /
        (speed_setpoint_rpm_per_mps_ * left_sign_);
      const double right = static_cast<double>(target0.setpoint_right) /
        (speed_setpoint_rpm_per_mps_ * right_sign_);
      return 0.5 * (left + right);
    }
    return static_cast<double>(target0.setpoint_left) /
      (speed_setpoint_rpm_per_mps_ * drive_sign_);
  }

  double steering_target_rad(const Desired & target0, const Desired & target1) const {
    if (vehicle_mode_ == "ackermann_1_board") {
      return std::clamp(steering_ticks_to_rad(target0.setpoint_right),
                        -max_steering_rad_, max_steering_rad_);
    }
    if (vehicle_mode_ == "ackermann_2_board") {
      return std::clamp(steering_ticks_to_rad(target1.setpoint_left),
                        -max_steering_rad_, max_steering_rad_);
    }
    return 0.0;
  }

  /* Mengambil kecepatan kendaraan dan yaw-rate MURNI dari telemetry ESC nyata. */
  bool vehicle_feedback(double &linear_mps, double &yaw_rate_rps, double &steering_rad) const {
    if (!drive_feedback_ready()) return false;

    const auto &t0 = boards_[0]->basic();
    steering_rad = 0.0;

    if (vehicle_mode_ == "differential_1_board") {
      const double wl = static_cast<double>(t0.speed_left) * kRpmToRadPerSec * left_sign_;
      const double wr = static_cast<double>(t0.speed_right) * kRpmToRadPerSec * right_sign_;
      const double vl = wl * wheel_radius_;
      const double vr = wr * wheel_radius_;
      linear_mps = 0.5 * (vl + vr);
      yaw_rate_rps = (vr - vl) / track_width_;
      return true;
    }

    double wheel_omega = 0.0;
    if (vehicle_mode_ == "ackermann_1_board") {
      wheel_omega = static_cast<double>(t0.speed_left) * kRpmToRadPerSec * drive_sign_;
      steering_rad = steering_ticks_to_rad(t0.position_right);
    } else {
      wheel_omega = 0.5 * static_cast<double>(t0.speed_left + t0.speed_right) *
                    kRpmToRadPerSec * drive_sign_;
      if (!steer_feedback_ready()) return false;
      steering_rad = steering_ticks_to_rad(boards_[1]->basic().position_left);
    }

    steering_rad = std::clamp(steering_rad, -max_steering_rad_, max_steering_rad_);
    linear_mps = wheel_omega * wheel_radius_;
    yaw_rate_rps = linear_mps * std::tan(steering_rad) / wheelbase_;
    return std::isfinite(linear_mps) && std::isfinite(yaw_rate_rps);
  }

  void publish_odom_and_speed() {
    double linear_mps = 0.0;
    double yaw_rate_rps = 0.0;
    double steering_rad = 0.0;
    const auto stamp = now();

    const bool feedback_ok = vehicle_feedback(linear_mps, yaw_rate_rps, steering_rad);
    if (!feedback_ok) {
      odom_time_valid_ = false;
      if (!offline_zero_output_) {
        return;
      }

      // User-requested bench state: before ESC hardware exists, all motion
      // feedback is explicitly ZERO. This is not declared valid feedback:
      // /esc/feedback_valid and connected/ready remain false in publish_state().
      // Preserve the last integrated pose across a temporary USB/ESC dropout.
      // Resetting pose to the origin creates an artificial discontinuity in raw
      // wheel odometry and invalidates distance/calibration measurements.
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = odom_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose.position.x = odom_x_;
      odom.pose.pose.position.y = odom_y_;
      odom.pose.pose.orientation.z = std::sin(0.5 * odom_yaw_);
      odom.pose.pose.orientation.w = std::cos(0.5 * odom_yaw_);
      odom.twist.twist.linear.x = 0.0;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.angular.z = 0.0;
      // Large covariance prevents accidental EKF use from treating the offline
      // zero placeholder as a real wheel-odometry measurement.
      odom.pose.covariance[0] = 1.0e6;
      odom.pose.covariance[7] = 1.0e6;
      odom.pose.covariance[35] = 1.0e6;
      odom.twist.covariance[0] = 1.0e6;
      odom.twist.covariance[7] = 1.0e6;
      odom.twist.covariance[35] = 1.0e6;
      odom_pub_->publish(odom);

      std_msgs::msg::Float64 zero;
      zero.data = 0.0;
      speed_pub_->publish(zero);
      drive_actual_pub_->publish(zero);
      steering_actual_pub_->publish(zero);
      yaw_rate_actual_pub_->publish(zero);
      return;
    }

    // A 50 Hz wall timer must not turn one ESC telemetry frame into several
    // synthetic odometry measurements. Integrate only when a new BASIC frame
    // from the drive board has actually been parsed.
    const std::uint64_t drive_generation = boards_[0]->basic_generation();
    if (drive_generation == 0U || drive_generation == last_odom_basic_generation_) {
      return;
    }
    last_odom_basic_generation_ = drive_generation;

    double dt = 0.0;
    if (odom_time_valid_) {
      dt = std::clamp((stamp - last_odom_time_).seconds(), 0.0, 0.10);
    }
    last_odom_time_ = stamp;
    odom_time_valid_ = true;

    if (dt > 0.0) {
      const double yaw_mid = odom_yaw_ + 0.5 * yaw_rate_rps * dt;
      odom_x_ += linear_mps * std::cos(yaw_mid) * dt;
      odom_y_ += linear_mps * std::sin(yaw_mid) * dt;
      odom_yaw_ = std::remainder(odom_yaw_ + yaw_rate_rps * dt, kTwoPi);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.orientation.z = std::sin(0.5 * odom_yaw_);
    odom.pose.pose.orientation.w = std::cos(0.5 * odom_yaw_);
    odom.twist.twist.linear.x = linear_mps;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = yaw_rate_rps;

    /* Covariance wheel odometry sengaja konservatif; EKF lokal hanya mengambil
     * vx/vy dari konfigurasi navigation, sedangkan yaw-rate berasal dari IMU. */
    odom.pose.covariance[0] = 0.10;
    odom.pose.covariance[7] = 0.10;
    odom.pose.covariance[35] = 0.20;
    odom.twist.covariance[0] = 0.04;
    odom.twist.covariance[7] = 0.02;
    odom.twist.covariance[35] = 0.10;
    odom_pub_->publish(odom);

    std_msgs::msg::Float64 speed;
    speed.data = linear_mps;
    speed_pub_->publish(speed);

    std_msgs::msg::Float64 drive_actual;
    drive_actual.data = linear_mps;
    drive_actual_pub_->publish(drive_actual);
    std_msgs::msg::Float64 steering_actual;
    steering_actual.data = steering_rad;
    steering_actual_pub_->publish(steering_actual);
    if (steer_feedback_ready()) {
      std_msgs::msg::Float64 steering_raw;
      if (vehicle_mode_ == "ackermann_1_board") {
        steering_raw.data = static_cast<double>(boards_[0]->basic().position_right);
      } else if (vehicle_mode_ == "ackermann_2_board") {
        steering_raw.data = static_cast<double>(boards_[1]->basic().position_left);
      } else {
        steering_raw.data = 0.0;
      }
      steering_raw_ticks_pub_->publish(steering_raw);
    }
    std_msgs::msg::Float64 yaw_actual;
    yaw_actual.data = yaw_rate_rps;
    yaw_rate_actual_pub_->publish(yaw_actual);
  }

  void publish_state(bool command_fresh, bool hardware_motion_permitted) {
    const bool link0 = boards_[0]->connected();
    const bool link1 = vehicle_mode_ == "ackermann_2_board" ? boards_[1]->connected() : link0;
    const bool drive_ready = drive_feedback_ready();
    const bool steer_ready = steer_feedback_ready();
    const bool board0_armed = link0 && (boards_[0]->status() & esc::STATUS_ARMED) != 0U;
    const bool board1_armed = vehicle_mode_ == "ackermann_2_board"
      ? (link1 && (boards_[1]->status() & esc::STATUS_ARMED) != 0U) : board0_armed;
    const bool all_armed = board0_armed && board1_armed;
    const bool board0_fault_free = drive_ready && boards_[0]->error_left() == 0U && boards_[0]->error_right() == 0U;
    const bool steering_calibration_ok = vehicle_mode_ == "differential_1_board" ||
      !require_steering_calibration_for_motion_ || steering_calibrated_;
    bool actuator_ready = drive_ready && steer_ready && all_armed && board0_fault_free &&
      steering_calibration_ok;
    if (vehicle_mode_ == "ackermann_2_board") {
      actuator_ready = actuator_ready &&
        boards_[1]->error_left() == 0U && boards_[1]->error_right() == 0U;
    }

    // Target command selalu dihitung dari command Nav2 terbaru, bahkan ketika link
    // serial belum ada. Ini hanya command/target, BUKAN feedback simulasi.
    const bool force_offline_zero = offline_zero_output_ && !drive_ready;
    const Desired target0 = desired_for(0, !hardware_motion_permitted || force_offline_zero);
    const Desired target1 = desired_for(1, !hardware_motion_permitted || force_offline_zero);
    const double target_drive_mps = force_offline_zero ? 0.0 : drive_target_mps(target0);
    const double target_steering_rad = force_offline_zero ? 0.0 : steering_target_rad(target0, target1);

    std_msgs::msg::Float64 drive_target;
    drive_target.data = target_drive_mps;
    drive_target_pub_->publish(drive_target);
    std_msgs::msg::Float64 steering_target;
    steering_target.data = target_steering_rad;
    steering_target_pub_->publish(steering_target);

    std_msgs::msg::Bool feedback_valid;
    feedback_valid.data = drive_ready && steer_ready;
    feedback_valid_pub_->publish(feedback_valid);

    std_msgs::msg::Bool drive_connected;
    drive_connected.data = drive_ready;
    drive_connected_pub_->publish(drive_connected);
    std_msgs::msg::Bool steer_connected;
    steer_connected.data = steer_ready;
    steer_connected_pub_->publish(steer_connected);
    std_msgs::msg::Bool ready;
    ready.data = actuator_ready;
    ready_pub_->publish(ready);
    std_msgs::msg::Bool armed;
    armed.data = all_armed;
    armed_pub_->publish(armed);

    if (drive_ready) {
      const auto telemetry = boards_[0]->basic();
      if (publish_joint_states_) {
        sensor_msgs::msg::JointState joints;
        joints.header.stamp = now();
        if (vehicle_mode_ == "differential_1_board") {
          const double left_pos = wheel_ticks_to_rad(telemetry.position_left, left_sign_);
          const double right_pos = wheel_ticks_to_rad(telemetry.position_right, right_sign_);
          const double left_vel = telemetry.speed_left * kRpmToRadPerSec * left_sign_;
          const double right_vel = telemetry.speed_right * kRpmToRadPerSec * right_sign_;
          joints.name = {
            "front_left_wheel_joint", "rear_left_wheel_joint",
            "front_right_wheel_joint", "rear_right_wheel_joint"};
          joints.position = {left_pos, left_pos, right_pos, right_pos};
          joints.velocity = {left_vel, left_vel, right_vel, right_vel};
        } else if (vehicle_mode_ == "ackermann_1_board") {
          const double wheel_pos = wheel_ticks_to_rad(telemetry.position_left, drive_sign_);
          const double wheel_vel = telemetry.speed_left * kRpmToRadPerSec * drive_sign_;
          const double steer_pos = steering_ticks_to_rad(telemetry.position_right);
          joints.name = {
            "front_left_wheel_joint", "front_right_wheel_joint",
            "rear_left_wheel_joint", "rear_right_wheel_joint", "steering_joint"};
          joints.position = {wheel_pos, wheel_pos, wheel_pos, wheel_pos, steer_pos};
          joints.velocity = {wheel_vel, wheel_vel, wheel_vel, wheel_vel, 0.0};
        } else {
          const double left_pos = wheel_ticks_to_rad(telemetry.position_left, drive_sign_);
          const double right_pos = wheel_ticks_to_rad(telemetry.position_right, drive_sign_);
          const double left_vel = telemetry.speed_left * kRpmToRadPerSec * drive_sign_;
          const double right_vel = telemetry.speed_right * kRpmToRadPerSec * drive_sign_;
          joints.name = {
            "front_left_wheel_joint", "rear_left_wheel_joint",
            "front_right_wheel_joint", "rear_right_wheel_joint"};
          joints.position = {left_pos, left_pos, right_pos, right_pos};
          joints.velocity = {left_vel, left_vel, right_vel, right_vel};
          if (steer_ready) {
            joints.name.push_back("steering_joint");
            joints.position.push_back(steering_ticks_to_rad(boards_[1]->basic().position_left));
            joints.velocity.push_back(0.0);
          }
        }
        joint_pub_->publish(joints);
      }

      sensor_msgs::msg::BatteryState battery;
      battery.header.stamp = now();
      battery.voltage = static_cast<float>(telemetry.battery_centi_volt) / 100.0F;
      battery_pub_->publish(battery);

      sensor_msgs::msg::Temperature temperature;
      temperature.header.stamp = battery.header.stamp;
      temperature.temperature = static_cast<double>(telemetry.temperature_deci_c) / 10.0;
      temperature_pub_->publish(temperature);
    } else if (offline_zero_output_) {
      if (publish_joint_states_) {
        sensor_msgs::msg::JointState joints;
        joints.header.stamp = now();
        if (vehicle_mode_ == "differential_1_board") {
          joints.name = {
            "front_left_wheel_joint", "rear_left_wheel_joint",
            "front_right_wheel_joint", "rear_right_wheel_joint"};
          joints.position.assign(4, 0.0);
          joints.velocity.assign(4, 0.0);
        } else {
          joints.name = {
            "front_left_wheel_joint", "front_right_wheel_joint",
            "rear_left_wheel_joint", "rear_right_wheel_joint", "steering_joint"};
          joints.position.assign(5, 0.0);
          joints.velocity.assign(5, 0.0);
        }
        joint_pub_->publish(joints);
      }

      sensor_msgs::msg::BatteryState battery;
      battery.header.stamp = now();
      battery.present = false;
      battery.voltage = 0.0F;
      battery_pub_->publish(battery);

      sensor_msgs::msg::Temperature temperature;
      temperature.header.stamp = battery.header.stamp;
      temperature.temperature = 0.0;
      temperature_pub_->publish(temperature);
    }

    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << std::boolalpha
           << "protocol=" << static_cast<unsigned>(esc::kVersion)
           << ";mode=" << vehicle_mode_
           << ";link0=" << link0
           << ";feedback0=" << drive_ready
           << ";offline_zero=" << (offline_zero_output_ && !drive_ready)
           << ";drive=" << (drive_ready ? "READY" : "OFFLINE")
           << ";steer=" << (steer_ready ? "READY" : "OFFLINE")
           << ";ready=" << actuator_ready
           << ";arm=" << (!link0 ? "DISARMED/OFFLINE" : (all_armed ? "ARMED" : "DISARMED"))
           << ";armed0=" << board0_armed
           << ";cmd_fresh=" << command_fresh
           << ";hw_motion_permitted=" << hardware_motion_permitted
           << ";cmd_v=" << last_twist_.linear.x
           << ";cmd_w=" << last_twist_.angular.z
           << ";v_target=" << target_drive_mps
           << ";steer_target=" << target_steering_rad
           << ";steer_cal=" << (steering_calibrated_ ? "VALID" : "NEEDS_CALIBRATION")
           << ";steer_cal_gate=" << (steering_calibration_ok ? "OPEN" : "BLOCKED")
           << ";mode0_l=" << static_cast<unsigned>(target0.mode_left)
           << ";mode0_r=" << static_cast<unsigned>(target0.mode_right)
           << ";sp0_l=" << target0.setpoint_left
           << ";sp0_r=" << target0.setpoint_right;

    if (drive_ready) {
      const auto &basic = boards_[0]->basic();
      double actual_v = 0.0;
      double actual_w = 0.0;
      double actual_steer = 0.0;
      const bool actual_valid = vehicle_feedback(actual_v, actual_w, actual_steer);
      if (actual_valid) {
        stream << ";v_actual=" << actual_v
               << ";w_actual=" << actual_w
               << ";steer_actual=" << actual_steer;
      } else {
        stream << ";v_actual=N/A;w_actual=N/A;steer_actual=N/A";
      }
      stream << ";err0=" << static_cast<int>(boards_[0]->error_left()) << ","
             << static_cast<int>(boards_[0]->error_right())
             << ";batt_v=" << (static_cast<double>(basic.battery_centi_volt) / 100.0)
             << ";temp_c=" << (static_cast<double>(basic.temperature_deci_c) / 10.0)
             << ";link_ms=" << basic.link_age_ms
             << ";telem_hz=" << basic.telemetry_rate_hz;
    } else {
      stream << ";err0=N/A;batt_v=N/A;temp_c=N/A;link_ms=N/A;telem_hz=0";
    }

    if (vehicle_mode_ == "ackermann_2_board") {
      stream << ";link1=" << link1
             << ";armed1=" << board1_armed
             << ";mode1_l=" << static_cast<unsigned>(target1.mode_left)
             << ";mode1_r=" << static_cast<unsigned>(target1.mode_right)
             << ";sp1_l=" << target1.setpoint_left
             << ";sp1_r=" << target1.setpoint_right;
      if (steer_ready) {
        stream << ";err1=" << static_cast<int>(boards_[1]->error_left()) << ","
               << static_cast<int>(boards_[1]->error_right());
      } else {
        stream << ";err1=N/A";
      }
    }
    status.data = stream.str();
    status_pub_->publish(status);

    if (drive_ready != last_drive_ready_ || steer_ready != last_steer_ready_) {
      RCLCPP_INFO(get_logger(), "ESC state: drive=%s steer=%s",
                  drive_ready ? "READY" : "OFFLINE",
                  steer_ready ? "READY" : "OFFLINE");
      last_drive_ready_ = drive_ready;
      last_steer_ready_ = steer_ready;
    }
  }

  void tick() {
    const bool command_fresh =
        received_cmd_ && ((now() - last_cmd_).seconds() < cmd_timeout_s_);
    const bool steering_calibration_ok = vehicle_mode_ == "differential_1_board" ||
      !require_steering_calibration_for_motion_ || steering_calibrated_;
    const bool hardware_motion_permitted = command_fresh && steering_calibration_ok;
    service_board(0, hardware_motion_permitted);
    if (vehicle_mode_ == "ackermann_2_board") service_board(1, hardware_motion_permitted);
    publish_odom_and_speed();
    publish_state(command_fresh, hardware_motion_permitted);
  }

  std::string vehicle_mode_;
  std::string board0_port_;
  std::string board1_port_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  double wheelbase_{0.70};
  double track_width_{0.55};
  double wheel_radius_{0.145};
  double wheel_ticks_per_revolution_{90.0};
  double speed_setpoint_rpm_per_mps_{0.0};
  double max_motor_setpoint_rpm_{1000.0};
  double steering_ticks_per_rad_{1000.0};
  double steering_zero_ticks_{0.0};
  double max_steering_rad_{0.34};
  double min_speed_for_steering_mps_{0.08};
  double cmd_timeout_s_{0.25};
  bool publish_joint_states_{false};
  bool offline_zero_output_{true};
  bool steering_calibrated_{false};
  bool require_steering_calibration_for_motion_{true};
  double drive_sign_{1.0};
  double left_sign_{1.0};
  double right_sign_{1.0};
  double steering_sign_{1.0};

  std::array<std::unique_ptr<esc::SerialBoard>, 2> boards_;
  std::array<int, 2> arm_stage_{{0, 0}};
  std::array<std::uint64_t, 2> seen_generation_{{0, 0}};
  geometry_msgs::msg::Twist last_twist_{};
  rclcpp::Time last_cmd_{};
  bool received_cmd_{false};

  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  rclcpp::Time last_odom_time_{};
  bool odom_time_valid_{false};
  std::uint64_t last_odom_basic_generation_{0};
  bool last_drive_ready_{false};
  bool last_steer_ready_{false};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drive_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drive_actual_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_actual_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_raw_ticks_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_actual_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr feedback_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drive_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr steer_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EscDriverNode>());
  rclcpp::shutdown();
  return 0;
}
