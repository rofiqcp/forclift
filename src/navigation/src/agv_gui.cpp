#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <sstream>
#include <iomanip>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <QApplication>
#include <QAbstractItemView>
#include <QFile>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QImage>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMainWindow>
#include <QPixmap>
#include <QProcess>
#include <QPushButton>
#include <QStringList>
#include <QScrollArea>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "yolo_obstacle_detection_ros2/msg/alignment_state.hpp"
#include "yolo_obstacle_detection_ros2/msg/obstacle_array.hpp"

namespace navigation
{

struct TopicState
{
  double last{0.0};
  double hz{0.0};
  uint64_t count{0};
  std::string type_frame;
};

class DashboardRosNode final : public rclcpp::Node
{
public:
  DashboardRosNode()
  : Node("autonomous_vehicle_interface"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_nav", sensor_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { handleScan("/scan_nav", *msg); });
    safety_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_safety", sensor_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { handleScan("/scan_safety", *msg); });
    lidar_health_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/lidar/safety_healthy", reliable_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        lidar_safety_healthy_ = msg->data;
        lidar_safety_seen_ = true;
        setValue("/lidar/safety_healthy", msg->data ? "true" : "false");
        touch("/lidar/safety_healthy", "std_msgs/Bool");
      });
    lidar_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/lidar/status", latched_qos,
      [this](std_msgs::msg::String::SharedPtr msg) {
        lidar_status_ = msg->data;
        setValue("/lidar/status", msg->data);
        touch("/lidar/status", "std_msgs/String");
      });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { handleImu("/imu/data", *msg); });
    imu_gyro_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/gyro", sensor_qos,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { handleImu("/imu/gyro", *msg); });
    imu_accel_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/accel", sensor_qos,
      [this](sensor_msgs::msg::Imu::SharedPtr msg) { handleImu("/imu/accel", *msg); });
    imu_mag_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/imu/mag", sensor_qos,
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) { handleVector("/imu/mag", *msg); });
    imu_euler_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/imu/euler", sensor_qos,
      [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) { handleVector("/imu/euler", *msg); });
    imu_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/imu/status", latched_qos,
      [this](std_msgs::msg::String::SharedPtr msg) {
        imu_status_ = msg->data;
        setValue("/imu/status", msg->data);
        touch("/imu/status", "std_msgs/String");
      });

    lidar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lidar/odom", reliable_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { handleOdom("/lidar/odom", *msg); });
    ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", reliable_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { handleOdom("/odometry/filtered", *msg); });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", latched_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        have_map_ = true;
        handleGrid("/map", *msg);
      });
    global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", latched_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { handleGrid("/global_costmap/costmap", *msg); });
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", latched_qos,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { handleGrid("/local_costmap/costmap", *msg); });
    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", reliable_qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { handlePose("/amcl_pose", *msg); });
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", reliable_qos,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { handlePose("/initialpose", *msg); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", reliable_qos,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { handlePoseStamped("/goal_pose", *msg); });
    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/smac_plan", latched_qos,
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        touch("/smac_plan", msg->header.frame_id);
        path_points_ = msg->poses.size();
        double len = 0.0;
        for (size_t i = 1; i < msg->poses.size(); ++i) {
          const auto & a = msg->poses[i - 1].pose.position;
          const auto & b = msg->poses[i].pose.position;
          len += std::hypot(b.x - a.x, b.y - a.y);
        }
        path_length_ = len;
        setValue("/smac_plan", std::to_string(path_points_) + " poses | length=" + f(path_length_, 2) + "m");
      });
    planner_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/navigation/planner_status", reliable_qos,
      [this](std_msgs::msg::String::SharedPtr msg) {
        planner_status_ = msg->data;
        setValue("/navigation/planner_status", msg->data);
        touch("/navigation/planner_status", "std_msgs/String");
      });

    autonomy_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/system/autonomy_motion_allowed", reliable_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        autonomy_allowed_ = msg->data; autonomy_seen_ = true;
        setValue("/system/autonomy_motion_allowed", msg->data ? "true" : "false");
        touch("/system/autonomy_motion_allowed", "std_msgs/Bool");
      });
    manual_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/system/manual_motion_allowed", reliable_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        manual_allowed_ = msg->data; manual_seen_ = true;
        setValue("/system/manual_motion_allowed", msg->data ? "true" : "false");
        touch("/system/manual_motion_allowed", "std_msgs/Bool");
      });
    sensor_guard_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/sensor_guard/healthy", reliable_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        sensor_guard_healthy_ = msg->data; sensor_guard_seen_ = true;
        setValue("/sensor_guard/healthy", msg->data ? "true" : "false");
        touch("/sensor_guard/healthy", "std_msgs/Bool");
      });

    esc_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/ready", latched_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        esc_ready_ = msg->data; esc_ready_seen_ = true;
        setValue("/esc/ready", msg->data ? "true" : "false");
        touch("/esc/ready", "std_msgs/Bool");
      });
    esc_armed_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/armed", latched_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) { setValue("/esc/armed", msg->data ? "true" : "false"); touch("/esc/armed", "std_msgs/Bool"); });
    esc_feedback_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/feedback_valid", latched_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) { setValue("/esc/feedback_valid", msg->data ? "true" : "false"); touch("/esc/feedback_valid", "std_msgs/Bool"); });
    esc_drive_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/drive/connected", latched_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) { setValue("/esc/drive/connected", msg->data ? "true" : "false"); touch("/esc/drive/connected", "std_msgs/Bool"); });
    esc_steer_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/steer/connected", latched_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) { setValue("/esc/steer/connected", msg->data ? "true" : "false"); touch("/esc/steer/connected", "std_msgs/Bool"); });
    esc_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/esc/status", latched_qos,
      [this](std_msgs::msg::String::SharedPtr msg) { setValue("/esc/status", msg->data); touch("/esc/status", "std_msgs/String"); });
    esc_mux_source_sub_ = create_subscription<std_msgs::msg::String>(
      "/esc/mux/active_source", latched_qos,
      [this](std_msgs::msg::String::SharedPtr msg) { setValue("/esc/mux/active_source", msg->data); touch("/esc/mux/active_source", "std_msgs/String"); });
    esc_mux_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/esc/mux/status", latched_qos,
      [this](std_msgs::msg::String::SharedPtr msg) { setValue("/esc/mux/status", msg->data); touch("/esc/mux/status", "std_msgs/String"); });

    auto float_topic = [this, &best_effort_qos](const std::string & topic, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr & sub) {
      sub = create_subscription<std_msgs::msg::Float64>(topic, best_effort_qos,
        [this, topic](std_msgs::msg::Float64::SharedPtr msg) { setValue(topic, f(msg->data)); touch(topic, "std_msgs/Float64"); });
    };
    float_topic("/esc/speed", esc_speed_sub_);
    float_topic("/esc/drive_target_mps", esc_drive_target_sub_);
    float_topic("/esc/drive_actual_mps", esc_drive_actual_sub_);
    float_topic("/esc/steering_target_rad", esc_steer_target_sub_);
    float_topic("/esc/steering_actual_rad", esc_steer_actual_sub_);
    float_topic("/esc/yaw_rate_actual_rps", esc_yaw_rate_sub_);

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/esc/battery", reliable_qos,
      [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
        setValue("/esc/battery", "V=" + f(msg->voltage, 2) + " I=" + f(msg->current, 2) + " A | " + f(msg->percentage * 100.0, 1) + "%");
        touch("/esc/battery", "sensor_msgs/BatteryState");
      });
    temperature_sub_ = create_subscription<sensor_msgs::msg::Temperature>(
      "/esc/temperature", reliable_qos,
      [this](sensor_msgs::msg::Temperature::SharedPtr msg) {
        setValue("/esc/temperature", f(msg->temperature, 1) + " degC");
        touch("/esc/temperature", "sensor_msgs/Temperature");
      });

    cmd_raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav_raw", reliable_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { handleTwist("/cmd_vel_nav_raw", *msg); });
    cmd_smoothed_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav_smoothed", reliable_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { handleTwist("/cmd_vel_nav_smoothed", *msg); });
    cmd_collision_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_collision_safe", reliable_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { handleTwist("/cmd_vel_collision_safe", *msg); });
    cmd_safe_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", reliable_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { handleTwist("/cmd_vel", *msg); });
    cmd_actuator_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel/actuator", reliable_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { handleTwist("/cmd_vel/actuator", *msg); });
    mux_selected_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/esc/mux/selected", reliable_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { handleTwist("/esc/mux/selected", *msg); });

    camera_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", sensor_qos,
      [this](sensor_msgs::msg::Image::SharedPtr msg) { storeImage("/camera/color/image_raw", *msg); });
    camera_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/camera/color/status", latched_qos,
      [this](std_msgs::msg::String::SharedPtr msg) { camera_status_ = msg->data; setValue("/camera/color/status", msg->data); touch("/camera/color/status", "std_msgs/String"); });
    yolo_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/obstacle_detection/visualization", sensor_qos,
      [this](sensor_msgs::msg::Image::SharedPtr msg) { storeImage("/obstacle_detection/visualization", *msg); });
    yolo_obstacles_sub_ = create_subscription<yolo_obstacle_detection_ros2::msg::ObstacleArray>(
      "/obstacle_detection/obstacles", reliable_qos,
      [this](yolo_obstacle_detection_ros2::msg::ObstacleArray::SharedPtr msg) {
        yolo_obstacle_count_ = msg->obstacles.size();
        yolo_pallet_count_ = static_cast<size_t>(std::max<int32_t>(0, msg->pallet_count));
        yolo_warning_ = msg->warning_active;
        std::ostringstream out;
        out << "objects=" << msg->obstacles.size() << " pallet=" << msg->pallet_count
            << " dynamic=" << msg->dynamic_count << " warning=" << (msg->warning_active ? "true" : "false");
        setValue("/obstacle_detection/obstacles", out.str());
        touch("/obstacle_detection/obstacles", msg->header.frame_id);
      });
    yolo_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/obstacle_detection/status", latched_qos,
      [this](std_msgs::msg::String::SharedPtr msg) { yolo_status_ = msg->data; setValue("/obstacle_detection/status", msg->data); touch("/obstacle_detection/status", "std_msgs/String"); });
    yolo_perf_sub_ = create_subscription<std_msgs::msg::String>(
      "/obstacle_detection/performance", rclcpp::QoS(10).best_effort(),
      [this](std_msgs::msg::String::SharedPtr msg) { yolo_performance_ = msg->data; setValue("/obstacle_detection/performance", msg->data); touch("/obstacle_detection/performance", "std_msgs/String"); });

    fork_state_sub_ = create_subscription<yolo_obstacle_detection_ros2::msg::AlignmentState>(
      "/fork_alignment/state", latched_qos,
      [this](yolo_obstacle_detection_ros2::msg::AlignmentState::SharedPtr msg) {
        fork_state_seen_ = true;
        fork_data_valid_ = msg->data_valid;
        fork_ready_ = msg->ready_for_insertion;
        fork_state_text_ = msg->state_text;
        fork_lateral_m_ = msg->error_lateral_m;
        fork_yaw_deg_ = msg->error_yaw_deg;
        fork_steer_deg_ = msg->estimated_steering_deg;
        std::ostringstream out;
        out << (msg->state_text.empty() ? "UNKNOWN" : msg->state_text)
            << " | valid=" << (msg->data_valid ? "true" : "false")
            << " pallet=" << (msg->pallet_detected ? "true" : "false")
            << " stable=" << (msg->detection_stable ? "true" : "false")
            << " lateral=" << f(msg->error_lateral_m, 3) << "m"
            << " yaw=" << f(msg->error_yaw_deg, 2) << "deg"
            << " steer=" << f(msg->estimated_steering_deg, 2) << "deg"
            << " ready=" << (msg->ready_for_insertion ? "true" : "false");
        setValue("/fork_alignment/state", out.str());
        touch("/fork_alignment/state", msg->header.frame_id);
      });
    fork_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/fork_alignment/image", sensor_qos,
      [this](sensor_msgs::msg::Image::SharedPtr msg) { storeImage("/fork_alignment/image", *msg); });

    winch_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/winch/connected", latched_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        winch_connected_seen_ = true; winch_connected_ = msg->data;
        setValue("/winch/connected", msg->data ? "true" : "false");
        touch("/winch/connected", "std_msgs/Bool");
      });
    winch_port_sub_ = create_subscription<std_msgs::msg::String>(
      "/winch/port", latched_qos,
      [this](std_msgs::msg::String::SharedPtr msg) { winch_port_ = msg->data; setValue("/winch/port", msg->data); touch("/winch/port", "std_msgs/String"); });
    winch_state_sub_ = create_subscription<std_msgs::msg::String>(
      "/winch/state", latched_qos,
      [this](std_msgs::msg::String::SharedPtr msg) { winch_state_ = msg->data; setValue("/winch/state", msg->data); touch("/winch/state", "std_msgs/String"); });
    winch_top_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/winch/top_limit", latched_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) { setValue("/winch/top_limit", msg->data ? "true" : "false"); touch("/winch/top_limit", "std_msgs/Bool"); });
    winch_bottom_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/winch/bottom_limit", latched_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) { setValue("/winch/bottom_limit", msg->data ? "true" : "false"); touch("/winch/bottom_limit", "std_msgs/Bool"); });
    winch_pwm_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/winch/pwm_pct", latched_qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) { setValue("/winch/pwm_pct", f(msg->data, 1) + "%"); touch("/winch/pwm_pct", "std_msgs/Float64"); });
    winch_direction_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/winch/direction", latched_qos,
      [this](std_msgs::msg::Int32::SharedPtr msg) { setValue("/winch/direction", std::to_string(msg->data)); touch("/winch/direction", "std_msgs/Int32"); });
    winch_servo_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/winch/servo_deg", latched_qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) { setValue("/winch/servo_deg", f(msg->data, 1) + "deg"); touch("/winch/servo_deg", "std_msgs/Float64"); });
    winch_raw_sub_ = create_subscription<std_msgs::msg::String>(
      "/winch/raw", reliable_qos,
      [this](std_msgs::msg::String::SharedPtr msg) { setValue("/winch/raw", msg->data); touch("/winch/raw", "std_msgs/String"); });
  }

  static double steadyNow()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static double yawDeg(double qx, double qy, double qz, double qw)
  {
    const double siny = 2.0 * (qw * qz + qx * qy);
    const double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny, cosy) * 180.0 / 3.14159265358979323846;
  }

  static std::string f(double value, int precision = 3)
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
  }

  void setValue(const std::string & topic, const std::string & value)
  {
    values_[topic] = value;
  }

  std::string value(const std::string & topic) const
  {
    const auto it = values_.find(topic);
    return it == values_.end() ? std::string{} : it->second;
  }

  QImage image(const std::string & topic) const
  {
    const auto it = images_.find(topic);
    return it == images_.end() ? QImage{} : it->second;
  }

  uint64_t imageGeneration(const std::string & topic) const
  {
    const auto it = image_generations_.find(topic);
    return it == image_generations_.end() ? 0U : it->second;
  }

  void storeImage(const std::string & topic, const sensor_msgs::msg::Image & msg)
  {
    touch(topic, msg.header.frame_id);
    setValue(topic,
      std::to_string(msg.width) + "x" + std::to_string(msg.height) +
      " " + msg.encoding + " step=" + std::to_string(msg.step));

    // GUI preview is deliberately decimated and downsized. ROS topic health still
    // counts every frame; the copy below is at most 5 Hz per image stream.
    const double now = steadyNow();
    auto & last_preview = image_preview_last_[topic];
    if (last_preview > 0.0 && now - last_preview < 0.20) return;
    last_preview = now;

    if (msg.width == 0 || msg.height == 0 || msg.data.empty()) return;
    QImage raw;
    if ((msg.encoding == "bgr8" || msg.encoding == "rgb8") &&
        msg.step >= msg.width * 3U) {
      raw = QImage(
        msg.data.data(), static_cast<int>(msg.width), static_cast<int>(msg.height),
        static_cast<int>(msg.step), QImage::Format_RGB888);
      raw = msg.encoding == "bgr8" ? raw.rgbSwapped() : raw.copy();
    } else if (msg.encoding == "mono8" && msg.step >= msg.width) {
      raw = QImage(
        msg.data.data(), static_cast<int>(msg.width), static_cast<int>(msg.height),
        static_cast<int>(msg.step), QImage::Format_Grayscale8).copy();
    } else {
      return;
    }
    if (raw.isNull()) return;
    images_[topic] = raw.scaled(720, 405, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    ++image_generations_[topic];
  }

  void handleScan(const std::string & topic, const sensor_msgs::msg::LaserScan & msg)
  {
    size_t valid = 0;
    double min_r = std::numeric_limits<double>::infinity();
    double max_r = 0.0;
    for (const float r : msg.ranges) {
      if (std::isfinite(r) && r >= msg.range_min && r <= msg.range_max) {
        ++valid;
        min_r = std::min(min_r, static_cast<double>(r));
        max_r = std::max(max_r, static_cast<double>(r));
      }
    }
    const double ratio = msg.ranges.empty() ? 0.0 : 100.0 * static_cast<double>(valid) / static_cast<double>(msg.ranges.size());
    std::ostringstream out;
    out << "beams=" << msg.ranges.size() << " valid=" << valid
        << " (" << std::fixed << std::setprecision(1) << ratio << "%)";
    if (valid > 0) out << " range=" << std::setprecision(2) << min_r << ".." << max_r << "m";
    setValue(topic, out.str());
    touch(topic, msg.header.frame_id);
  }

  void handleImu(const std::string & topic, const sensor_msgs::msg::Imu & msg)
  {
    std::ostringstream out;
    out << "yaw=" << std::fixed << std::setprecision(2)
        << yawDeg(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        << "deg | gyro=[" << msg.angular_velocity.x << ", " << msg.angular_velocity.y << ", " << msg.angular_velocity.z
        << "] | accel=[" << msg.linear_acceleration.x << ", " << msg.linear_acceleration.y << ", " << msg.linear_acceleration.z << "]";
    setValue(topic, out.str());
    touch(topic, msg.header.frame_id);
  }

  void handleVector(const std::string & topic, const geometry_msgs::msg::Vector3Stamped & msg)
  {
    setValue(topic, "[" + f(msg.vector.x) + ", " + f(msg.vector.y) + ", " + f(msg.vector.z) + "]");
    touch(topic, msg.header.frame_id);
  }

  void handleOdom(const std::string & topic, const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const auto & t = msg.twist.twist;
    std::ostringstream out;
    out << "x=" << f(p.x) << " y=" << f(p.y) << " yaw=" << f(yawDeg(q.x, q.y, q.z, q.w), 2)
        << "deg | vx=" << f(t.linear.x) << " wz=" << f(t.angular.z);
    setValue(topic, out.str());
    touch(topic, msg.header.frame_id);
  }

  void handleGrid(const std::string & topic, const nav_msgs::msg::OccupancyGrid & msg)
  {
    const double width_m = static_cast<double>(msg.info.width) * msg.info.resolution;
    const double height_m = static_cast<double>(msg.info.height) * msg.info.resolution;
    size_t occupied = 0, free = 0, unknown = 0;
    for (const auto cell : msg.data) {
      if (cell < 0) ++unknown;
      else if (cell >= 65) ++occupied;
      else ++free;
    }
    std::ostringstream out;
    out << msg.info.width << "x" << msg.info.height << " @ " << f(msg.info.resolution, 3) << "m"
        << " (" << f(width_m, 1) << "x" << f(height_m, 1) << "m)"
        << " | occ=" << occupied << " free=" << free << " unk=" << unknown;
    setValue(topic, out.str());
    touch(topic, msg.header.frame_id);
  }

  void handlePose(const std::string & topic, const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const double cov_xy = std::sqrt(std::max(0.0, msg.pose.covariance[0] + msg.pose.covariance[7]));
    std::ostringstream out;
    out << "x=" << f(p.x) << " y=" << f(p.y) << " yaw=" << f(yawDeg(q.x, q.y, q.z, q.w), 2)
        << "deg | cov_xy=" << f(cov_xy, 4);
    setValue(topic, out.str());
    touch(topic, msg.header.frame_id);
  }

  void handlePoseStamped(const std::string & topic, const geometry_msgs::msg::PoseStamped & msg)
  {
    const auto & p = msg.pose.position;
    const auto & q = msg.pose.orientation;
    setValue(topic, "x=" + f(p.x) + " y=" + f(p.y) + " yaw=" + f(yawDeg(q.x, q.y, q.z, q.w), 2) + "deg");
    touch(topic, msg.header.frame_id);
  }

  void handleTwist(const std::string & topic, const geometry_msgs::msg::Twist & msg)
  {
    setValue(topic, "vx=" + f(msg.linear.x) + " vy=" + f(msg.linear.y) + " wz=" + f(msg.angular.z));
    touch(topic, "geometry_msgs/Twist");
  }

  void touch(const std::string & topic, const std::string & type_frame = {})
  {
    const double t = steadyNow();
    auto & state = topics_[topic];
    if (state.last > 0.0) {
      const double dt = t - state.last;
      if (dt > 1e-4) {
        const double instant_hz = 1.0 / dt;
        state.hz = state.hz <= 0.0 ? instant_hz : (0.85 * state.hz + 0.15 * instant_hz);
      }
    }
    state.last = t;
    ++state.count;
    if (!type_frame.empty()) state.type_frame = type_frame;
  }

  TopicState topic(const std::string & name) const
  {
    const auto it = topics_.find(name);
    return it == topics_.end() ? TopicState{} : it->second;
  }

  bool fresh(const std::string & name, double max_age = 1.5) const
  {
    const auto state = topic(name);
    return state.last > 0.0 && steadyNow() - state.last <= max_age;
  }

  std::set<std::string> nodeNames() const
  {
    std::set<std::string> result;
    for (const auto & name : get_node_names()) result.insert(name);
    return result;
  }

  bool transformAvailable(const std::string & parent, const std::string & child) const
  {
    try {
      (void)tf_buffer_.lookupTransform(parent, child, tf2::TimePointZero);
      return true;
    } catch (...) {
      return false;
    }
  }

  bool transformValues(
    const std::string & parent, const std::string & child,
    double & x, double & y, double & z, double & yaw_deg) const
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(parent, child, tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      z = tf.transform.translation.z;
      const auto & q = tf.transform.rotation;
      const double siny = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      yaw_deg = std::atan2(siny, cosy) * 180.0 / 3.14159265358979323846;
      return true;
    } catch (...) {
      x = y = z = yaw_deg = 0.0;
      return false;
    }
  }

  bool have_map_{false};
  bool autonomy_allowed_{false};
  bool manual_allowed_{false};
  bool sensor_guard_healthy_{false};
  bool esc_ready_{false};
  bool lidar_safety_healthy_{false};
  bool autonomy_seen_{false};
  bool manual_seen_{false};
  bool sensor_guard_seen_{false};
  bool esc_ready_seen_{false};
  bool lidar_safety_seen_{false};

  bool fork_state_seen_{false};
  bool fork_data_valid_{false};
  bool fork_ready_{false};
  double fork_lateral_m_{0.0};
  double fork_yaw_deg_{0.0};
  double fork_steer_deg_{0.0};
  std::string fork_state_text_;

  bool winch_connected_seen_{false};
  bool winch_connected_{false};
  std::string winch_port_;
  std::string winch_state_;

  size_t yolo_obstacle_count_{0};
  size_t yolo_pallet_count_{0};
  bool yolo_warning_{false};
  std::string camera_status_;
  std::string yolo_status_;
  std::string yolo_performance_;

  size_t path_points_{0};
  double path_length_{0.0};
  std::string planner_status_;
  std::string lidar_status_;
  std::string imu_status_;

private:
  std::unordered_map<std::string, TopicState> topics_;
  std::unordered_map<std::string, std::string> values_;
  std::unordered_map<std::string, QImage> images_;
  std::unordered_map<std::string, double> image_preview_last_;
  std::unordered_map<std::string, uint64_t> image_generations_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr safety_scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lidar_health_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lidar_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_gyro_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_accel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_mag_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_euler_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr imu_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensor_guard_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr esc_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr esc_armed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr esc_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr esc_drive_connected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr esc_steer_connected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr esc_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr esc_mux_source_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr esc_mux_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr esc_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr esc_drive_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr esc_drive_actual_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr esc_steer_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr esc_steer_actual_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr esc_yaw_rate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temperature_sub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_raw_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_smoothed_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_collision_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_safe_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_actuator_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mux_selected_sub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_image_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr yolo_image_sub_;
  rclcpp::Subscription<yolo_obstacle_detection_ros2::msg::ObstacleArray>::SharedPtr yolo_obstacles_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_perf_sub_;
  rclcpp::Subscription<yolo_obstacle_detection_ros2::msg::AlignmentState>::SharedPtr fork_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr fork_image_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr winch_connected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr winch_port_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr winch_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr winch_top_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr winch_bottom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr winch_pwm_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr winch_direction_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr winch_servo_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr winch_raw_sub_;

  mutable tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

struct StatusCard
{
  QLabel * state{nullptr};
  QLabel * detail{nullptr};
};

struct PageDef
{
  QString group;
  QString code;
  QString name;
};

struct TelemetryRowDef
{
  QString label;
  std::string topic;
  bool latched{false};
};

struct TelemetryPage
{
  QString name;
  QLabel * summary{nullptr};
  QTableWidget * table{nullptr};
  std::vector<TelemetryRowDef> rows;
  QLabel * image{nullptr};
  std::string image_topic;
  uint64_t image_generation{0};
  int map_slot{0};
  QLabel * map_info{nullptr};
};

class DashboardWindow final : public QMainWindow
{
public:
  explicit DashboardWindow(std::shared_ptr<DashboardRosNode> node)
  : node_(std::move(node)), start_time_(DashboardRosNode::steadyNow())
  {
    setWindowTitle("Autonomous Vehicle Interface — Sekolah Vokasi UNDIP");
    resize(1600, 900);
    setMinimumSize(1180, 720);
    applyStyle();
    buildUi();

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() {
      rclcpp::spin_some(node_);
      refresh();
    });
    timer_->start(100);
  }

private:
  static constexpr int HEALTHY = 0;
  static constexpr int DEGRADED = 1;
  static constexpr int ERROR_STATE = 2;

  void applyStyle()
  {
    qApp->setStyleSheet(R"QSS(
      QMainWindow, QWidget { background:#20252b; color:#e8edf2; font-size:12px; }
      QFrame#LeftPanel { background:#1a1f24; border-right:1px solid #39414a; }
      QFrame#BrandFrame { background:#242b32; border:1px solid #39414a; border-radius:10px; }
      QLabel#LogoBox { background:#f5f5f5; border:1px solid #48535e; border-radius:10px; padding:3px; }
      QLabel#BrandTitle { font-size:17px; font-weight:700; }
      QLabel#BrandSub { color:#bfc8d1; }
      QLabel#WorkspaceTitle { font-size:19px; font-weight:700; padding:4px 0; }
      QLabel#NavGroup { color:#80909f; font-size:10px; font-weight:700; padding:8px 6px 2px 6px; }
      QFrame#SectionFrame, QGroupBox { background:#252c33; border:1px solid #39414a; border-radius:8px; margin-top:8px; }
      QGroupBox::title { subcontrol-origin: margin; left:9px; padding:0 4px; font-weight:700; }
      QPushButton { background:#303842; color:#e8edf2; border:1px solid #48535e; border-radius:6px; padding:7px 9px; }
      QPushButton:hover { background:#39444f; }
      QPushButton:checked { background:#5f1c34; border-color:#8d3453; }
      QPushButton#MenuToggle { font-weight:700; }
      QTableWidget { background:#191e23; alternate-background-color:#20262c; border:1px solid #414b55; gridline-color:#38414a; }
      QHeaderView::section { background:#2a323a; color:#dfe6ec; padding:6px; border:0; border-right:1px solid #414b55; }
      QScrollArea { border:0; }
      QTabWidget::pane { border:1px solid #3a444d; }
      QTabBar::tab { background:#2b333b; padding:7px 12px; }
      QTabBar::tab:selected { background:#5f1c34; }
      QSplitter::handle { background:#39414a; }
      QStatusBar { background:#15191d; color:#cbd5df; }
    )QSS");
  }

  QFrame * sectionFrame(const QString & title, QVBoxLayout *& out_layout)
  {
    auto * frame = new QFrame();
    frame->setObjectName("SectionFrame");
    out_layout = new QVBoxLayout(frame);
    out_layout->setContentsMargins(10, 10, 10, 10);
    out_layout->setSpacing(6);
    auto * label = new QLabel(title);
    label->setStyleSheet("font-weight:700;");
    out_layout->addWidget(label);
    return frame;
  }

  StatusCard addCard(QGridLayout * grid, int row, int col, const QString & title)
  {
    auto * box = new QGroupBox(title);
    auto * layout = new QVBoxLayout(box);
    layout->setContentsMargins(8, 10, 8, 8);
    layout->setSpacing(5);
    auto * state = new QLabel("WAITING");
    state->setAlignment(Qt::AlignCenter);
    state->setMinimumHeight(22);
    auto * detail = new QLabel("starting...");
    detail->setAlignment(Qt::AlignCenter);
    detail->setWordWrap(true);
    detail->setMinimumHeight(25);
    detail->setStyleSheet("font-size:10px;color:#b9c4ce;");
    layout->addWidget(state);
    layout->addWidget(detail);
    grid->addWidget(box, row, col);
    setCardStyle(state, DEGRADED);
    return {state, detail};
  }

  static void setCardStyle(QLabel * label, int severity)
  {
    QString background = "#bd8d19";
    if (severity == HEALTHY) background = "#1aa55b";
    if (severity == ERROR_STATE) background = "#d64b50";
    label->setStyleSheet(QString(
      "font-weight:700;background:%1;color:white;border-radius:6px;padding:3px;").arg(background));
  }

  void setCard(StatusCard & card, const QString & state, const QString & detail, int severity)
  {
    card.state->setText(state);
    card.detail->setText(detail);
    setCardStyle(card.state, severity);
  }

  void buildUi()
  {
    auto * central = new QWidget(this);
    setCentralWidget(central);
    auto * outer = new QVBoxLayout(central);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(0);

    auto * main_splitter = new QSplitter(Qt::Horizontal);
    outer->addWidget(main_splitter, 1);

    // LEFT panel: exact organization of the previous engineering GUI.
    auto * left_panel = new QFrame();
    left_panel->setObjectName("LeftPanel");
    auto * left_layout = new QVBoxLayout(left_panel);
    left_layout->setContentsMargins(10, 10, 10, 10);
    left_layout->setSpacing(8);

    auto * brand = new QFrame();
    brand->setObjectName("BrandFrame");
    auto * brand_layout = new QHBoxLayout(brand);
    brand_layout->setContentsMargins(10, 8, 10, 8);
    auto * logo = new QLabel();
    logo->setObjectName("LogoBox");
    logo->setAlignment(Qt::AlignCenter);
    logo->setFixedSize(72, 72);
    const QStringList logo_candidates = {
      "/home/otomasi2/forclift/install/navigation/share/navigation/assets/logo_undip.png",
      "/home/otomasi2/forclift/src/navigation/assets/logo_undip.png"};
    bool logo_loaded = false;
    for (const auto & candidate : logo_candidates) {
      QPixmap pix(candidate);
      if (!pix.isNull()) {
        logo->setPixmap(pix.scaled(66, 66, Qt::KeepAspectRatio, Qt::SmoothTransformation));
        logo_loaded = true;
        break;
      }
    }
    if (!logo_loaded) logo->setText("UNDIP");

    auto * brand_text = new QVBoxLayout();
    auto * title = new QLabel("Autonomous Vehicle Interface");
    title->setObjectName("BrandTitle");
    auto * sub1 = new QLabel("Sekolah Vokasi");
    auto * sub2 = new QLabel("Universitas Diponegoro");
    sub1->setObjectName("BrandSub");
    sub2->setObjectName("BrandSub");
    brand_text->addWidget(title);
    brand_text->addWidget(sub1);
    brand_text->addWidget(sub2);
    brand_layout->addWidget(logo);
    brand_layout->addLayout(brand_text, 1);
    left_layout->addWidget(brand);

    auto * left_body = new QSplitter(Qt::Horizontal);
    left_layout->addWidget(left_body, 1);

    auto * rail_container = new QWidget();
    auto * rail_layout = new QVBoxLayout(rail_container);
    rail_layout->setContentsMargins(0, 0, 0, 0);
    rail_layout->setSpacing(4);
    menu_toggle_ = new QPushButton("☰   Minimize");
    menu_toggle_->setObjectName("MenuToggle");
    connect(menu_toggle_, &QPushButton::clicked, this, [this]() { toggleNav(); });
    rail_layout->addWidget(menu_toggle_);

    auto * nav_scroll = new QScrollArea();
    nav_scroll->setWidgetResizable(true);
    nav_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    auto * nav_body = new QWidget();
    nav_layout_ = new QVBoxLayout(nav_body);
    nav_layout_->setContentsMargins(0, 0, 0, 0);
    nav_layout_->setSpacing(3);
    nav_scroll->setWidget(nav_body);
    rail_layout->addWidget(nav_scroll, 1);
    left_body->addWidget(rail_container);

    settings_stack_ = new QStackedWidget();
    left_body->addWidget(settings_stack_);
    left_body->setSizes({145, 390});
    left_body->setStretchFactor(0, 0);
    left_body->setStretchFactor(1, 1);
    left_body_splitter_ = left_body;

    workspace_stack_ = new QStackedWidget();
    main_splitter->addWidget(left_panel);
    main_splitter->addWidget(workspace_stack_);
    main_splitter->setSizes({540, 1060});
    main_splitter->setStretchFactor(0, 0);
    main_splitter->setStretchFactor(1, 1);

    const std::vector<PageDef> defs = {
      {"SYSTEM", "CN", "Connection"}, {"SYSTEM", "CF", "Configuration"},
      {"MAP", "M1", "Map 1"}, {"MAP", "M2", "Map 2"}, {"MAP", "M3", "Map 3"},
      {"MAP", "MC", "Map Comparison"}, {"MAP", "GT", "Ground Truth"},
      {"LOCALIZATION", "LD", "LiDAR"}, {"LOCALIZATION", "LO", "LiDAR Odometry"},
      {"LOCALIZATION", "SL", "SLAM Toolbox"}, {"LOCALIZATION", "IM", "IMU"},
      {"LOCALIZATION", "EK", "EKF"}, {"LOCALIZATION", "AM", "AMCL"},
      {"LOCALIZATION", "TF", "TF & Timing"}, {"LOCALIZATION", "ES", "ESC / Motion"},
      {"ACTUATOR", "WC", "Winch"},
      {"NAVIGATION", "GC", "Global Costmap"}, {"NAVIGATION", "SM", "Smac Hybrid-A*"},
      {"NAVIGATION", "MP", "MPPI"}, {"NAVIGATION", "CS", "Command & Safety"},
      {"NAVIGATION", "GO", "Goal / End-to-End"},
      {"PERCEPTION", "CA", "Camera"}, {"PERCEPTION", "YO", "YOLO Detection"},
      {"PERCEPTION", "HA", "Hole-Block Alignment"},
      {"DATA", "EX", "Experiments"}, {"DATA", "RP", "Reports / Export"}, {"DATA", "ST", "Settings"}
    };

    QString previous_group;
    for (const auto & def : defs) {
      if (def.group != previous_group) {
        auto * group = new QLabel(def.group);
        group->setObjectName("NavGroup");
        nav_layout_->addWidget(group);
        previous_group = def.group;
      }
      auto * button = new QPushButton(def.code + "   " + def.name);
      button->setCheckable(true);
      button->setMinimumHeight(34);
      button->setProperty("code", def.code);
      button->setProperty("fullname", def.name);
      nav_buttons_.push_back(button);
      nav_layout_->addWidget(button);

      QWidget * settings = nullptr;
      QWidget * workspace = nullptr;
      if (def.name == "Connection") {
        settings = buildConnectionSettings();
        workspace = buildConnectionWorkspace();
      } else {
        settings = buildSubsystemSettings(def.name);
        workspace = buildSubsystemWorkspace(def.name);
      }
      const int index = settings_stack_->addWidget(settings);
      workspace_stack_->addWidget(workspace);
      connect(button, &QPushButton::clicked, this, [this, index, button]() {
        selectPage(index, button);
      });
    }
    nav_layout_->addStretch(1);
    if (!nav_buttons_.empty()) selectPage(0, nav_buttons_.front());

    setStatusBar(new QStatusBar(this));
    status_label_ = new QLabel("ROS: STARTING | MAP DISPLAY: - | MAP NAV: - | TF: monitor | REC: OFF");
    statusBar()->addPermanentWidget(status_label_, 1);
  }

  QWidget * buildConnectionSettings()
  {
    auto * scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    auto * body = new QWidget();
    auto * layout = new QVBoxLayout(body);
    layout->setContentsMargins(0, 0, 4, 0);

    QVBoxLayout * conn_layout = nullptr;
    auto * conn = sectionFrame("Connection", conn_layout);
    ros_settings_pill_ = new QLabel("ROS STARTING");
    ros_settings_pill_->setAlignment(Qt::AlignCenter);
    setCardStyle(ros_settings_pill_, DEGRADED);
    conn_layout->addWidget(ros_settings_pill_);
    auto * refresh = new QPushButton("Refresh Mapping Results");
    connect(refresh, &QPushButton::clicked, this, [this]() { refreshMaps(); });
    conn_layout->addWidget(refresh);
    layout->addWidget(conn);

    QVBoxLayout * maps_layout = nullptr;
    auto * maps = sectionFrame("Map Slots", maps_layout);
    for (int i = 0; i < 3; ++i) {
      map_labels_[static_cast<size_t>(i)] = new QLabel(QString("Map %1: checking...").arg(i + 1));
      map_labels_[static_cast<size_t>(i)]->setWordWrap(true);
      maps_layout->addWidget(map_labels_[static_cast<size_t>(i)]);
    }
    active_map_label_ = new QLabel("Active navigation map: -");
    active_map_label_->setWordWrap(true);
    maps_layout->addWidget(active_map_label_);
    layout->addWidget(maps);

    QVBoxLayout * tools_layout = nullptr;
    auto * tools = sectionFrame("Runtime", tools_layout);
    auto * rviz_btn = new QPushButton("Open Autonomous RViz");
    connect(rviz_btn, &QPushButton::clicked, this, [this]() { openRviz(); });
    auto * map_btn = new QPushButton("Open Mapping Control C++");
    connect(map_btn, &QPushButton::clicked, this, [this]() { openMapping(); });
    tools_layout->addWidget(rviz_btn);
    tools_layout->addWidget(map_btn);
    layout->addWidget(tools);
    layout->addStretch(1);
    scroll->setWidget(body);
    return scroll;
  }

  std::vector<TelemetryRowDef> rowsForPage(const QString & name) const
  {
    using R = TelemetryRowDef;
    if (name == "Configuration") return {
      R{"Autonomy permission", "/system/autonomy_motion_allowed", true},
      R{"Manual permission", "/system/manual_motion_allowed", true},
      R{"Sensor guard", "/sensor_guard/healthy", true},
      R{"ESC ready", "/esc/ready", true},
      R{"LiDAR driver status", "/lidar/status", true},
      R{"IMU driver status", "/imu/status", true},
      R{"Camera status", "/camera/color/status", true},
      R{"YOLO status", "/obstacle_detection/status", true}};
    if (name == "Map 1" || name == "Map 2" || name == "Map 3") return {
      R{"Navigation map", "/map", true}, R{"AMCL pose", "/amcl_pose", false},
      R{"EKF odometry", "/odometry/filtered", false}, R{"Navigation scan", "/scan_nav", false},
      R{"Current goal", "/goal_pose", false}, R{"Smac path", "/smac_plan", true}};
    if (name == "Map Comparison") return {
      R{"Active map", "/map", true}, R{"AMCL pose", "/amcl_pose", false},
      R{"Global costmap", "/global_costmap/costmap", true}, R{"Local costmap", "/local_costmap/costmap", true}};
    if (name == "Ground Truth") return {
      R{"AMCL estimate", "/amcl_pose", false}, R{"EKF estimate", "/odometry/filtered", false},
      R{"Initial/reference pose", "/initialpose", false}, R{"Goal reference", "/goal_pose", false}};
    if (name == "LiDAR") return {
      R{"Navigation scan", "/scan_nav", false}, R{"Safety scan", "/scan_safety", false},
      R{"Safety health", "/lidar/safety_healthy", true}, R{"Driver status", "/lidar/status", true}};
    if (name == "LiDAR Odometry") return {
      R{"LiDAR odometry", "/lidar/odom", false}, R{"Navigation scan", "/scan_nav", false},
      R{"EKF fused odometry", "/odometry/filtered", false}};
    if (name == "SLAM Toolbox") return {
      R{"SLAM / map output", "/map", true}, R{"LiDAR scan", "/scan_nav", false},
      R{"LiDAR odometry", "/lidar/odom", false}, R{"IMU", "/imu/data", false}};
    if (name == "IMU") return {
      R{"Fused IMU", "/imu/data", false}, R{"Gyroscope", "/imu/gyro", false},
      R{"Accelerometer", "/imu/accel", false}, R{"Magnetometer", "/imu/mag", false},
      R{"Euler", "/imu/euler", false}, R{"Driver status", "/imu/status", true}};
    if (name == "EKF") return {
      R{"Filtered odometry", "/odometry/filtered", false}, R{"LiDAR odometry input", "/lidar/odom", false},
      R{"IMU input", "/imu/data", false}};
    if (name == "AMCL") return {
      R{"AMCL pose", "/amcl_pose", false}, R{"Static map", "/map", true},
      R{"Scan", "/scan_nav", false}, R{"Initial pose", "/initialpose", false}};
    if (name == "TF & Timing") return {
      R{"EKF odometry timing", "/odometry/filtered", false}, R{"AMCL timing", "/amcl_pose", false},
      R{"LiDAR timing", "/scan_nav", false}, R{"IMU timing", "/imu/data", false}};
    if (name == "ESC / Motion") return {
      R{"ESC ready", "/esc/ready", true}, R{"ESC armed", "/esc/armed", true},
      R{"Feedback valid", "/esc/feedback_valid", true}, R{"Drive connected", "/esc/drive/connected", true},
      R{"Steer connected", "/esc/steer/connected", true}, R{"Drive target", "/esc/drive_target_mps", false},
      R{"Drive actual", "/esc/drive_actual_mps", false}, R{"Steer target", "/esc/steering_target_rad", false},
      R{"Steer actual", "/esc/steering_actual_rad", false}, R{"Yaw rate actual", "/esc/yaw_rate_actual_rps", false},
      R{"Battery", "/esc/battery", false}, R{"Temperature", "/esc/temperature", false},
      R{"ESC status", "/esc/status", true}};
    if (name == "Winch") return {
      R{"Connection", "/winch/connected", true}, R{"Serial port", "/winch/port", true},
      R{"State", "/winch/state", true}, R{"Top limit", "/winch/top_limit", true},
      R{"Bottom limit", "/winch/bottom_limit", true}, R{"PWM", "/winch/pwm_pct", true},
      R{"Direction", "/winch/direction", true}, R{"Servo", "/winch/servo_deg", true},
      R{"Raw telemetry", "/winch/raw", false}};
    if (name == "Global Costmap") return {
      R{"Global costmap", "/global_costmap/costmap", true}, R{"Static map", "/map", true},
      R{"AMCL pose", "/amcl_pose", false}, R{"LiDAR scan", "/scan_nav", false}};
    if (name == "Smac Hybrid-A*") return {
      R{"Planner status", "/navigation/planner_status", false}, R{"Smac plan", "/smac_plan", true},
      R{"Goal", "/goal_pose", false}, R{"Global costmap", "/global_costmap/costmap", true}};
    if (name == "MPPI") return {
      R{"Controller raw cmd", "/cmd_vel_nav_raw", false}, R{"Smoothed cmd", "/cmd_vel_nav_smoothed", false},
      R{"Collision-safe cmd", "/cmd_vel_collision_safe", false}, R{"Final nav cmd", "/cmd_vel", false},
      R{"Actuator cmd", "/cmd_vel/actuator", false}, R{"Local costmap", "/local_costmap/costmap", true}};
    if (name == "Command & Safety") return {
      R{"Controller command", "/cmd_vel_nav_raw", false}, R{"Velocity smoother", "/cmd_vel_nav_smoothed", false},
      R{"Collision monitor", "/cmd_vel_collision_safe", false}, R{"Sensor guard output", "/cmd_vel", false},
      R{"MUX selected", "/esc/mux/selected", false}, R{"Actuator output", "/cmd_vel/actuator", false},
      R{"MUX source", "/esc/mux/active_source", true}, R{"MUX status", "/esc/mux/status", true},
      R{"Autonomy interlock", "/system/autonomy_motion_allowed", true}, R{"Sensor guard", "/sensor_guard/healthy", true}};
    if (name == "Goal / End-to-End") return {
      R{"Goal", "/goal_pose", false}, R{"Planner status", "/navigation/planner_status", false},
      R{"Smac path", "/smac_plan", true}, R{"AMCL pose", "/amcl_pose", false},
      R{"EKF odometry", "/odometry/filtered", false}, R{"Final command", "/cmd_vel", false},
      R{"ESC feedback", "/esc/drive_actual_mps", false}};
    if (name == "Camera") return {
      R{"Raw image", "/camera/color/image_raw", false}, R{"Camera status", "/camera/color/status", true}};
    if (name == "YOLO Detection") return {
      R{"Detected objects", "/obstacle_detection/obstacles", false},
      R{"Visualization", "/obstacle_detection/visualization", false},
      R{"Detector status", "/obstacle_detection/status", true},
      R{"GPU performance", "/obstacle_detection/performance", false}};
    if (name == "Hole-Block Alignment") return {
      R{"Alignment state", "/fork_alignment/state", false}, R{"Alignment image", "/fork_alignment/image", false},
      R{"YOLO objects", "/obstacle_detection/obstacles", false}, R{"Camera image", "/camera/color/image_raw", false}};
    if (name == "Experiments") return {
      R{"LiDAR evidence", "/scan_nav", false}, R{"IMU evidence", "/imu/data", false},
      R{"Localization evidence", "/amcl_pose", false}, R{"Planner evidence", "/smac_plan", true},
      R{"Command evidence", "/cmd_vel", false}, R{"YOLO performance", "/obstacle_detection/performance", false}};
    if (name == "Reports / Export") return {
      R{"Map", "/map", true}, R{"AMCL", "/amcl_pose", false}, R{"Plan", "/smac_plan", true},
      R{"ESC", "/esc/status", true}, R{"YOLO", "/obstacle_detection/performance", false},
      R{"Fork", "/fork_alignment/state", false}};
    if (name == "Settings") return {
      R{"LiDAR status", "/lidar/status", true}, R{"IMU status", "/imu/status", true},
      R{"Camera status", "/camera/color/status", true}, R{"YOLO status", "/obstacle_detection/status", true},
      R{"ESC status", "/esc/status", true}, R{"Winch state", "/winch/state", true}};
    return {};
  }

  QWidget * buildSubsystemSettings(const QString & name)
  {
    auto * scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    auto * body = new QWidget();
    auto * layout = new QVBoxLayout(body);
    QVBoxLayout * section_layout = nullptr;
    auto * section = sectionFrame(name, section_layout);
    auto * text = new QLabel(
      "Live ROS 2 telemetry is connected directly to this page. The table on the right shows the actual topic state, rate, age, value, and frame instead of a placeholder page.");
    text->setWordWrap(true);
    section_layout->addWidget(text);
    auto * count = new QLabel(QString("Monitored signals: %1").arg(static_cast<int>(rowsForPage(name).size())));
    count->setStyleSheet("color:#b9c4ce;");
    section_layout->addWidget(count);
    layout->addWidget(section);
    layout->addStretch(1);
    scroll->setWidget(body);
    return scroll;
  }

  QWidget * buildSubsystemWorkspace(const QString & name)
  {
    auto * widget = new QWidget();
    auto * layout = new QVBoxLayout(widget);
    layout->setContentsMargins(10, 8, 10, 10);
    layout->setSpacing(8);

    auto * title = new QLabel(name);
    title->setObjectName("WorkspaceTitle");
    layout->addWidget(title);

    TelemetryPage page;
    page.name = name;
    page.rows = rowsForPage(name);
    page.summary = new QLabel("Waiting for ROS telemetry...");
    page.summary->setWordWrap(true);
    page.summary->setStyleSheet("background:#252c33;border:1px solid #39414a;border-radius:7px;padding:8px;color:#d7e0e8;");
    layout->addWidget(page.summary);

    if (name == "Camera") page.image_topic = "/camera/color/image_raw";
    else if (name == "YOLO Detection") page.image_topic = "/obstacle_detection/visualization";
    else if (name == "Hole-Block Alignment") page.image_topic = "/fork_alignment/image";

    if (!page.image_topic.empty()) {
      page.image = new QLabel("Waiting for image...");
      page.image->setAlignment(Qt::AlignCenter);
      page.image->setMinimumHeight(260);
      page.image->setMaximumHeight(430);
      page.image->setStyleSheet("background:#12171c;border:1px solid #414b55;border-radius:7px;color:#8fa0af;");
      layout->addWidget(page.image, 1);
    }

    if (name == "Map 1") page.map_slot = 1;
    else if (name == "Map 2") page.map_slot = 2;
    else if (name == "Map 3") page.map_slot = 3;
    if (page.map_slot > 0) {
      page.map_info = new QLabel("Map slot: checking file...");
      page.map_info->setWordWrap(true);
      page.map_info->setStyleSheet("background:#191e23;border:1px solid #414b55;border-radius:7px;padding:8px;color:#c9d4de;");
      layout->addWidget(page.map_info);
    }

    page.table = new QTableWidget(static_cast<int>(page.rows.size()), 6);
    page.table->setHorizontalHeaderLabels({"Signal", "Topic", "State", "Hz", "Age [s]", "Value / Frame"});
    page.table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    page.table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    page.table->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
    page.table->setAlternatingRowColors(true);
    page.table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    for (int row = 0; row < static_cast<int>(page.rows.size()); ++row) {
      page.table->setItem(row, 0, new QTableWidgetItem(page.rows[static_cast<size_t>(row)].label));
      page.table->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(page.rows[static_cast<size_t>(row)].topic)));
      for (int col = 2; col < 6; ++col) page.table->setItem(row, col, new QTableWidgetItem("-"));
    }
    layout->addWidget(page.table, 2);
    telemetry_pages_.push_back(page);
    return widget;
  }

  QWidget * buildConnectionWorkspace()
  {
    auto * widget = new QWidget();
    auto * root = new QVBoxLayout(widget);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(7);

    auto * title = new QLabel("Connection & System Overview");
    title->setObjectName("WorkspaceTitle");
    root->addWidget(title);

    auto * cards_widget = new QWidget();
    auto * cards = new QGridLayout(cards_widget);
    cards->setContentsMargins(0, 0, 0, 0);
    cards->setHorizontalSpacing(7);
    cards->setVerticalSpacing(7);

    ros_ = addCard(cards, 0, 0, "ROS 2");
    usb_ = addCard(cards, 0, 1, "USB Resolver");
    lidar_ = addCard(cards, 0, 2, "LiDAR");
    lidar_odom_ = addCard(cards, 0, 3, "LiDAR Odometry");
    imu_ = addCard(cards, 0, 4, "IMU");
    ekf_ = addCard(cards, 1, 0, "EKF");
    map_ = addCard(cards, 1, 1, "Map Server");
    amcl_ = addCard(cards, 1, 2, "AMCL");
    global_costmap_ = addCard(cards, 1, 3, "Global Costmap");
    smac_ = addCard(cards, 1, 4, "Smac Planner");
    local_costmap_ = addCard(cards, 2, 0, "Local Costmap");
    mppi_ = addCard(cards, 2, 1, "MPPI Controller");
    smoother_ = addCard(cards, 2, 2, "Velocity Smoother");
    collision_ = addCard(cards, 2, 3, "Collision Monitor");
    sensor_guard_ = addCard(cards, 2, 4, "Sensor Guard");
    autonomy_ = addCard(cards, 3, 0, "Autonomy Interlock");
    manual_ = addCard(cards, 3, 1, "Manual Motion Interlock");
    esc_ = addCard(cards, 3, 2, "ESC");
    camera_ = addCard(cards, 3, 3, "Camera");
    yolo_ = addCard(cards, 3, 4, "YOLO");
    fork_ = addCard(cards, 4, 0, "Fork Alignment");
    winch_ = addCard(cards, 4, 1, "Winch");
    tf_ = addCard(cards, 4, 2, "TF");
    root->addWidget(cards_widget);

    auto * tabs = new QTabWidget();
    topic_table_ = new QTableWidget();
    topic_names_ = {
      "/scan_nav", "/scan_safety", "/lidar/safety_healthy", "/lidar/status", "/lidar/odom",
      "/imu/gyro", "/imu/accel", "/imu/mag", "/imu/euler", "/imu/status", "/imu/data",
      "/odometry/filtered", "/map", "/amcl_pose", "/global_costmap/costmap", "/local_costmap/costmap",
      "/smac_plan", "/navigation/planner_status", "/goal_pose",
      "/sensor_guard/healthy", "/system/autonomy_motion_allowed", "/system/manual_motion_allowed",
      "/cmd_vel_nav_raw", "/cmd_vel_nav_smoothed", "/cmd_vel_collision_safe", "/cmd_vel", "/cmd_vel/actuator",
      "/esc/ready", "/esc/status", "/esc/drive_actual_mps", "/esc/steering_actual_rad",
      "/camera/color/image_raw", "/camera/color/status",
      "/obstacle_detection/obstacles", "/obstacle_detection/visualization", "/obstacle_detection/status", "/obstacle_detection/performance",
      "/fork_alignment/state", "/fork_alignment/image",
      "/winch/connected", "/winch/state", "/winch/pwm_pct", "/winch/servo_deg"};
    topic_table_->setRowCount(static_cast<int>(topic_names_.size()));
    topic_table_->setColumnCount(6);
    topic_table_->setHorizontalHeaderLabels({"Topic", "State", "Hz", "Age [s]", "Count", "Type/Frame"});
    topic_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    topic_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);
    topic_table_->setAlternatingRowColors(true);
    for (int row = 0; row < static_cast<int>(topic_names_.size()); ++row) {
      topic_table_->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(topic_names_[static_cast<size_t>(row)])));
      for (int column = 1; column < 6; ++column) topic_table_->setItem(row, column, new QTableWidgetItem("-"));
    }
    tabs->addTab(topic_table_, "Topics");

    tf_table_ = new QTableWidget(4, 6);
    tf_table_->setHorizontalHeaderLabels({"Transform", "State", "X", "Y", "Z", "Yaw [deg]"});
    tf_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    const QStringList tf_names = {"map→odom", "odom→base_footprint", "base_footprint→lidar_link", "base_footprint→imu_link"};
    for (int row = 0; row < tf_names.size(); ++row) {
      tf_table_->setItem(row, 0, new QTableWidgetItem(tf_names[row]));
      for (int column = 1; column < 6; ++column) tf_table_->setItem(row, column, new QTableWidgetItem("-"));
    }
    tabs->addTab(tf_table_, "TF");
    root->addWidget(tabs, 1);
    return widget;
  }

  void selectPage(int index, QPushButton * selected)
  {
    settings_stack_->setCurrentIndex(index);
    workspace_stack_->setCurrentIndex(index);
    for (auto * button : nav_buttons_) button->setChecked(button == selected);
  }

  void toggleNav()
  {
    nav_expanded_ = !nav_expanded_;
    menu_toggle_->setText(nav_expanded_ ? "☰   Minimize" : "☰");
    for (auto * button : nav_buttons_) {
      const QString code = button->property("code").toString();
      const QString full = button->property("fullname").toString();
      button->setText(nav_expanded_ ? code + "   " + full : code);
      button->setToolTip(full);
    }
    if (nav_expanded_) {
      settings_stack_->show();
      left_body_splitter_->setSizes({145, 390});
    } else {
      settings_stack_->hide();
      left_body_splitter_->setSizes({86, 0});
    }
  }

  bool hasNodeLike(const std::set<std::string> & nodes, const std::vector<std::string> & patterns) const
  {
    for (const auto & node_name : nodes) {
      for (const auto & pattern : patterns) {
        if (node_name == pattern || node_name == "/" + pattern || node_name.find(pattern) != std::string::npos) return true;
      }
    }
    return false;
  }

  QString topicDetail(const std::string & name) const
  {
    const auto state = node_->topic(name);
    if (state.count == 0) return "no data";
    const double age = DashboardRosNode::steadyNow() - state.last;
    return QString("%1 Hz | %2s").arg(state.hz, 0, 'f', 1).arg(age, 0, 'f', 2);
  }

  void refresh()
  {
    const auto nodes = node_->nodeNames();
    const double uptime = DashboardRosNode::steadyNow() - start_time_;
    setCard(ros_, "ONLINE", QString("graph nodes: %1").arg(static_cast<qulonglong>(nodes.size())), HEALTHY);
    ros_settings_pill_->setText("ROS ONLINE");
    setCardStyle(ros_settings_pill_, HEALTHY);

    const bool imu_alias = QFileInfo::exists("/tmp/agv_devices/imu");
    const bool lidar_alias = QFileInfo::exists("/tmp/agv_devices/lidar");
    if (imu_alias && lidar_alias) {
      setCard(usb_, "HEALTHY", "IMU/LiDAR aliases ready", HEALTHY);
    } else if (uptime < 8.0) {
      setCard(usb_, "STARTING", QString("IMU %1 | LiDAR %2").arg(imu_alias ? "OK" : "WAIT").arg(lidar_alias ? "OK" : "WAIT"), DEGRADED);
    } else if (imu_alias || lidar_alias) {
      setCard(usb_, "PARTIAL", QString("IMU %1 | LiDAR %2").arg(imu_alias ? "OK" : "MISSING").arg(lidar_alias ? "OK" : "MISSING"), DEGRADED);
    } else {
      setCard(usb_, "ERROR", "IMU/LiDAR aliases missing", ERROR_STATE);
    }

    const bool lidar_node = hasNodeLike(nodes, {"lidar_node"});
    const bool lidar_fresh = node_->fresh("/scan_nav", 1.5);
    if (lidar_fresh) setCard(lidar_, "RUNNING", topicDetail("/scan_nav"), HEALTHY);
    else if (lidar_node) setCard(lidar_, "STARTING", node_->lidar_status_.empty() ? "waiting scan data" : QString::fromStdString(node_->lidar_status_).left(72), DEGRADED);
    else setCard(lidar_, uptime < 8.0 ? "WAITING" : "NOT RUNNING", "node/topic not found", uptime < 8.0 ? DEGRADED : ERROR_STATE);

    const bool lodom_node = hasNodeLike(nodes, {"hector_slam_node", "lidar_odometry"});
    const bool lodom_fresh = node_->fresh("/lidar/odom", 1.5);
    if (lodom_fresh) {
      setCard(lidar_odom_, "RUNNING", topicDetail("/lidar/odom"), HEALTHY);
    } else if (lodom_node) {
      setCard(lidar_odom_, "WARMING", "node active; waiting fresh /lidar/odom", DEGRADED);
    } else {
      setCard(lidar_odom_, uptime < 8.0 ? "WAITING" : "NOT RUNNING", "LiDAR odometry node/topic not found", uptime < 8.0 ? DEGRADED : ERROR_STATE);
    }

    const bool imu_node = hasNodeLike(nodes, {"imu_node"});
    const bool imu_fresh = node_->fresh("/imu/data", 1.5);
    if (imu_fresh) setCard(imu_, "RUNNING", topicDetail("/imu/data"), HEALTHY);
    else if (imu_node) setCard(imu_, "STARTING", node_->imu_status_.empty() ? "waiting IMU frames" : QString::fromStdString(node_->imu_status_).left(72), DEGRADED);
    else setCard(imu_, uptime < 8.0 ? "WAITING" : "NOT RUNNING", "node/topic not found", uptime < 8.0 ? DEGRADED : ERROR_STATE);

    const bool ekf_node = hasNodeLike(nodes, {"ekf_filter_node"});
    const bool ekf_fresh = node_->fresh("/odometry/filtered", 1.5);
    if (ekf_fresh) setCard(ekf_, "HEALTHY", topicDetail("/odometry/filtered"), HEALTHY);
    else if (ekf_node) setCard(ekf_, "WARMING", "filter active; waiting fresh odometry inputs", DEGRADED);
    else setCard(ekf_, uptime < 8.0 ? "WAITING" : "NOT RUNNING", "EKF node not found", uptime < 8.0 ? DEGRADED : ERROR_STATE);

    const bool map_node = hasNodeLike(nodes, {"map_server"});
    if (node_->have_map_) setCard(map_, "MAP READY", node_->value("/map").empty() ? "latched occupancy map available" : QString::fromStdString(node_->value("/map")).left(72), HEALTHY);
    else if (map_node) setCard(map_, "LOADING", "map_server active; waiting latched /map", DEGRADED);
    else setCard(map_, uptime < 8.0 ? "WAITING" : "NOT RUNNING", "map_server not found", uptime < 8.0 ? DEGRADED : ERROR_STATE);

    const bool amcl_node = hasNodeLike(nodes, {"amcl"});
    const bool map_odom = node_->transformAvailable("map", "odom");
    if (map_odom && node_->fresh("/amcl_pose", 3.0)) setCard(amcl_, "LOCALIZED", "map→odom TF valid | " + topicDetail("/amcl_pose"), HEALTHY);
    else if (map_odom) setCard(amcl_, "TF READY", "map→odom valid; waiting fresh /amcl_pose", DEGRADED);
    else setCard(amcl_, amcl_node ? "LOCALIZING" : "WAITING", "waiting for AMCL convergence / map→odom", DEGRADED);

    const bool planner_node = hasNodeLike(nodes, {"planner_server"});
    const bool controller_node = hasNodeLike(nodes, {"controller_server"});
    const bool smoother_node = hasNodeLike(nodes, {"velocity_smoother"});
    const bool collision_node = hasNodeLike(nodes, {"collision_monitor"});
    const bool gc_fresh = node_->fresh("/global_costmap/costmap", 3.0);
    const bool lc_fresh = node_->fresh("/local_costmap/costmap", 3.0);
    if (gc_fresh) setCard(global_costmap_, "RUNNING", topicDetail("/global_costmap/costmap"), HEALTHY);
    else if (planner_node) setCard(global_costmap_, "WARMING", "planner active; waiting global costmap", DEGRADED);
    else setCard(global_costmap_, "WAITING", "planner/global costmap unavailable", DEGRADED);

    if (planner_node && node_->path_points_ >= 2) {
      setCard(smac_, "PATH READY", QString("%1 pts | %2 m").arg(static_cast<qulonglong>(node_->path_points_)).arg(node_->path_length_, 0, 'f', 2), HEALTHY);
    } else if (planner_node) {
      const QString detail = node_->planner_status_.empty() ? "planner ready; waiting Goal Pose" : QString::fromStdString(node_->planner_status_).left(72);
      setCard(smac_, "IDLE / READY", detail, HEALTHY);
    } else {
      setCard(smac_, "WAITING", "planner_server not found", DEGRADED);
    }

    if (lc_fresh) setCard(local_costmap_, "RUNNING", topicDetail("/local_costmap/costmap"), HEALTHY);
    else if (controller_node) setCard(local_costmap_, "WARMING", "controller active; waiting local costmap", DEGRADED);
    else setCard(local_costmap_, "WAITING", "controller/local costmap unavailable", DEGRADED);

    const bool controller_cmd = node_->fresh("/cmd_vel_nav_raw", 1.5);
    setCard(mppi_, controller_cmd ? "TRACKING" : (controller_node ? "IDLE / READY" : "WAITING"),
      controller_cmd ? topicDetail("/cmd_vel_nav_raw") : (controller_node ? "controller ready; no active path command" : "controller_server not found"),
      controller_node ? HEALTHY : DEGRADED);
    setCard(smoother_, node_->fresh("/cmd_vel_nav_smoothed", 1.5) ? "ACTIVE" : (smoother_node ? "IDLE / READY" : "WAITING"),
      node_->fresh("/cmd_vel_nav_smoothed", 1.5) ? topicDetail("/cmd_vel_nav_smoothed") : "waiting command stream", smoother_node ? HEALTHY : DEGRADED);
    setCard(collision_, node_->fresh("/cmd_vel_collision_safe", 1.5) ? "ACTIVE" : (collision_node ? "IDLE / READY" : "WAITING"),
      node_->fresh("/cmd_vel_collision_safe", 1.5) ? topicDetail("/cmd_vel_collision_safe") : "waiting command stream", collision_node ? HEALTHY : DEGRADED);

    const bool sensor_ok = node_->sensor_guard_seen_ ? node_->sensor_guard_healthy_ : (lidar_fresh && imu_fresh);
    setCard(sensor_guard_, sensor_ok ? "HEALTHY" : (node_->sensor_guard_seen_ ? "HOLD" : "STARTING"),
      sensor_ok ? "sensor inputs fresh" : (node_->sensor_guard_seen_ ? "sensor guard reports HOLD" : "waiting sensor guard state"), sensor_ok ? HEALTHY : DEGRADED);
    setCard(autonomy_, node_->autonomy_seen_ ? (node_->autonomy_allowed_ ? "HEALTHY" : "HOLD") : "STARTING",
      node_->autonomy_seen_ ? (node_->autonomy_allowed_ ? "motion prerequisites passed" : "fail-closed until all checks pass") : "waiting autonomy interlock state",
      node_->autonomy_seen_ && node_->autonomy_allowed_ ? HEALTHY : DEGRADED);
    setCard(manual_, node_->manual_seen_ ? (node_->manual_allowed_ ? "HEALTHY" : "HOLD") : "STARTING",
      node_->manual_seen_ ? (node_->manual_allowed_ ? "manual motion available" : "manual interlock is holding") : "waiting manual interlock state",
      node_->manual_seen_ && node_->manual_allowed_ ? HEALTHY : DEGRADED);

    const bool esc_node = hasNodeLike(nodes, {"esc_driver"});
    if (node_->esc_ready_seen_ && node_->esc_ready_) {
      const auto esc_status = node_->value("/esc/status");
      setCard(esc_, "READY", esc_status.empty() ? "ESC ready telemetry received" : QString::fromStdString(esc_status).left(72), HEALTHY);
    } else if (node_->esc_ready_seen_) {
      const auto esc_status = node_->value("/esc/status");
      setCard(esc_, "NOT READY", esc_status.empty() ? "ESC reports ready=false" : QString::fromStdString(esc_status).left(72), DEGRADED);
    } else if (esc_node) {
      setCard(esc_, "CONNECTING", "esc_driver active; waiting /esc/ready", DEGRADED);
    } else {
      setCard(esc_, uptime < 8.0 ? "WAITING" : "NOT RUNNING", "esc_driver not found", uptime < 8.0 ? DEGRADED : ERROR_STATE);
    }

    const bool camera_node = hasNodeLike(nodes, {"astra_rgb_v4l2_node", "astra_camera", "camera_node"});
    const bool yolo_node = hasNodeLike(nodes, {"obstacle_detector_node", "yolo_obstacle", "yolo_detection"});
    const bool fork_node = hasNodeLike(nodes, {"hole_block_alignment_node", "fork_alignment"});
    const bool winch_node = hasNodeLike(nodes, {"winch_serial_node", "winch"});

    const bool camera_fresh = node_->fresh("/camera/color/image_raw", 2.0);
    if (camera_fresh) {
      const auto meta = node_->value("/camera/color/image_raw");
      setCard(camera_, "STREAMING", QString("%1 | %2").arg(topicDetail("/camera/color/image_raw")).arg(meta.empty() ? "image" : QString::fromStdString(meta)).left(92), HEALTHY);
    } else if (camera_node) {
      setCard(camera_, "STARTING", node_->camera_status_.empty() ? "camera node active; waiting image frames" : QString::fromStdString(node_->camera_status_).left(72), DEGRADED);
    } else {
      setCard(camera_, uptime < 8.0 ? "WAITING" : "NOT RUNNING", "camera node/topic not found", uptime < 8.0 ? DEGRADED : ERROR_STATE);
    }

    const bool yolo_fresh = node_->fresh("/obstacle_detection/obstacles", 2.5) || node_->fresh("/obstacle_detection/visualization", 2.5);
    const QString yolo_status = QString::fromStdString(node_->yolo_status_);
    const bool yolo_passthrough = yolo_status.contains("passthrough", Qt::CaseInsensitive) || yolo_status.contains("not loaded", Qt::CaseInsensitive);
    if (yolo_fresh && !yolo_passthrough) {
      setCard(yolo_, "RUNNING", QString("objects=%1 | pallets=%2%3").arg(static_cast<qulonglong>(node_->yolo_obstacle_count_)).arg(static_cast<qulonglong>(node_->yolo_pallet_count_)).arg(node_->yolo_warning_ ? " | WARNING" : ""), HEALTHY);
    } else if (yolo_fresh && yolo_passthrough) {
      setCard(yolo_, "PASSTHROUGH", yolo_status.left(72), DEGRADED);
    } else if (yolo_node) {
      setCard(yolo_, "STARTING", yolo_status.isEmpty() ? "detector active; waiting detections" : yolo_status.left(72), DEGRADED);
    } else {
      setCard(yolo_, uptime < 8.0 ? "WAITING" : "NOT RUNNING", "YOLO detector node not found", uptime < 8.0 ? DEGRADED : ERROR_STATE);
    }

    const bool fork_fresh = node_->fresh("/fork_alignment/state", 2.0);
    if (fork_fresh && node_->fork_data_valid_) {
      const QString detail = QString("%1 | lateral=%2m yaw=%3deg steer=%4deg")
        .arg(QString::fromStdString(node_->fork_state_text_).left(24))
        .arg(node_->fork_lateral_m_, 0, 'f', 3).arg(node_->fork_yaw_deg_, 0, 'f', 2).arg(node_->fork_steer_deg_, 0, 'f', 2);
      setCard(fork_, node_->fork_ready_ ? "READY INSERT" : "ALIGNING", detail, HEALTHY);
    } else if (fork_fresh) {
      const QString detail = node_->fork_state_text_.empty() ? "state streaming; waiting valid pallet/camera data" : QString::fromStdString(node_->fork_state_text_).left(72);
      setCard(fork_, "WAITING TARGET", detail, DEGRADED);
    } else if (fork_node) {
      setCard(fork_, "STARTING", "alignment node active; waiting /fork_alignment/state", DEGRADED);
    } else {
      setCard(fork_, uptime < 8.0 ? "WAITING" : "NOT RUNNING", "hole_block_alignment_node not found", uptime < 8.0 ? DEGRADED : ERROR_STATE);
    }

    if (node_->winch_connected_seen_) {
      const QString detail = QString("%1 | %2").arg(QString::fromStdString(node_->winch_port_).isEmpty() ? "port unknown" : QString::fromStdString(node_->winch_port_))
        .arg(QString::fromStdString(node_->winch_state_).isEmpty() ? "state unknown" : QString::fromStdString(node_->winch_state_));
      setCard(winch_, node_->winch_connected_ ? "CONNECTED" : "DISCONNECTED", detail.left(86), node_->winch_connected_ ? HEALTHY : DEGRADED);
    } else if (winch_node) {
      setCard(winch_, "PROBING", "winch node active; waiting /winch/connected", DEGRADED);
    } else {
      setCard(winch_, uptime < 8.0 ? "WAITING" : "NOT RUNNING", "winch_serial_node not found", uptime < 8.0 ? DEGRADED : ERROR_STATE);
    }

    const bool odom_base = node_->transformAvailable("odom", "base_footprint");
    setCard(tf_, map_odom && odom_base ? "HEALTHY" : "INITIAL POSE", QString("local TF %1; %2").arg(odom_base ? "OK" : "WAIT").arg(map_odom ? "map→odom OK" : "waiting AMCL map→odom"), map_odom && odom_base ? HEALTHY : DEGRADED);

    refreshTopics();
    refreshTf();
    refreshTelemetryPages();
    if (++refresh_counter_ >= 20) {
      refresh_counter_ = 0;
      refreshMaps();
    }
    const QString map_state = node_->have_map_ ? "READY" : "-";
    const QString tf_state = map_odom && odom_base ? "OK" : "monitor";
    status_label_->setText(QString("ROS: ONLINE | MAP DISPLAY: %1 | MAP NAV: %2 | TF: %3 | REC: OFF").arg(map_state).arg(active_map_short_).arg(tf_state));
  }

  void refreshTelemetryPages()
  {
    const double now = DashboardRosNode::steadyNow();
    for (auto & page : telemetry_pages_) {
      int received = 0;
      int healthy = 0;
      for (int row = 0; row < static_cast<int>(page.rows.size()); ++row) {
        const auto & def = page.rows[static_cast<size_t>(row)];
        const auto state = node_->topic(def.topic);
        const bool has = state.count > 0;
        const double age = has ? std::max(0.0, now - state.last) : 0.0;
        const bool fresh = has && age <= 2.5;
        if (has) ++received;
        if (has && (def.latched || fresh)) ++healthy;

        QString state_text = "NO DATA";
        if (has) {
          if (fresh) state_text = "LIVE";
          else if (def.latched) state_text = "LATCHED";
          else state_text = "STALE";
        }
        page.table->item(row, 2)->setText(state_text);
        page.table->item(row, 3)->setText(has ? QString::number(state.hz, 'f', 1) : "-");
        page.table->item(row, 4)->setText(has ? QString::number(age, 'f', 2) : "-");

        const std::string value = node_->value(def.topic);
        QString value_text;
        if (!value.empty()) value_text = QString::fromStdString(value);
        else if (!state.type_frame.empty()) value_text = QString::fromStdString(state.type_frame);
        else value_text = has ? "data received" : "waiting publisher/data";
        page.table->item(row, 5)->setText(value_text);
      }

      QString summary = QString("ROS telemetry: %1/%2 signals usable, %3/%2 have received data.")
        .arg(healthy).arg(static_cast<int>(page.rows.size())).arg(received);

      if (page.name == "Camera") {
        if (node_->fresh("/camera/color/image_raw", 2.0)) {
          summary = QString("Camera STREAMING — %1. %2")
            .arg(topicDetail("/camera/color/image_raw"))
            .arg(QString::fromStdString(node_->value("/camera/color/image_raw")));
        } else if (!node_->camera_status_.empty()) {
          summary = "Camera status: " + QString::fromStdString(node_->camera_status_);
        } else {
          summary = "Camera belum mengirim frame ke /camera/color/image_raw.";
        }
      } else if (page.name == "YOLO Detection") {
        if (node_->fresh("/obstacle_detection/obstacles", 2.5)) {
          summary = QString("YOLO LIVE — objects=%1, pallet=%2, warning=%3. %4")
            .arg(static_cast<qulonglong>(node_->yolo_obstacle_count_))
            .arg(static_cast<qulonglong>(node_->yolo_pallet_count_))
            .arg(node_->yolo_warning_ ? "ON" : "OFF")
            .arg(QString::fromStdString(node_->yolo_performance_).left(100));
        } else if (!node_->yolo_status_.empty()) {
          summary = "YOLO status: " + QString::fromStdString(node_->yolo_status_);
        } else {
          summary = "YOLO belum mengirim /obstacle_detection/obstacles.";
        }
      } else if (page.name == "Hole-Block Alignment") {
        if (node_->fresh("/fork_alignment/state", 2.0)) {
          if (node_->fork_data_valid_) {
            summary = QString("Fork alignment %1 — lateral=%2 m, yaw=%3°, steering=%4°, ready=%5.")
              .arg(QString::fromStdString(node_->fork_state_text_).isEmpty() ? "ACTIVE" : QString::fromStdString(node_->fork_state_text_))
              .arg(node_->fork_lateral_m_, 0, 'f', 3)
              .arg(node_->fork_yaw_deg_, 0, 'f', 2)
              .arg(node_->fork_steer_deg_, 0, 'f', 2)
              .arg(node_->fork_ready_ ? "YES" : "NO");
          } else {
            summary = QString("Fork node STREAMING, tetapi data_valid=false — %1. Menunggu kamera + deteksi pallet yang fresh.")
              .arg(QString::fromStdString(node_->fork_state_text_).isEmpty() ? "WAITING TARGET" : QString::fromStdString(node_->fork_state_text_));
          }
        } else {
          summary = "Belum ada /fork_alignment/state. Fork tidak dianggap aktif hanya karena nama node ada.";
        }
      } else if (page.name == "Winch") {
        if (node_->winch_connected_seen_) {
          summary = QString("Winch %1 — port=%2, state=%3.")
            .arg(node_->winch_connected_ ? "CONNECTED" : "DISCONNECTED")
            .arg(QString::fromStdString(node_->winch_port_).isEmpty() ? "-" : QString::fromStdString(node_->winch_port_))
            .arg(QString::fromStdString(node_->winch_state_).isEmpty() ? "-" : QString::fromStdString(node_->winch_state_));
        } else {
          summary = "Menunggu telemetry /winch/connected dari winch_serial_node.";
        }
      } else if (page.name == "ESC / Motion") {
        const QString ready = node_->esc_ready_seen_ ? (node_->esc_ready_ ? "READY" : "NOT READY") : "NO STATE";
        summary = QString("ESC %1 — drive actual: %2 | steering actual: %3")
          .arg(ready)
          .arg(QString::fromStdString(node_->value("/esc/drive_actual_mps")).isEmpty() ? "-" : QString::fromStdString(node_->value("/esc/drive_actual_mps")))
          .arg(QString::fromStdString(node_->value("/esc/steering_actual_rad")).isEmpty() ? "-" : QString::fromStdString(node_->value("/esc/steering_actual_rad")));
      } else if (page.name == "TF & Timing") {
        const bool map_odom = node_->transformAvailable("map", "odom");
        const bool odom_base = node_->transformAvailable("odom", "base_footprint");
        const bool lidar_tf = node_->transformAvailable("base_footprint", "lidar_link");
        const bool imu_tf = node_->transformAvailable("base_footprint", "imu_link");
        summary = QString("TF: map→odom=%1 | odom→base=%2 | base→LiDAR=%3 | base→IMU=%4. Topic timing ditampilkan di tabel.")
          .arg(map_odom ? "OK" : "WAIT").arg(odom_base ? "OK" : "WAIT")
          .arg(lidar_tf ? "OK" : "WAIT").arg(imu_tf ? "OK" : "WAIT");
      } else if (page.name == "Smac Hybrid-A*") {
        if (node_->path_points_ >= 2) {
          summary = QString("Path tersedia: %1 pose, panjang %2 m. Planner: %3")
            .arg(static_cast<qulonglong>(node_->path_points_)).arg(node_->path_length_, 0, 'f', 2)
            .arg(QString::fromStdString(node_->planner_status_).isEmpty() ? "active" : QString::fromStdString(node_->planner_status_));
        } else {
          summary = "Planner belum memiliki path aktif; ini normal sebelum Goal Pose dikirim. Status/topik aktual tetap dipantau di bawah.";
        }
      } else if (page.name == "MPPI") {
        summary = node_->fresh("/cmd_vel_nav_raw", 1.5)
          ? "MPPI sedang TRACKING dan /cmd_vel_nav_raw aktif."
          : "MPPI tidak sedang mengeluarkan command; normal saat belum ada path/goal aktif.";
      } else if (page.map_slot > 0) {
        summary = QString("Map slot %1 — live /map, AMCL, EKF, scan, goal dan path ditampilkan dari runtime ROS 2.").arg(page.map_slot);
      }
      page.summary->setText(summary);

      if (page.image != nullptr && !page.image_topic.empty()) {
        const uint64_t generation = node_->imageGeneration(page.image_topic);
        if (generation != 0U && generation != page.image_generation) {
          const QImage img = node_->image(page.image_topic);
          if (!img.isNull()) {
            QSize target = page.image->size();
            if (target.width() < 120 || target.height() < 90) target = QSize(720, 405);
            page.image->setPixmap(QPixmap::fromImage(img).scaled(target, Qt::KeepAspectRatio, Qt::SmoothTransformation));
            page.image->setText(QString());
            page.image_generation = generation;
          }
        } else if (generation == 0U) {
          page.image->setText("Waiting for image: " + QString::fromStdString(page.image_topic));
        }
      }
    }
  }

  bool isLatchedTopic(const std::string & name) const
  {
    static const std::set<std::string> latched = {
      "/lidar/status", "/imu/status", "/map", "/smac_plan", "/system/autonomy_motion_allowed", "/system/manual_motion_allowed",
      "/sensor_guard/healthy", "/esc/ready", "/esc/armed", "/esc/feedback_valid",
      "/esc/drive/connected", "/esc/steer/connected", "/esc/status", "/esc/mux/active_source", "/esc/mux/status",
      "/camera/color/status", "/obstacle_detection/status",
      "/winch/connected", "/winch/port", "/winch/state", "/winch/top_limit", "/winch/bottom_limit",
      "/winch/pwm_pct", "/winch/direction", "/winch/servo_deg"};
    return latched.find(name) != latched.end();
  }

  void refreshTopics()
  {
    const double now = DashboardRosNode::steadyNow();
    for (int row = 0; row < static_cast<int>(topic_names_.size()); ++row) {
      const auto state = node_->topic(topic_names_[static_cast<size_t>(row)]);
      const bool has = state.count > 0;
      const double age = has ? now - state.last : 0.0;
      topic_table_->item(row, 1)->setText(has ? (age < 2.0 ? "DATA" : (isLatchedTopic(topic_names_[static_cast<size_t>(row)]) ? "LATCHED" : "STALE")) : "NO DATA");
      topic_table_->item(row, 2)->setText(has ? QString::number(state.hz, 'f', 1) : "-");
      topic_table_->item(row, 3)->setText(has ? QString::number(age, 'f', 2) : "-");
      topic_table_->item(row, 4)->setText(QString::number(static_cast<qulonglong>(state.count)));
      topic_table_->item(row, 5)->setText(state.type_frame.empty() ? "-" : QString::fromStdString(state.type_frame));
    }
  }

  void refreshTf()
  {
    const std::array<std::pair<std::string, std::string>, 4> transforms = {{{"map", "odom"}, {"odom", "base_footprint"}, {"base_footprint", "lidar_link"}, {"base_footprint", "imu_link"}}};
    for (int row = 0; row < static_cast<int>(transforms.size()); ++row) {
      double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
      const bool ok = node_->transformValues(transforms[static_cast<size_t>(row)].first, transforms[static_cast<size_t>(row)].second, x, y, z, yaw);
      tf_table_->item(row, 1)->setText(ok ? "OK" : "WAIT");
      tf_table_->item(row, 2)->setText(ok ? QString::number(x, 'f', 3) : "-");
      tf_table_->item(row, 3)->setText(ok ? QString::number(y, 'f', 3) : "-");
      tf_table_->item(row, 4)->setText(ok ? QString::number(z, 'f', 3) : "-");
      tf_table_->item(row, 5)->setText(ok ? QString::number(yaw, 'f', 2) : "-");
    }
  }

  QString mapPathForSlot(int slot) const
  {
    const QString filename = QString("map_%1.yaml").arg(slot);
    const QStringList candidates = {
      "/home/otomasi2/forclift/src/navigation/maps/" + filename,
      "/home/otomasi2/forclift/maps/" + filename,
      "/home/otomasi2/forclift/install/navigation/share/navigation/maps/" + filename};
    for (const auto & path : candidates) if (QFileInfo::exists(path)) return path;
    return candidates.front();
  }

  void refreshMaps()
  {
    std::array<QString, 3> paths;
    for (int slot = 1; slot <= 3; ++slot) {
      const QString path = mapPathForSlot(slot);
      paths[static_cast<size_t>(slot - 1)] = path;
      map_labels_[static_cast<size_t>(slot - 1)]->setText(
        QFileInfo::exists(path) ? QString("Map %1: FOUND\n%2").arg(slot).arg(path) : QString("Map %1: NOT FOUND\n%2").arg(slot).arg(path));
    }

    QFile pointer("/home/otomasi2/forclift/maps/latest_map.txt");
    QString active = "-";
    if (pointer.open(QIODevice::ReadOnly | QIODevice::Text)) active = QString::fromUtf8(pointer.readAll()).trimmed();

    int active_slot = 0;
    if (active != "-") {
      const QString active_base = QFileInfo(active).baseName();
      for (int slot = 1; slot <= 3; ++slot) {
        if (QFileInfo(paths[static_cast<size_t>(slot - 1)]).baseName() == active_base || active_base == QString("map_%1").arg(slot)) {
          active_slot = slot;
          break;
        }
      }
    }
    const QString active_kind = active_slot > 0 ? QString("MAP %1").arg(active_slot) : (active == "-" ? "OTHER / NONE" : "OTHER / EXTERNAL");
    active_map_label_->setText(QString("Active navigation map (map_server): %1\n%2").arg(active_kind).arg(active));
    active_map_short_ = active_slot > 0 ? QString("map_%1").arg(active_slot) : (active == "-" ? "OTHER/NONE" : QFileInfo(active).baseName());

    for (auto & page : telemetry_pages_) {
      if (page.map_slot <= 0 || page.map_info == nullptr) continue;
      const QString path = paths[static_cast<size_t>(page.map_slot - 1)];
      const bool exists = QFileInfo::exists(path);
      const bool active_here = page.map_slot == active_slot;
      page.map_info->setText(QString("Map %1: %2%3\n%4\nActive navigation map: %5")
        .arg(page.map_slot)
        .arg(exists ? "FOUND" : "NOT FOUND")
        .arg(active_here ? " | ACTIVE" : "")
        .arg(path)
        .arg(active == "-" ? "-" : active));
    }
  }

  void openRviz()
  {
    QString rviz = "/home/otomasi2/forclift/install/navigation/share/navigation/rviz/autonomous.rviz";
    if (!QFileInfo::exists(rviz)) rviz = "/home/otomasi2/forclift/src/navigation/rviz/autonomous.rviz";
    QProcess::startDetached("rviz2", QStringList{"-d", rviz}, "/home/otomasi2/forclift");
  }

  void openMapping()
  {
    QProcess::startDetached("ros2", QStringList{"launch", "navigation", "map.launch.py", "enable_rviz:=true"}, "/home/otomasi2/forclift");
  }

  std::shared_ptr<DashboardRosNode> node_;
  double start_time_{0.0};
  QTimer * timer_{nullptr};
  QPushButton * menu_toggle_{nullptr};
  QSplitter * left_body_splitter_{nullptr};
  QVBoxLayout * nav_layout_{nullptr};
  QStackedWidget * settings_stack_{nullptr};
  QStackedWidget * workspace_stack_{nullptr};
  std::vector<QPushButton *> nav_buttons_;
  bool nav_expanded_{true};

  QLabel * ros_settings_pill_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * active_map_label_{nullptr};
  std::array<QLabel *, 3> map_labels_{};
  QString active_map_short_{"OTHER/NONE"};
  QTableWidget * topic_table_{nullptr};
  QTableWidget * tf_table_{nullptr};
  std::vector<std::string> topic_names_;
  std::vector<TelemetryPage> telemetry_pages_;
  int refresh_counter_{19};

  StatusCard ros_, usb_, lidar_, lidar_odom_, imu_, ekf_, map_, amcl_, global_costmap_, smac_;
  StatusCard local_costmap_, mppi_, smoother_, collision_, sensor_guard_, autonomy_, manual_;
  StatusCard esc_, camera_, yolo_, fork_, winch_, tf_;
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int qt_argc = 1;
  char * qt_argv[] = {argv[0], nullptr};
  QApplication app(qt_argc, qt_argv);
  auto node = std::make_shared<navigation::DashboardRosNode>();
  navigation::DashboardWindow window(node);
  window.show();
  const int rc = app.exec();
  rclcpp::shutdown();
  return rc;
}
