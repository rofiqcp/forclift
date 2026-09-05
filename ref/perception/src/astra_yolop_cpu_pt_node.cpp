/*
 * Portable YOLOPv2 CPU backend for ROS 2 Humble.
 *
 * Direct TorchScript YOLOPv2 CPU backend for the Mini-PC runtime.
 * The node reads the official yolopv2.pt directly through LibTorch; there is
 * no PT->TorchScript conversion, TorchScript cache, or onnx Python dependency. Post-processing,
 * metric projection, and ROS safety streams remain compatible with the
 * navigation stack. If processing becomes too slow, downstream freshness
 * watchdogs stop autonomous motion instead of accepting stale perception.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "perception/perception_safety_core.hpp"

#include <opencv2/core.hpp>
#include <torch/script.h>
#include <torch/torch.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <unistd.h>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace perception
{

namespace
{

constexpr int MODEL_WIDTH = 640;
constexpr int MODEL_HEIGHT = 384;

rclcpp::QoS stateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

float sigmoid(float value)
{
  if (value >= 0.0F) {
    const float z = std::exp(-value);
    return 1.0F / (1.0F + z);
  }
  const float z = std::exp(value);
  return z / (1.0F + z);
}

struct Detection
{
  float x1{0.0F};
  float y1{0.0F};
  float x2{0.0F};
  float y2{0.0F};
  float score{0.0F};
  int class_id{-1};
};

struct MetricObstacle
{
  int class_id{-1};
  float score{0.0F};
  float forward_m{0.0F};
  float left_m{0.0F};
  float width_m{0.0F};
};

float intersectionOverUnion(const Detection & a, const Detection & b)
{
  const float left = std::max(a.x1, b.x1);
  const float top = std::max(a.y1, b.y1);
  const float right = std::min(a.x2, b.x2);
  const float bottom = std::min(a.y2, b.y2);
  const float intersection = std::max(0.0F, right - left) * std::max(0.0F, bottom - top);
  const float area_a = std::max(0.0F, a.x2 - a.x1) * std::max(0.0F, a.y2 - a.y1);
  const float area_b = std::max(0.0F, b.x2 - b.x1) * std::max(0.0F, b.y2 - b.y1);
  return intersection / (area_a + area_b - intersection + 1.0e-6F);
}

bool regularNonEmptyFile(const std::string & path)
{
  std::error_code error;
  return fs::is_regular_file(path, error) && !error && fs::file_size(path, error) > 0U && !error;
}

std::string resolveCpuModelPath(const std::string & requested)
{
  if (!requested.empty() && requested != "auto") {
    return fs::path(requested).lexically_normal().string();
  }

  std::vector<fs::path> candidates;
  if (const char * env = std::getenv("YOLOPV2_PT_PATH"); env && *env) {
    candidates.emplace_back(env);
  }
  if (const char * home = std::getenv("HOME"); home && *home) {
    candidates.emplace_back(fs::path(home) / "ros/models/yolopv2.pt");
  }
  std::error_code error;
  const fs::path cwd = fs::current_path(error);
  if (!error) {
    candidates.emplace_back(cwd / "models/yolopv2.pt");
  }
  // Lokasi deployment Mini-PC yang disepakati.
  candidates.emplace_back("/home/sirobo/ros/models/yolopv2.pt");

  for (const auto & candidate : candidates) {
    if (regularNonEmptyFile(candidate.string())) return candidate.lexically_normal().string();
  }
  return "auto";
}

int resolveCpuThreadCount(int requested)
{
  if (requested > 0) return std::clamp(requested, 1, 64);
  const unsigned int hardware = std::max(1U, std::thread::hardware_concurrency());
  // Sisakan core untuk ROS 2, GUI, sensor serial, EKF, dan Nav2.
  if (hardware <= 2U) return 1;
  return std::clamp(static_cast<int>(hardware) - 2, 1, 4);
}

}  // namespace

class AstraYolopCpuNode final : public rclcpp::Node
{
public:
  AstraYolopCpuNode()
  : Node("perception")
  {
    declareParameters();
    readParameters();
    validateParameters();
    loadModel();
    inference_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    createInterfaces();
    publishConnected(false);
    publishHealth(false, "STARTUP");
    publishEmergency(true);

    const auto inference_period = std::chrono::duration<double>(1.0 / inference_fps_);
    inference_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(inference_period),
      std::bind(&AstraYolopCpuNode::inferenceTick, this), inference_group_);
    const auto control_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
      std::bind(&AstraYolopCpuNode::controlTick, this), control_group_);

    RCLCPP_INFO(
      get_logger(),
      "YOLOPv2 CPU TorchScript aktif langsung: model=%s target_fps=%.1f threads=%d. "
      "Tidak ada konversi TorchScript pada runtime.",
      pt_model_path_.c_str(), inference_fps_, cpu_threads_);
  }

  ~AstraYolopCpuNode() override
  {
    if (capture_.isOpened()) capture_.release();
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("pt_model_path", "/home/sirobo/ros/models/yolopv2.pt");
    declare_parameter<double>("cpu_inference_fps", 2.0);
    // 0 = otomatis: maksimal 4 thread dan menyisakan core untuk ROS/Nav2/GUI.
    declare_parameter<int>("cpu_threads", 0);
    declare_parameter<std::string>("rgb_device", "auto");
    declare_parameter<int>("rgb_width", 1280);
    declare_parameter<int>("rgb_height", 720);
    declare_parameter<int>("fps", 30);
    declare_parameter<std::string>("v4l2_pixel_format", "MJPEG");
    declare_parameter<bool>("strict_camera_mode", false);
    declare_parameter<bool>("flip_horizontal", true);
    declare_parameter<bool>("camera_hotplug_retry", true);
    declare_parameter<double>("camera_retry_interval_sec", 2.0);
    declare_parameter<double>("confidence_threshold", 0.10);
    declare_parameter<double>("iou_threshold", 0.45);
    declare_parameter<double>("lane_threshold", 0.50);
    declare_parameter<int>("max_candidates", 16384);
    declare_parameter<int>("max_detections", 300);
    declare_parameter<bool>("publish_annotated", true);
    declare_parameter<bool>("publish_raw_rgb", true);
    declare_parameter<bool>("publish_drivable_mask", false);
    declare_parameter<bool>("publish_lane_mask", false);
    declare_parameter<bool>("publish_detections", false);
    declare_parameter<double>("overlay_alpha", 0.45);
    declare_parameter<int>("box_thickness", 2);
    declare_parameter<std::string>("frame_id", "camera_color_optical_frame");
    declare_parameter<std::string>("metric_frame_id", "base_footprint");
    declare_parameter<std::string>("annotated_topic", "/camera/yolop/image_annotated");
    declare_parameter<std::string>("raw_topic", "/camera/astra/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    declare_parameter<std::string>("detections_topic", "/yolop/detections");
    declare_parameter<std::string>("drivable_mask_topic", "/yolop/drivable_mask");
    declare_parameter<std::string>("lane_mask_topic", "/yolop/lane_mask");
    declare_parameter<std::string>("performance_topic", "/perception/performance");
    declare_parameter<std::string>("lane_metrics_topic", "/yolop/lane_metrics");
    declare_parameter<std::string>("drivable_space_topic", "/perception/drivable_space");
    declare_parameter<std::string>("perception_obstacle_metrics_topic", "/perception/obstacle_metrics");
    declare_parameter<std::string>("near_field_state_topic", "/perception/near_field_state");
    declare_parameter<std::string>("camera_connected_topic", "/perception/camera_connected");
    declare_parameter<std::string>("camera_health_topic", "/perception/camera_healthy");
    declare_parameter<std::string>("camera_health_state_topic", "/perception/camera_health_state");
    declare_parameter<std::string>("emergency_stop_topic", "/perception/emergency_stop");
    declare_parameter<std::string>("object_points_topic", "/perception/object_points");
    declare_parameter<std::string>("object_clearing_points_topic", "/perception/object_clearing_points");
    declare_parameter<std::string>("drivable_boundary_points_topic", "/perception/drivable_boundary_points");
    declare_parameter<std::string>("lane_state_topic", "/perception/lane_safety_state");
    declare_parameter<std::string>("lane_control_state_topic", "/perception/lane_control_state");
    declare_parameter<std::string>("raw_detection_summary_topic", "/perception/raw_detections");
    declare_parameter<std::string>("nav_cmd_topic", "/cmd_vel_nav_smoothed");
    declare_parameter<std::string>("safe_cmd_topic", "/cmd_vel/perception_advisory");
    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("cmd_timeout_sec", 0.50);
    declare_parameter<double>("lane_state_timeout_sec", 0.50);
    declare_parameter<bool>("camera_metric_calibration_validated", false);
    declare_parameter<bool>("lane_safety_enabled", false);
    declare_parameter<std::string>("control_mode", "active");
    declare_parameter<double>("wheelbase_m", 0.70);
    declare_parameter<double>("maximum_steering_angle_rad", 0.34);
    declare_parameter<double>("recenter_speed_mps", 0.20);
    declare_parameter<double>("critical_recenter_speed_mps", 0.10);
    declare_parameter<double>("center_gain", 0.55);
    declare_parameter<double>("heading_gain", 0.70);
    declare_parameter<double>("lane_blend_gain", 1.0);
    declare_parameter<double>("minimum_speed_for_yaw_limit_mps", 0.10);

    declare_parameter<int>("ground_calibration_width", 1280);
    declare_parameter<int>("ground_calibration_height", 720);
    declare_parameter<std::vector<double>>(
      "ground_src_points", {40.0, 680.0, 1240.0, 680.0, 760.0, 350.0, 520.0, 350.0});
    declare_parameter<std::vector<double>>(
      "ground_dst_points", {0.0, 513.5714, 1279.0, 503.30, 1279.0, 10.2714, 0.0, 0.0});
    declare_parameter<int>("ground_canvas_width", 1280);
    declare_parameter<int>("ground_canvas_height", 720);
    declare_parameter<double>("ground_origin_x_px", 596.8667);
    declare_parameter<double>("ground_origin_y_px", 719.0);
    declare_parameter<double>("ground_meters_per_pixel_x", 0.0039093041);
    declare_parameter<double>("ground_meters_per_pixel_y", 0.0097357441);
    declare_parameter<double>("metric_forward_offset_m", 0.0);
    declare_parameter<double>("metric_lateral_offset_m", 0.0);
    declare_parameter<double>("metric_minimum_forward_m", 0.20);
    declare_parameter<double>("metric_maximum_forward_m", 4.0);
    declare_parameter<double>("metric_maximum_abs_left_m", 2.5);
    declare_parameter<double>("minimum_obstacle_confidence", 0.30);
    declare_parameter<bool>("accept_all_detected_classes_as_obstacles", false);
    declare_parameter<std::vector<int64_t>>("safety_obstacle_class_ids", {0, 2, 3});
    declare_parameter<bool>("require_drivable_contact", true);
    declare_parameter<int>("drivable_contact_min_samples", 3);
    declare_parameter<double>("drivable_contact_min_fraction", 0.20);
    declare_parameter<int>("points_per_box", 5);
    declare_parameter<int>("clearing_ray_count", 41);
    declare_parameter<double>("clearing_fov_deg", 100.0);
    declare_parameter<double>("clearing_range_m", 4.5);
    declare_parameter<double>("clearing_obstacle_margin_m", 0.20);
    declare_parameter<double>("cpu_emergency_stop_distance_m", 0.65);
    declare_parameter<double>("cpu_emergency_half_width_m", 0.55);
    declare_parameter<double>("lane_vehicle_width_m", 0.55);
    declare_parameter<double>("edge_warning_clearance_m", 1.0);
    declare_parameter<double>("edge_critical_clearance_m", 0.40);
    declare_parameter<double>("edge_release_clearance_m", 1.20);
    declare_parameter<double>("center_deadband_m", 0.25);
    declare_parameter<int>("state_confirm_frames", 3);
    declare_parameter<int>("release_confirm_frames", 5);
    declare_parameter<int>("lost_confirm_frames", 2);
    declare_parameter<int>("camera_health_dark_luma", 18);
    declare_parameter<int>("camera_health_bright_luma", 245);
    declare_parameter<double>("camera_health_extreme_fraction", 0.97);
    declare_parameter<double>("camera_health_min_stddev", 5.0);
    declare_parameter<double>("camera_health_min_gradient", 2.0);
    declare_parameter<double>("camera_fx", 910.0);
    declare_parameter<double>("camera_fy", 910.0);
    declare_parameter<double>("camera_cx", 640.0);
    declare_parameter<double>("camera_cy", 360.0);
    declare_parameter<std::vector<double>>("camera_distortion", {0.0, 0.0, 0.0, 0.0, 0.0});
  }

  void readParameters()
  {
    pt_model_path_ = resolveCpuModelPath(get_parameter("pt_model_path").as_string());
    inference_fps_ = get_parameter("cpu_inference_fps").as_double();
    cpu_threads_ = resolveCpuThreadCount(get_parameter("cpu_threads").as_int());
    rgb_device_ = get_parameter("rgb_device").as_string();
    requested_width_ = get_parameter("rgb_width").as_int();
    requested_height_ = get_parameter("rgb_height").as_int();
    camera_fps_ = get_parameter("fps").as_int();
    pixel_format_ = get_parameter("v4l2_pixel_format").as_string();
    strict_camera_mode_ = get_parameter("strict_camera_mode").as_bool();
    flip_horizontal_ = get_parameter("flip_horizontal").as_bool();
    hotplug_retry_ = get_parameter("camera_hotplug_retry").as_bool();
    retry_sec_ = get_parameter("camera_retry_interval_sec").as_double();
    confidence_threshold_ = static_cast<float>(get_parameter("confidence_threshold").as_double());
    iou_threshold_ = static_cast<float>(get_parameter("iou_threshold").as_double());
    lane_threshold_ = static_cast<float>(get_parameter("lane_threshold").as_double());
    max_candidates_ = get_parameter("max_candidates").as_int();
    max_detections_ = get_parameter("max_detections").as_int();
    publish_annotated_ = get_parameter("publish_annotated").as_bool();
    publish_raw_ = get_parameter("publish_raw_rgb").as_bool();
    publish_drivable_ = get_parameter("publish_drivable_mask").as_bool();
    publish_lane_ = get_parameter("publish_lane_mask").as_bool();
    publish_detections_ = get_parameter("publish_detections").as_bool();
    overlay_alpha_ = get_parameter("overlay_alpha").as_double();
    box_thickness_ = get_parameter("box_thickness").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    metric_frame_id_ = get_parameter("metric_frame_id").as_string();
    annotated_topic_ = get_parameter("annotated_topic").as_string();
    raw_topic_ = get_parameter("raw_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    detections_topic_ = get_parameter("detections_topic").as_string();
    drivable_topic_ = get_parameter("drivable_mask_topic").as_string();
    lane_topic_ = get_parameter("lane_mask_topic").as_string();
    performance_topic_ = get_parameter("performance_topic").as_string();
    lane_metrics_topic_ = get_parameter("lane_metrics_topic").as_string();
    drivable_space_topic_ = get_parameter("drivable_space_topic").as_string();
    obstacle_metrics_topic_ = get_parameter("perception_obstacle_metrics_topic").as_string();
    near_field_state_topic_ = get_parameter("near_field_state_topic").as_string();
    camera_connected_topic_ = get_parameter("camera_connected_topic").as_string();
    camera_health_topic_ = get_parameter("camera_health_topic").as_string();
    camera_health_state_topic_ = get_parameter("camera_health_state_topic").as_string();
    emergency_topic_ = get_parameter("emergency_stop_topic").as_string();
    object_points_topic_ = get_parameter("object_points_topic").as_string();
    clearing_points_topic_ = get_parameter("object_clearing_points_topic").as_string();
    drivable_boundary_topic_ = get_parameter("drivable_boundary_points_topic").as_string();
    lane_state_topic_ = get_parameter("lane_state_topic").as_string();
    lane_control_state_topic_ = get_parameter("lane_control_state_topic").as_string();
    raw_detection_topic_ = get_parameter("raw_detection_summary_topic").as_string();
    nav_cmd_topic_ = get_parameter("nav_cmd_topic").as_string();
    safe_cmd_topic_ = get_parameter("safe_cmd_topic").as_string();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    cmd_timeout_sec_ = get_parameter("cmd_timeout_sec").as_double();
    lane_state_timeout_sec_ = get_parameter("lane_state_timeout_sec").as_double();
    camera_metric_calibration_validated_ =
      get_parameter("camera_metric_calibration_validated").as_bool();
    lane_safety_enabled_ = get_parameter("lane_safety_enabled").as_bool();
    control_mode_ = get_parameter("control_mode").as_string();
    mixer_config_.wheelbase_m = get_parameter("wheelbase_m").as_double();
    mixer_config_.maximum_steering_angle_rad =
      get_parameter("maximum_steering_angle_rad").as_double();
    mixer_config_.recenter_speed_mps = get_parameter("recenter_speed_mps").as_double();
    mixer_config_.critical_recenter_speed_mps =
      get_parameter("critical_recenter_speed_mps").as_double();
    mixer_config_.center_gain = get_parameter("center_gain").as_double();
    mixer_config_.heading_gain = get_parameter("heading_gain").as_double();
    mixer_config_.lane_blend_gain = get_parameter("lane_blend_gain").as_double();
    mixer_config_.minimum_speed_for_yaw_limit_mps =
      get_parameter("minimum_speed_for_yaw_limit_mps").as_double();

    ground_calibration_width_ = get_parameter("ground_calibration_width").as_int();
    ground_calibration_height_ = get_parameter("ground_calibration_height").as_int();
    ground_src_ = get_parameter("ground_src_points").as_double_array();
    ground_dst_ = get_parameter("ground_dst_points").as_double_array();
    ground_canvas_width_ = get_parameter("ground_canvas_width").as_int();
    ground_canvas_height_ = get_parameter("ground_canvas_height").as_int();
    ground_origin_x_ = get_parameter("ground_origin_x_px").as_double();
    ground_origin_y_ = get_parameter("ground_origin_y_px").as_double();
    ground_scale_x_ = get_parameter("ground_meters_per_pixel_x").as_double();
    ground_scale_y_ = get_parameter("ground_meters_per_pixel_y").as_double();
    forward_offset_ = get_parameter("metric_forward_offset_m").as_double();
    lateral_offset_ = get_parameter("metric_lateral_offset_m").as_double();
    min_forward_ = get_parameter("metric_minimum_forward_m").as_double();
    max_forward_ = get_parameter("metric_maximum_forward_m").as_double();
    max_abs_left_ = get_parameter("metric_maximum_abs_left_m").as_double();
    minimum_obstacle_confidence_ = get_parameter("minimum_obstacle_confidence").as_double();
    accept_all_obstacles_ = get_parameter("accept_all_detected_classes_as_obstacles").as_bool();
    safety_classes_ = get_parameter("safety_obstacle_class_ids").as_integer_array();
    require_drivable_contact_ = get_parameter("require_drivable_contact").as_bool();
    drivable_contact_min_samples_ = get_parameter("drivable_contact_min_samples").as_int();
    drivable_contact_min_fraction_ = get_parameter("drivable_contact_min_fraction").as_double();
    points_per_box_ = get_parameter("points_per_box").as_int();
    clearing_ray_count_ = get_parameter("clearing_ray_count").as_int();
    clearing_fov_rad_ = get_parameter("clearing_fov_deg").as_double() *
      3.14159265358979323846 / 180.0;
    clearing_range_m_ = get_parameter("clearing_range_m").as_double();
    clearing_obstacle_margin_m_ = get_parameter("clearing_obstacle_margin_m").as_double();
    emergency_distance_m_ = get_parameter("cpu_emergency_stop_distance_m").as_double();
    emergency_half_width_m_ = get_parameter("cpu_emergency_half_width_m").as_double();
    lane_vehicle_width_m_ = get_parameter("lane_vehicle_width_m").as_double();
    lane_thresholds_.warning_clearance_m = get_parameter("edge_warning_clearance_m").as_double();
    lane_thresholds_.critical_clearance_m = get_parameter("edge_critical_clearance_m").as_double();
    lane_thresholds_.release_clearance_m = get_parameter("edge_release_clearance_m").as_double();
    lane_thresholds_.center_deadband_m = get_parameter("center_deadband_m").as_double();
    lane_state_filter_ = std::make_unique<safety::ConfirmedState>(
      safety::LANE_LOST,
      get_parameter("state_confirm_frames").as_int(),
      get_parameter("release_confirm_frames").as_int(),
      get_parameter("lost_confirm_frames").as_int());
    health_dark_ = get_parameter("camera_health_dark_luma").as_int();
    health_bright_ = get_parameter("camera_health_bright_luma").as_int();
    health_extreme_fraction_ = get_parameter("camera_health_extreme_fraction").as_double();
    health_min_stddev_ = get_parameter("camera_health_min_stddev").as_double();
    health_min_gradient_ = get_parameter("camera_health_min_gradient").as_double();
    camera_fx_ = get_parameter("camera_fx").as_double();
    camera_fy_ = get_parameter("camera_fy").as_double();
    camera_cx_ = get_parameter("camera_cx").as_double();
    camera_cy_ = get_parameter("camera_cy").as_double();
    camera_distortion_ = get_parameter("camera_distortion").as_double_array();
  }

  void validateParameters()
  {
    if (!regularNonEmptyFile(pt_model_path_)) {
      throw std::runtime_error("YOLOPv2 .pt tidak ditemukan/kosong: " + pt_model_path_);
    }
    const fs::path pt_path(pt_model_path_);
    if (pt_path.extension() != ".pt" && pt_path.extension() != ".torchscript") {
      throw std::runtime_error("CPU backend membutuhkan model TorchScript .pt/.torchscript: " + pt_model_path_);
    }
    inference_fps_ = std::clamp(inference_fps_, 0.5, 30.0);
    cpu_threads_ = std::clamp(cpu_threads_, 1, 64);
    requested_width_ = std::max(320, requested_width_);
    requested_height_ = std::max(240, requested_height_);
    camera_fps_ = std::clamp(camera_fps_, 1, 60);
    std::transform(pixel_format_.begin(), pixel_format_.end(), pixel_format_.begin(),
      [](unsigned char value) {return static_cast<char>(std::toupper(value));});
    if (pixel_format_ != "MJPEG" && pixel_format_ != "YUYV") {
      throw std::runtime_error("v4l2_pixel_format CPU harus MJPEG atau YUYV");
    }
    retry_sec_ = std::clamp(retry_sec_, 0.5, 30.0);
    confidence_threshold_ = std::clamp(confidence_threshold_, 0.01F, 0.99F);
    iou_threshold_ = std::clamp(iou_threshold_, 0.01F, 0.99F);
    lane_threshold_ = std::clamp(lane_threshold_, 0.01F, 0.99F);
    max_candidates_ = std::clamp(max_candidates_, 100, 100000);
    max_detections_ = std::clamp(max_detections_, 1, 1000);
    overlay_alpha_ = std::clamp(overlay_alpha_, 0.0, 1.0);
    box_thickness_ = std::clamp(box_thickness_, 1, 10);
    control_rate_hz_ = std::clamp(control_rate_hz_, 5.0, 50.0);
    cmd_timeout_sec_ = std::clamp(cmd_timeout_sec_, 0.10, 2.0);
    lane_state_timeout_sec_ = std::clamp(lane_state_timeout_sec_, 0.10, 2.0);
    if (control_mode_ != "active" && control_mode_ != "monitor") {
      throw std::runtime_error("control_mode CPU harus 'active' atau 'monitor'");
    }
    mixer_config_.validate();
    if (ground_src_.size() != 8U || ground_dst_.size() != 8U ||
      ground_calibration_width_ <= 0 || ground_calibration_height_ <= 0 ||
      ground_canvas_width_ <= 0 || ground_canvas_height_ <= 0 ||
      ground_scale_x_ <= 0.0 || ground_scale_y_ <= 0.0 ||
      max_forward_ <= min_forward_ || max_abs_left_ <= 0.0)
    {
      throw std::runtime_error("Parameter homography/ground metric CPU tidak valid");
    }
    lane_thresholds_.validate();
    points_per_box_ = std::clamp(points_per_box_, 1, 21);
    clearing_ray_count_ = std::clamp(clearing_ray_count_, 3, 181);
    cv::setNumThreads(cpu_threads_);
  }

  static cv::Mat tensorToCv(const torch::Tensor & source)
  {
    auto tensor = source.detach().to(torch::kCPU).to(torch::kFloat32).contiguous();
    std::vector<int> sizes;
    sizes.reserve(static_cast<size_t>(tensor.dim()));
    for (const auto value : tensor.sizes()) {
      sizes.push_back(static_cast<int>(value));
    }
    cv::Mat view(static_cast<int>(sizes.size()), sizes.data(), CV_32F, tensor.data_ptr<float>());
    return view.clone();
  }

  std::vector<cv::Mat> forwardTorch(const cv::Mat & input)
  {
    cv::Mat rgb;
    cv::cvtColor(input, rgb, cv::COLOR_BGR2RGB);
    auto tensor = torch::from_blob(
      rgb.data, {1, rgb.rows, rgb.cols, 3},
      torch::TensorOptions().dtype(torch::kUInt8));
    tensor = tensor.permute({0, 3, 1, 2}).to(torch::kFloat32).div_(255.0).contiguous();

    torch::InferenceMode inference_guard;
    const auto output = module_.forward({tensor});
    if (!output.isTuple()) {
      throw std::runtime_error("Output root YOLOPv2 .pt harus tuple 3-item");
    }
    const auto root = output.toTuple()->elements();
    if (root.size() != 3U) {
      throw std::runtime_error("Output root YOLOPv2 .pt bukan 3-item ([pred,anchor],seg,lane)");
    }

    std::vector<torch::Tensor> prediction_heads;
    std::vector<torch::Tensor> anchor_heads;
    const auto extract_tensor_list = [](const torch::IValue & value, const char * label) {
      std::vector<torch::Tensor> tensors;
      if (value.isTensorList()) {
        for (const auto & tensor : value.toTensorVector()) tensors.push_back(tensor);
      } else if (value.isList()) {
        // GenericList iteration returns c10::ListElementReference on newer
        // LibTorch releases.  That proxy deliberately has no isTensor()/
        // toTensor() members.  Access through List::get() materializes the
        // underlying IValue and is compatible with ROS Humble-era and current
        // PyTorch C++ APIs.
        const auto list = value.toList();
        tensors.reserve(static_cast<size_t>(list.size()));
        for (size_t i = 0; i < static_cast<size_t>(list.size()); ++i) {
          const torch::IValue element = list.get(i);
          if (!element.isTensor()) {
            throw std::runtime_error(std::string(label) + " berisi non-tensor");
          }
          tensors.push_back(element.toTensor());
        }
      } else if (value.isTuple()) {
        for (const auto & element : value.toTuple()->elements()) {
          if (!element.isTensor()) throw std::runtime_error(std::string(label) + " berisi non-tensor");
          tensors.push_back(element.toTensor());
        }
      } else {
        throw std::runtime_error(std::string(label) + " bukan list/tuple tensor");
      }
      return tensors;
    };

    if (!root[0].isTuple() && !root[0].isList()) {
      throw std::runtime_error("Detection pack YOLOPv2 .pt bukan [pred, anchor_grid]");
    }
    std::vector<torch::IValue> det_items;
    if (root[0].isTuple()) det_items = root[0].toTuple()->elements();
    else {
      const auto list = root[0].toList();
      det_items.reserve(static_cast<size_t>(list.size()));
      for (size_t i = 0; i < static_cast<size_t>(list.size()); ++i) {
        det_items.push_back(list.get(i));
      }
    }
    if (det_items.size() != 2U) {
      throw std::runtime_error("Detection pack YOLOPv2 .pt harus tepat 2-item");
    }
    prediction_heads = extract_tensor_list(det_items[0], "pred");
    anchor_heads = extract_tensor_list(det_items[1], "anchor_grid");
    if (prediction_heads.size() != 3U || anchor_heads.size() != 3U) {
      throw std::runtime_error("YOLOPv2 .pt harus memiliki 3 prediction head dan 3 anchor head");
    }
    if (!root[1].isTensor() || !root[2].isTensor()) {
      throw std::runtime_error("Output drivable/lane YOLOPv2 .pt harus tensor");
    }

    std::vector<cv::Mat> outputs;
    outputs.reserve(8U);
    for (const auto & tensor_head : prediction_heads) outputs.push_back(tensorToCv(tensor_head));
    for (const auto & tensor_head : anchor_heads) outputs.push_back(tensorToCv(tensor_head));
    outputs.push_back(tensorToCv(root[1].toTensor()));
    outputs.push_back(tensorToCv(root[2].toTensor()));
    return outputs;
  }

  void loadModel()
  {
    torch::set_num_threads(cpu_threads_);
    // Inter-op parallelism >1 mudah meng-oversubscribe Mini-PC ketika OpenCV,
    // ROS executor, GUI, dan LibTorch aktif bersamaan.
    torch::set_num_interop_threads(1);
    try {
      module_ = torch::jit::load(pt_model_path_, torch::kCPU);
      module_.eval();
    } catch (const c10::Error & error) {
      throw std::runtime_error(
        std::string("LibTorch gagal membaca YOLOPv2 .pt sebagai TorchScript: ") + error.what());
    }

    cv::Mat zero_image = cv::Mat::zeros(MODEL_HEIGHT, MODEL_WIDTH, CV_8UC3);
    auto warmup_outputs = forwardTorch(zero_image);
    validateOutputs(warmup_outputs);
    RCLCPP_INFO(
      get_logger(),
      "YOLOPv2 CPU TorchScript warm-up + 8-output contract: PASS (%s)",
      pt_model_path_.c_str());
  }

  void createInterfaces()
  {
    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic_, image_qos);
    raw_pub_ = create_publisher<sensor_msgs::msg::Image>(raw_topic_, image_qos);
    drivable_pub_ = create_publisher<sensor_msgs::msg::Image>(drivable_topic_, image_qos);
    lane_pub_ = create_publisher<sensor_msgs::msg::Image>(lane_topic_, image_qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, image_qos);
    detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(detections_topic_, image_qos);
    performance_pub_ = create_publisher<std_msgs::msg::String>(performance_topic_, image_qos);
    lane_metrics_pub_ = create_publisher<std_msgs::msg::String>(lane_metrics_topic_, image_qos);
    drivable_space_pub_ = create_publisher<std_msgs::msg::String>(drivable_space_topic_, image_qos);
    obstacle_metrics_pub_ = create_publisher<std_msgs::msg::String>(obstacle_metrics_topic_, image_qos);
    near_field_state_pub_ = create_publisher<std_msgs::msg::String>(near_field_state_topic_, image_qos);
    connected_pub_ = create_publisher<std_msgs::msg::Bool>(camera_connected_topic_, stateQos());
    health_pub_ = create_publisher<std_msgs::msg::Bool>(camera_health_topic_, stateQos());
    health_state_pub_ = create_publisher<std_msgs::msg::String>(camera_health_state_topic_, stateQos());
    emergency_pub_ = create_publisher<std_msgs::msg::Bool>(emergency_topic_, stateQos());
    object_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(object_points_topic_, cloud_qos);
    clearing_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(clearing_points_topic_, cloud_qos);
    boundary_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(drivable_boundary_topic_, cloud_qos);
    lane_state_pub_ = create_publisher<std_msgs::msg::String>(lane_state_topic_, stateQos());
    lane_control_state_pub_ = create_publisher<std_msgs::msg::String>(lane_control_state_topic_, stateQos());
    raw_detection_pub_ = create_publisher<std_msgs::msg::String>(raw_detection_topic_, image_qos);
    advisory_pub_ = create_publisher<geometry_msgs::msg::Twist>(safe_cmd_topic_, image_qos);
    rclcpp::SubscriptionOptions nav_options;
    nav_options.callback_group = control_group_;
    nav_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_cmd_topic_, image_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_nav_cmd_ = *message;
        last_nav_cmd_time_ = now();
        have_nav_cmd_ = true;
      }, nav_options);
  }

  std::vector<std::string> candidateCameraDevices() const
  {
    if (rgb_device_ != "auto") return {rgb_device_};

    std::vector<std::string> candidates;
    std::error_code error;
    const fs::path by_id("/dev/v4l/by-id");
    if (fs::is_directory(by_id, error)) {
      for (const auto & entry : fs::directory_iterator(by_id, error)) {
        if (error) break;
        const std::string name = entry.path().filename().string();
        if (name.find("index0") != std::string::npos || name.find("video") != std::string::npos) {
          candidates.push_back(entry.path().string());
        }
      }
    }
    std::sort(candidates.begin(), candidates.end());

    // USB/V4L2 numbering is not stable across power cycles. Probe all present
    // numeric nodes after stable by-id symlinks instead of hard-coding video0.
    for (int index = 0; index < 64; ++index) {
      const std::string device = "/dev/video" + std::to_string(index);
      if (::access(device.c_str(), R_OK) == 0) candidates.push_back(device);
    }
    std::vector<std::string> unique;
    for (const auto & candidate : candidates) {
      if (std::find(unique.begin(), unique.end(), candidate) == unique.end()) unique.push_back(candidate);
    }
    return unique;
  }

  bool configureCameraCandidate(const std::string & device, cv::VideoCapture & candidate)
  {
    bool opened = false;
    if (!device.empty() && std::all_of(device.begin(), device.end(), [](unsigned char c) {
        return std::isdigit(c) != 0;
      })) {
      opened = candidate.open(std::stoi(device), cv::CAP_V4L2);
    } else {
      opened = candidate.open(device, cv::CAP_V4L2);
    }
    if (!opened) return false;

    const int fourcc = pixel_format_ == "YUYV" ? cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V') :
      cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    candidate.set(cv::CAP_PROP_FOURCC, fourcc);
    candidate.set(cv::CAP_PROP_FRAME_WIDTH, requested_width_);
    candidate.set(cv::CAP_PROP_FRAME_HEIGHT, requested_height_);
    candidate.set(cv::CAP_PROP_FPS, camera_fps_);
    candidate.set(cv::CAP_PROP_BUFFERSIZE, 1.0);

    const double actual_width = candidate.get(cv::CAP_PROP_FRAME_WIDTH);
    const double actual_height = candidate.get(cv::CAP_PROP_FRAME_HEIGHT);
    const double actual_fps = candidate.get(cv::CAP_PROP_FPS);
    const double fps_tolerance = std::max(1.0, 0.05 * static_cast<double>(camera_fps_));
    const bool resolution_matches =
      std::abs(actual_width - requested_width_) <= 0.5 &&
      std::abs(actual_height - requested_height_) <= 0.5;
    const bool fps_matches = actual_fps > 0.0 &&
      std::abs(actual_fps - static_cast<double>(camera_fps_)) <= fps_tolerance;
    if (strict_camera_mode_ && (!resolution_matches || !fps_matches)) {
      // CAMERA_MODE_MISMATCH: reject this V4L2 node and continue probing other
      // candidates; do not kill the localization/navigation stack.
      candidate.release();
      return false;
    }

    // A V4L2 node may open successfully even when it is metadata/depth-only.
    // Accept the node only after a real BGR frame can be dequeued.
    cv::Mat probe;
    bool frame_ok = false;
    for (int attempt = 0; attempt < 4; ++attempt) {
      if (candidate.read(probe) && !probe.empty() && probe.cols > 0 && probe.rows > 0 &&
        probe.type() == CV_8UC3)
      {
        frame_ok = true;
        break;
      }
    }
    if (!frame_ok) {
      candidate.release();
      return false;
    }

    if (!resolution_matches || !fps_matches) {
      RCLCPP_WARN(
        get_logger(),
        "Mode kamera CPU fallback: device=%s request=%dx%d@%d actual=%.0fx%.0f@%.2f (strict=false)",
        device.c_str(), requested_width_, requested_height_, camera_fps_,
        actual_width, actual_height, actual_fps);
    }
    return true;
  }

  bool openCamera()
  {
    if (capture_.isOpened()) return true;
    if (!hotplug_retry_ && last_open_attempt_.time_since_epoch().count() != 0) return false;
    const auto t = std::chrono::steady_clock::now();
    if (last_open_attempt_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(t - last_open_attempt_).count() < retry_sec_) return false;
    last_open_attempt_ = t;

    const auto candidates = candidateCameraDevices();
    for (const auto & device : candidates) {
      cv::VideoCapture candidate;
      if (!configureCameraCandidate(device, candidate)) continue;
      capture_ = std::move(candidate);
      publishConnected(true);
      const double actual_width = capture_.get(cv::CAP_PROP_FRAME_WIDTH);
      const double actual_height = capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
      const double actual_fps = capture_.get(cv::CAP_PROP_FPS);
      RCLCPP_INFO(
        get_logger(), "Kamera CPU terbuka: %s %.0fx%.0f @ %.1f FPS",
        device.c_str(), actual_width, actual_height, actual_fps);
      return true;
    }

    publishConnected(false);
    publishHealth(false, "CAMERA_OPEN_FAILED");
    publishEmergency(true);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Kamera CPU belum tersedia/usable; %zu kandidat V4L2 sudah diprobe, menunggu hot-plug",
      candidates.size());
    return false;
  }

  cv::Mat letterbox(const cv::Mat & frame)
  {
    gain_ = std::min(
      static_cast<float>(MODEL_WIDTH) / static_cast<float>(frame.cols),
      static_cast<float>(MODEL_HEIGHT) / static_cast<float>(frame.rows));
    resized_width_ = std::max(1, static_cast<int>(std::round(frame.cols * gain_)));
    resized_height_ = std::max(1, static_cast<int>(std::round(frame.rows * gain_)));
    pad_x_ = (MODEL_WIDTH - resized_width_) / 2;
    pad_y_ = (MODEL_HEIGHT - resized_height_) / 2;
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(resized_width_, resized_height_), 0.0, 0.0, cv::INTER_LINEAR);
    cv::Mat canvas(MODEL_HEIGHT, MODEL_WIDTH, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(pad_x_, pad_y_, resized_width_, resized_height_)));
    return canvas;
  }

  void validateOutputs(const std::vector<cv::Mat> & outputs) const
  {
    if (outputs.size() != 8U) throw std::runtime_error("TorchScript wajib memiliki tepat 8 output YOLOPv2");
    for (const auto & output : outputs) {
      if (output.empty() || output.type() != CV_32F || !output.isContinuous() ||
        !cv::checkRange(output, true, nullptr, -std::numeric_limits<double>::max(),
          std::numeric_limits<double>::max()))
      {
        throw std::runtime_error("Output TorchScript wajib float32 kontinu dan tanpa NaN/Inf");
      }
    }
    for (int head = 0; head < 3; ++head) {
      const cv::Mat & value = outputs[static_cast<size_t>(head)];
      const int stride = 1 << (head + 3);
      if (value.dims != 4 || value.size[0] != 1 || value.size[1] != 255 ||
        value.size[2] != MODEL_HEIGHT / stride || value.size[3] != MODEL_WIDTH / stride ||
        value.total() != static_cast<size_t>(255 * (MODEL_HEIGHT / stride) * (MODEL_WIDTH / stride)))
      {
        throw std::runtime_error("Shape detection head TorchScript tidak sesuai kontrak YOLOPv2");
      }
      const cv::Mat & anchor = outputs[static_cast<size_t>(head + 3)];
      if (anchor.total() != 6U) throw std::runtime_error("Anchor-grid TorchScript harus berisi 6 float/head");
    }
    if (outputs[6].dims != 4 || outputs[6].size[0] != 1 || outputs[6].size[1] != 2 ||
      outputs[6].size[2] != MODEL_HEIGHT || outputs[6].size[3] != MODEL_WIDTH ||
      outputs[7].dims != 4 || outputs[7].size[0] != 1 || outputs[7].size[1] != 1 ||
      outputs[7].size[2] != MODEL_HEIGHT || outputs[7].size[3] != MODEL_WIDTH)
    {
      throw std::runtime_error("Shape segmentation TorchScript tidak sesuai kontrak YOLOPv2");
    }
  }

  std::vector<Detection> decodeDetections(const std::vector<cv::Mat> & outputs, int width, int height)
  {
    std::vector<Detection> candidates;
    candidates.reserve(static_cast<size_t>(std::min(max_candidates_, 20000)));
    for (int head_index = 0; head_index < 3; ++head_index) {
      const cv::Mat & head = outputs[static_cast<size_t>(head_index)];
      const float * values = head.ptr<float>();
      const float * anchors = outputs[static_cast<size_t>(head_index + 3)].ptr<float>();
      const int grid_h = head.size[2];
      const int grid_w = head.size[3];
      const int area = grid_h * grid_w;
      const int attrs = head.size[1] / 3;
      const int classes = attrs - 5;
      const int stride = MODEL_WIDTH / grid_w;
      for (int anchor = 0; anchor < 3 && static_cast<int>(candidates.size()) < max_candidates_; ++anchor) {
        const int base = anchor * attrs * area;
        for (int cell = 0; cell < area && static_cast<int>(candidates.size()) < max_candidates_; ++cell) {
          const float objectness = sigmoid(values[base + 4 * area + cell]);
          if (objectness < confidence_threshold_) continue;
          float best_score = 0.0F;
          int best_class = -1;
          for (int cls = 0; cls < classes; ++cls) {
            const float score = objectness * sigmoid(values[base + (5 + cls) * area + cell]);
            if (score > best_score) {best_score = score; best_class = cls;}
          }
          if (best_score < confidence_threshold_) continue;
          const int gy = cell / grid_w;
          const int gx = cell % grid_w;
          const float tx = sigmoid(values[base + cell]);
          const float ty = sigmoid(values[base + area + cell]);
          const float tw = sigmoid(values[base + 2 * area + cell]);
          const float th = sigmoid(values[base + 3 * area + cell]);
          const float cx = (tx * 2.0F - 0.5F + gx) * stride;
          const float cy = (ty * 2.0F - 0.5F + gy) * stride;
          const float box_w = std::pow(tw * 2.0F, 2.0F) * anchors[anchor * 2];
          const float box_h = std::pow(th * 2.0F, 2.0F) * anchors[anchor * 2 + 1];
          candidates.push_back({
            cx - box_w * 0.5F, cy - box_h * 0.5F,
            cx + box_w * 0.5F, cy + box_h * 0.5F, best_score, best_class});
        }
      }
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      return a.score > b.score;
    });
    std::vector<Detection> result;
    std::vector<bool> suppressed(candidates.size(), false);
    for (size_t i = 0; i < candidates.size() && static_cast<int>(result.size()) < max_detections_; ++i) {
      if (suppressed[i]) continue;
      Detection d = candidates[i];
      for (size_t j = i + 1; j < candidates.size(); ++j) {
        if (!suppressed[j] && candidates[j].class_id == d.class_id &&
          intersectionOverUnion(d, candidates[j]) > iou_threshold_) suppressed[j] = true;
      }
      d.x1 = std::clamp((d.x1 - pad_x_) / gain_, 0.0F, static_cast<float>(width - 1));
      d.y1 = std::clamp((d.y1 - pad_y_) / gain_, 0.0F, static_cast<float>(height - 1));
      d.x2 = std::clamp((d.x2 - pad_x_) / gain_, 0.0F, static_cast<float>(width - 1));
      d.y2 = std::clamp((d.y2 - pad_y_) / gain_, 0.0F, static_cast<float>(height - 1));
      if (d.x2 > d.x1 && d.y2 > d.y1) result.push_back(d);
    }
    return result;
  }

  void decodeMasks(
    const std::vector<cv::Mat> & outputs, int width, int height,
    cv::Mat & drivable, cv::Mat & lane)
  {
    const size_t plane = static_cast<size_t>(MODEL_WIDTH * MODEL_HEIGHT);
    const float * da = outputs[6].ptr<float>();
    const float * ll = outputs[7].ptr<float>();
    cv::Mat da_small(MODEL_HEIGHT, MODEL_WIDTH, CV_8UC1);
    cv::Mat ll_small(MODEL_HEIGHT, MODEL_WIDTH, CV_8UC1);
    for (size_t i = 0; i < plane; ++i) {
      da_small.data[i] = da[plane + i] > da[i] ? 255U : 0U;
      ll_small.data[i] = ll[i] > lane_threshold_ ? 255U : 0U;
    }
    const cv::Rect valid_roi(pad_x_, pad_y_, resized_width_, resized_height_);
    cv::resize(da_small(valid_roi), drivable, cv::Size(width, height), 0.0, 0.0, cv::INTER_NEAREST);
    cv::resize(ll_small(valid_roi), lane, cv::Size(width, height), 0.0, 0.0, cv::INTER_NEAREST);
  }

  void prepareHomography(int width, int height)
  {
    if (!homography_.empty() && homography_width_ == width && homography_height_ == height) return;
    std::vector<cv::Point2f> source(4), destination(4);
    const float sx = static_cast<float>(width) / ground_calibration_width_;
    const float sy = static_cast<float>(height) / ground_calibration_height_;
    for (size_t i = 0; i < 4U; ++i) {
      source[i] = cv::Point2f(
        static_cast<float>(ground_src_[i * 2] * sx),
        static_cast<float>(ground_src_[i * 2 + 1] * sy));
      destination[i] = cv::Point2f(
        static_cast<float>(ground_dst_[i * 2]), static_cast<float>(ground_dst_[i * 2 + 1]));
    }
    homography_ = cv::getPerspectiveTransform(source, destination);
    if (homography_.empty()) throw std::runtime_error("Homography CPU tidak dapat dihitung");
    homography_width_ = width;
    homography_height_ = height;
  }

  bool projectPixel(float x, float y, float & forward, float & left) const
  {
    if (homography_.empty()) return false;
    std::vector<cv::Point2f> input{{x, y}};
    std::vector<cv::Point2f> output;
    cv::perspectiveTransform(input, output, homography_);
    if (output.size() != 1U || !std::isfinite(output[0].x) || !std::isfinite(output[0].y) ||
      output[0].x < 0.0F || output[0].x >= ground_canvas_width_ ||
      output[0].y < 0.0F || output[0].y >= ground_canvas_height_) return false;
    forward = static_cast<float>((ground_origin_y_ - output[0].y) * ground_scale_y_ + forward_offset_);
    left = static_cast<float>((ground_origin_x_ - output[0].x) * ground_scale_x_ + lateral_offset_);
    return std::isfinite(forward) && std::isfinite(left);
  }

  bool drivableContact(const Detection & d, const cv::Mat & drivable) const
  {
    if (!require_drivable_contact_) return true;
    const int bottom = std::clamp(static_cast<int>(std::lround(d.y2)), 0, drivable.rows - 1);
    int road = 0;
    int total = 0;
    for (int dy : {-8, -2, 2}) {
      const int y = std::clamp(bottom + dy, 0, drivable.rows - 1);
      for (int sample = 0; sample < 5; ++sample) {
        const float alpha = sample * 0.25F;
        const int x = std::clamp(
          static_cast<int>(std::lround(d.x1 + alpha * (d.x2 - d.x1))), 0, drivable.cols - 1);
        ++total;
        if (drivable.at<uint8_t>(y, x) > 0U) ++road;
      }
    }
    return road >= drivable_contact_min_samples_ &&
      static_cast<double>(road) / std::max(1, total) >= drivable_contact_min_fraction_;
  }

  bool isSafetyClass(int class_id) const
  {
    return accept_all_obstacles_ || std::find(safety_classes_.begin(), safety_classes_.end(), class_id) != safety_classes_.end();
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
    const rclcpp::Time & stamp, const std::vector<std::array<float, 4>> & points) const
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = metric_frame_id_;
    message.height = 1U;
    message.width = static_cast<uint32_t>(points.size());
    message.is_bigendian = false;
    message.is_dense = true;
    message.point_step = 16U;
    message.row_step = message.point_step * message.width;
    message.fields.resize(4U);
    const char * names[] = {"x", "y", "z", "intensity"};
    for (size_t i = 0; i < 4U; ++i) {
      message.fields[i].name = names[i];
      message.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
      message.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      message.fields[i].count = 1U;
    }
    message.data.resize(points.size() * 4U * sizeof(float));
    if (!points.empty()) std::memcpy(message.data.data(), points.data(), message.data.size());
    return message;
  }

  std::vector<std::array<float, 4>> projectObstacles(
    const std::vector<Detection> & detections, const cv::Mat & drivable,
    bool & emergency, std::vector<std::pair<float, float>> & centers,
    std::vector<MetricObstacle> & metric_obstacles)
  {
    std::vector<std::array<float, 4>> points;
    emergency = false;
    centers.clear();
    metric_obstacles.clear();
    for (const auto & d : detections) {
      if (!isSafetyClass(d.class_id) || d.score < minimum_obstacle_confidence_ ||
        !drivableContact(d, drivable))
      {
        continue;
      }
      const float center_x = 0.5F * (d.x1 + d.x2);
      float forward = 0.0F;
      float left = 0.0F;
      if (!projectPixel(center_x, d.y2, forward, left) ||
        forward < min_forward_ || forward > max_forward_ || std::abs(left) > max_abs_left_)
      {
        continue;
      }
      float left_edge_forward = 0.0F;
      float left_edge = 0.0F;
      float right_edge_forward = 0.0F;
      float right_edge = 0.0F;
      float width = 0.35F;
      if (projectPixel(d.x1, d.y2, left_edge_forward, left_edge) &&
        projectPixel(d.x2, d.y2, right_edge_forward, right_edge))
      {
        width = std::clamp(std::abs(left_edge - right_edge), 0.20F, 2.5F);
      }
      centers.emplace_back(forward, left);
      metric_obstacles.push_back(MetricObstacle{d.class_id, d.score, forward, left, width});
      if (forward <= emergency_distance_m_ &&
        std::abs(left) <= emergency_half_width_m_ + 0.5F * width)
      {
        emergency = true;
      }
      const float half = 0.5F * width;
      for (int i = 0; i < points_per_box_; ++i) {
        const float ratio = points_per_box_ <= 1 ? 0.5F :
          static_cast<float>(i) / static_cast<float>(points_per_box_ - 1);
        points.push_back({forward, left - half + ratio * 2.0F * half, 0.20F, d.score});
      }
    }
    return points;
  }

  std::vector<std::array<float, 4>> buildDrivableBoundary(
    const cv::Mat & drivable, bool & valid, double & left_clearance,
    double & right_clearance, double & center_error, double & heading_error,
    double & road_width, int & valid_rows)
  {
    std::vector<std::array<float, 4>> points;
    std::vector<double> left_values;
    std::vector<double> right_values;
    std::vector<double> width_values;
    std::vector<std::pair<double, double>> center_samples;
    valid_rows = 0;
    const int start_y = std::max(0, static_cast<int>(drivable.rows * 0.42));
    const int step = std::max(4, drivable.rows / 48);
    for (int y = drivable.rows - 2; y >= start_y; y -= step) {
      int left_pixel = -1;
      int right_pixel = -1;
      const auto * row = drivable.ptr<uint8_t>(y);
      for (int x = 0; x < drivable.cols; ++x) {
        if (row[x] > 0U) {left_pixel = x; break;}
      }
      for (int x = drivable.cols - 1; x >= 0; --x) {
        if (row[x] > 0U) {right_pixel = x; break;}
      }
      if (left_pixel < 0 || right_pixel <= left_pixel) continue;
      float lf = 0.0F, ll = 0.0F, rf = 0.0F, rl = 0.0F;
      if (projectPixel(static_cast<float>(left_pixel), static_cast<float>(y), lf, ll) &&
        projectPixel(static_cast<float>(right_pixel), static_cast<float>(y), rf, rl))
      {
        if (lf >= min_forward_ && lf <= max_forward_ && std::abs(ll) <= max_abs_left_) {
          points.push_back({lf, ll, 0.20F, 1.0F});
        }
        if (rf >= min_forward_ && rf <= max_forward_ && std::abs(rl) <= max_abs_left_) {
          points.push_back({rf, rl, 0.20F, 1.0F});
        }
        const double average_forward = 0.5 * (static_cast<double>(lf) + static_cast<double>(rf));
        if (average_forward >= 0.8 && average_forward <= 2.5 && ll > rl) {
          left_values.push_back(ll);
          right_values.push_back(rl);
          width_values.push_back(static_cast<double>(ll - rl));
          center_samples.emplace_back(average_forward, 0.5 * static_cast<double>(ll + rl));
          ++valid_rows;
        }
      }
    }
    valid = left_values.size() >= 4U && right_values.size() >= 4U;
    left_clearance = right_clearance = center_error = heading_error = road_width = 0.0;
    if (valid) {
      const auto mean = [](const std::vector<double> & values) {
        return std::accumulate(values.begin(), values.end(), 0.0) /
          static_cast<double>(values.size());
      };
      const double left = mean(left_values);
      const double right = mean(right_values);
      road_width = mean(width_values);
      left_clearance = left - 0.5 * lane_vehicle_width_m_;
      right_clearance = -right - 0.5 * lane_vehicle_width_m_;
      center_error = 0.5 * (left + right);
      if (center_samples.size() >= 3U) {
        double mean_x = 0.0;
        double mean_y = 0.0;
        for (const auto & sample : center_samples) {
          mean_x += sample.first;
          mean_y += sample.second;
        }
        mean_x /= static_cast<double>(center_samples.size());
        mean_y /= static_cast<double>(center_samples.size());
        double covariance = 0.0;
        double variance = 0.0;
        for (const auto & sample : center_samples) {
          const double dx = sample.first - mean_x;
          covariance += dx * (sample.second - mean_y);
          variance += dx * dx;
        }
        if (variance > 1.0e-9) heading_error = std::atan(covariance / variance);
      }
      valid = left_clearance >= 0.0 && right_clearance >= 0.0 &&
        std::isfinite(heading_error) && road_width > lane_vehicle_width_m_;
    }
    return points;
  }

  void publishClearingFan(
    const rclcpp::Time & stamp, const std::vector<std::pair<float, float>> & obstacles)
  {
    std::vector<std::array<float, 4>> points;
    for (int i = 0; i < clearing_ray_count_; ++i) {
      const double ratio = static_cast<double>(i) / std::max(1, clearing_ray_count_ - 1);
      const double angle = -0.5 * clearing_fov_rad_ + ratio * clearing_fov_rad_;
      bool blocked = false;
      for (const auto & obstacle : obstacles) {
        const double distance = std::hypot(obstacle.first, obstacle.second);
        const double obstacle_angle = std::atan2(obstacle.second, obstacle.first);
        const double half_angle = std::atan2(clearing_obstacle_margin_m_ + 0.25, std::max(0.05, distance));
        const double error = std::atan2(std::sin(angle - obstacle_angle), std::cos(angle - obstacle_angle));
        if (std::abs(error) <= half_angle) {blocked = true; break;}
      }
      if (!blocked) {
        points.push_back({
          static_cast<float>(clearing_range_m_ * std::cos(angle)),
          static_cast<float>(clearing_range_m_ * std::sin(angle)), 0.20F, 0.0F});
      }
    }
    clearing_pub_->publish(makeCloud(stamp, points));
  }

  bool cameraHealthy(
    const cv::Mat & frame, double & mean_luma, double & stddev,
    double & extreme_fraction, double & mean_gradient) const
  {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Scalar mean_value;
    cv::Scalar std_value;
    cv::meanStdDev(gray, mean_value, std_value);
    mean_luma = mean_value[0];
    stddev = std_value[0];
    int extreme = 0;
    int total = 0;
    double gradient_sum = 0.0;
    int gradient_count = 0;
    const int stride = std::max(4, std::min(frame.cols, frame.rows) / 80);
    for (int y = 0; y < gray.rows; y += stride) {
      const auto * row = gray.ptr<uint8_t>(y);
      for (int x = 0; x < gray.cols; x += stride) {
        ++total;
        if (row[x] <= health_dark_ || row[x] >= health_bright_) ++extreme;
        if (x + stride < gray.cols) {
          gradient_sum += std::abs(static_cast<int>(row[x + stride]) - static_cast<int>(row[x]));
          ++gradient_count;
        }
        if (y + stride < gray.rows) {
          const auto * next = gray.ptr<uint8_t>(y + stride);
          gradient_sum += std::abs(static_cast<int>(next[x]) - static_cast<int>(row[x]));
          ++gradient_count;
        }
      }
    }
    extreme_fraction = total > 0 ? static_cast<double>(extreme) / total : 1.0;
    mean_gradient = gradient_count > 0 ? gradient_sum / gradient_count : 0.0;
    return stddev >= health_min_stddev_ && mean_gradient >= health_min_gradient_ &&
      extreme_fraction < health_extreme_fraction_;
  }

  void publishImage(
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher,
    const rclcpp::Time & stamp, const cv::Mat & image, const std::string & encoding)
  {
    if (!publisher || publisher->get_subscription_count() == 0U) return;
    cv::Mat source = image.isContinuous() ? image : image.clone();
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    message->header.stamp = stamp;
    message->header.frame_id = frame_id_;
    message->height = static_cast<uint32_t>(source.rows);
    message->width = static_cast<uint32_t>(source.cols);
    message->encoding = encoding;
    message->is_bigendian = false;
    message->step = static_cast<uint32_t>(source.cols * source.elemSize());
    message->data.assign(source.data, source.data + source.total() * source.elemSize());
    publisher->publish(std::move(message));
  }

  void publishCameraInfo(const rclcpp::Time & stamp, int width, int height)
  {
    sensor_msgs::msg::CameraInfo message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.width = static_cast<uint32_t>(width);
    message.height = static_cast<uint32_t>(height);
    message.distortion_model = "plumb_bob";
    message.d = camera_distortion_;
    message.k = {camera_fx_, 0.0, camera_cx_, 0.0, camera_fy_, camera_cy_, 0.0, 0.0, 1.0};
    message.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    message.p = {camera_fx_, 0.0, camera_cx_, 0.0, 0.0, camera_fy_, camera_cy_, 0.0, 0.0, 0.0, 1.0, 0.0};
    camera_info_pub_->publish(std::move(message));
  }

  void publishDetectionMessages(const rclcpp::Time & stamp, const std::vector<Detection> & detections)
  {
    if (publish_detections_ && detections_pub_->get_subscription_count() > 0U) {
      vision_msgs::msg::Detection2DArray array;
      array.header.stamp = stamp;
      array.header.frame_id = frame_id_;
      for (const auto & source : detections) {
        vision_msgs::msg::Detection2D detection;
        detection.header = array.header;
        detection.bbox.center.position.x = 0.5 * (source.x1 + source.x2);
        detection.bbox.center.position.y = 0.5 * (source.y1 + source.y2);
        detection.bbox.center.theta = 0.0;
        detection.bbox.size_x = source.x2 - source.x1;
        detection.bbox.size_y = source.y2 - source.y1;
        vision_msgs::msg::ObjectHypothesisWithPose result;
        result.hypothesis.class_id = std::to_string(source.class_id);
        result.hypothesis.score = source.score;
        detection.results.push_back(std::move(result));
        array.detections.push_back(std::move(detection));
      }
      detections_pub_->publish(std::move(array));
    }
    std::ostringstream summary;
    summary << std::fixed << std::setprecision(3) << "backend=cpu;count=" << detections.size();
    for (size_t i = 0; i < detections.size(); ++i) {
      summary << ";d" << i << "=cls:" << detections[i].class_id << ",score:" << detections[i].score;
    }
    std_msgs::msg::String message;
    message.data = summary.str();
    raw_detection_pub_->publish(std::move(message));
  }

  double bottomDrivableFraction(const cv::Mat & drivable) const
  {
    if (drivable.empty()) return 0.0;
    const int y0 = std::clamp(static_cast<int>(std::lround(drivable.rows * 0.78)), 0, drivable.rows - 1);
    const cv::Mat roi = drivable.rowRange(y0, drivable.rows);
    return static_cast<double>(cv::countNonZero(roi)) /
      static_cast<double>(std::max<size_t>(1U, roi.total()));
  }

  void publishObstacleMetrics(
    const rclcpp::Time & stamp, const std::vector<MetricObstacle> & obstacles)
  {
    if (!obstacle_metrics_pub_) return;
    std::ostringstream json;
    json << std::fixed << std::setprecision(5)
         << "{\"backend\":\"cpu\",\"stamp_ns\":" << stamp.nanoseconds()
         << ",\"frame_id\":\"" << metric_frame_id_ << "\",\"mode\":\"cpu_homography_ready\""
         << ",\"count\":" << obstacles.size() << ",\"detections\":[";
    for (size_t i = 0; i < obstacles.size(); ++i) {
      if (i) json << ',';
      const auto & obstacle = obstacles[i];
      json << "{\"class_id\":" << obstacle.class_id
           << ",\"score\":" << obstacle.score
           << ",\"forward_homography_m\":" << obstacle.forward_m
           << ",\"forward_area_m\":null"
           << ",\"forward_m\":" << obstacle.forward_m
           << ",\"left_m\":" << obstacle.left_m
           << ",\"metric_width_m\":" << obstacle.width_m << '}';
    }
    json << "]}";
    std_msgs::msg::String message;
    message.data = json.str();
    obstacle_metrics_pub_->publish(std::move(message));
  }

  void publishNearFieldState(
    bool emergency, const std::vector<MetricObstacle> & obstacles, double drivable_fraction)
  {
    int class_id = -1;
    double confidence = 0.0;
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto & obstacle : obstacles) {
      if (obstacle.forward_m < nearest &&
        std::abs(obstacle.left_m) <= emergency_half_width_m_ + 0.5 * obstacle.width_m)
      {
        nearest = obstacle.forward_m;
        class_id = obstacle.class_id;
        confidence = obstacle.score;
      }
    }
    std::ostringstream json;
    json << std::boolalpha << std::fixed << std::setprecision(4)
         << "{\"backend\":\"cpu\",\"emergency\":" << emergency
         << ",\"reason\":\"" << (emergency ? "METRIC_NEAR_FIELD" : "CLEAR") << "\""
         << ",\"raw_class_id\":" << class_id
         << ",\"confidence\":" << confidence
         << ",\"near_field_drivable_fraction\":" << drivable_fraction << '}';
    std_msgs::msg::String message;
    message.data = json.str();
    near_field_state_pub_->publish(std::move(message));
  }

  void publishDrivableSpace(
    const rclcpp::Time & stamp, bool valid, int valid_rows, size_t boundary_points)
  {
    std::ostringstream json;
    json << std::boolalpha << std::fixed << std::setprecision(4)
         << "{\"backend\":\"cpu\",\"stamp_ns\":" << stamp.nanoseconds()
         << ",\"frame_id\":\"" << metric_frame_id_ << "\""
         << ",\"valid_rows\":" << valid_rows
         << ",\"sample_rows\":48"
         << ",\"boundary_points\":" << boundary_points
         << ",\"valid\":" << valid << '}';
    std_msgs::msg::String message;
    message.data = json.str();
    drivable_space_pub_->publish(std::move(message));
  }

  void publishLaneState(
    const rclcpp::Time & stamp, bool valid, double left_clearance,
    double right_clearance, double center_error, double heading_error,
    double road_width, int valid_rows)
  {
    const auto decision = safety::classifyLaneState(
      valid, left_clearance, right_clearance, center_error,
      lane_state_filter_->state(), lane_thresholds_);
    const std::string state = lane_state_filter_->update(decision.state);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_lane_state_ = state;
      latest_lane_valid_ = valid;
      latest_lane_critical_ = decision.critical;
      latest_center_error_ = center_error;
      latest_lane_time_ = now();
      have_lane_ = true;
    }
    const double confidence = std::clamp(static_cast<double>(valid_rows) / 12.0, 0.0, 1.0);
    std::ostringstream json;
    json << std::boolalpha << std::fixed << std::setprecision(4)
         << "{\"backend\":\"cpu\",\"valid\":" << valid
         << ",\"state\":\"" << state << "\",\"critical\":" << decision.critical
         << ",\"reason\":\"" << decision.reason << "\""
         << ",\"left_clearance_m\":" << left_clearance
         << ",\"right_clearance_m\":" << right_clearance
         << ",\"center_error_m\":" << center_error
         << ",\"heading_error_rad\":" << heading_error
         << ",\"road_width_m\":" << road_width
         << ",\"confidence\":" << confidence
         << ",\"valid_rows\":" << valid_rows
         << ",\"stamp_ns\":" << stamp.nanoseconds() << '}';
    std_msgs::msg::String message;
    message.data = json.str();
    lane_state_pub_->publish(message);

    std_msgs::msg::String metrics;
    metrics.data = json.str();
    lane_metrics_pub_->publish(std::move(metrics));
  }

  void publishConnected(bool connected)
  {
    std_msgs::msg::Bool message;
    message.data = connected;
    connected_pub_->publish(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    camera_connected_ = connected;
  }

  void publishHealth(
    bool healthy, const std::string & reason,
    double mean_luma = std::numeric_limits<double>::quiet_NaN(),
    double stddev_luma = std::numeric_limits<double>::quiet_NaN(),
    double extreme_fraction = std::numeric_limits<double>::quiet_NaN(),
    double mean_gradient = std::numeric_limits<double>::quiet_NaN())
  {
    std_msgs::msg::Bool state;
    state.data = healthy;
    health_pub_->publish(state);
    std::ostringstream json;
    json << std::boolalpha << std::fixed << std::setprecision(4)
         << "{\"backend\":\"cpu\",\"healthy\":" << healthy
         << ",\"reason\":\"" << reason << "\"";
    const auto field = [&json](const char * name, double value) {
      json << ",\"" << name << "\":";
      if (std::isfinite(value)) json << value; else json << "null";
    };
    field("mean_luma", mean_luma);
    field("stddev_luma", stddev_luma);
    field("extreme_fraction", extreme_fraction);
    field("mean_gradient", mean_gradient);
    json << '}';
    std_msgs::msg::String detail;
    detail.data = json.str();
    health_state_pub_->publish(detail);
    std::lock_guard<std::mutex> lock(state_mutex_);
    camera_healthy_ = healthy;
  }

  void publishEmergency(bool emergency)
  {
    std_msgs::msg::Bool message;
    message.data = emergency;
    emergency_pub_->publish(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    emergency_stop_ = emergency;
  }

  void inferenceTick()
  {
    if (!openCamera()) return;
    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty() || frame.type() != CV_8UC3) {
      ++capture_dropped_total_;
      capture_.release();
      publishConnected(false);
      publishHealth(false, "FRAME_READ_FAILED_OR_NOT_BGR8");
      publishEmergency(true);
      return;
    }
    if (flip_horizontal_) cv::flip(frame, frame, 1);
    const auto stamp = now();
    const auto started = std::chrono::steady_clock::now();
    try {
      prepareHomography(frame.cols, frame.rows);
      double mean_luma = 0.0;
      double image_stddev = 0.0;
      double extreme_fraction = 1.0;
      double mean_gradient = 0.0;
      const bool image_healthy = cameraHealthy(
        frame, mean_luma, image_stddev, extreme_fraction, mean_gradient);
      const cv::Mat input = letterbox(frame);
      std::vector<cv::Mat> outputs = forwardTorch(input);
      validateOutputs(outputs);
      auto detections = decodeDetections(outputs, frame.cols, frame.rows);
      cv::Mat drivable;
      cv::Mat lane;
      decodeMasks(outputs, frame.cols, frame.rows, drivable, lane);

      bool emergency = false;
      std::vector<std::pair<float, float>> obstacle_centers;
      std::vector<MetricObstacle> metric_obstacles;
      const auto object_points = projectObstacles(
        detections, drivable, emergency, obstacle_centers, metric_obstacles);
      bool lane_valid = false;
      double left_clearance = 0.0;
      double right_clearance = 0.0;
      double center_error = 0.0;
      double heading_error = 0.0;
      double road_width = 0.0;
      int valid_rows = 0;
      const auto boundary_points = buildDrivableBoundary(
        drivable, lane_valid, left_clearance, right_clearance, center_error,
        heading_error, road_width, valid_rows);

      object_pub_->publish(makeCloud(stamp, object_points));
      boundary_pub_->publish(makeCloud(stamp, boundary_points));
      publishClearingFan(stamp, obstacle_centers);
      publishObstacleMetrics(stamp, metric_obstacles);
      publishDrivableSpace(stamp, lane_valid, valid_rows, boundary_points.size());
      publishNearFieldState(emergency, metric_obstacles, bottomDrivableFraction(drivable));
      publishLaneState(
        stamp, lane_valid, left_clearance, right_clearance, center_error,
        heading_error, road_width, valid_rows);
      publishEmergency(emergency || !image_healthy);
      publishHealth(
        image_healthy, image_healthy ? "OK" : "IMAGE_DEGRADED",
        mean_luma, image_stddev, extreme_fraction, mean_gradient);
      publishConnected(true);
      publishDetectionMessages(stamp, detections);
      publishCameraInfo(stamp, frame.cols, frame.rows);

      if (publish_raw_) publishImage(raw_pub_, stamp, frame, "bgr8");
      if (publish_drivable_) publishImage(drivable_pub_, stamp, drivable, "mono8");
      if (publish_lane_) publishImage(lane_pub_, stamp, lane, "mono8");
      if (publish_annotated_ && annotated_pub_->get_subscription_count() > 0U) {
        cv::Mat annotated = frame.clone();
        cv::Mat green(frame.size(), CV_8UC3, cv::Scalar(0, 255, 0));
        cv::Mat red(frame.size(), CV_8UC3, cv::Scalar(0, 0, 255));
        green.copyTo(annotated, drivable);
        cv::addWeighted(frame, 1.0 - overlay_alpha_, annotated, overlay_alpha_, 0.0, annotated);
        red.copyTo(annotated, lane);
        for (const auto & d : detections) {
          cv::rectangle(
            annotated, cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1)),
            cv::Point(static_cast<int>(d.x2), static_cast<int>(d.y2)), cv::Scalar(0, 255, 255), box_thickness_);
        }
        publishImage(annotated_pub_, stamp, annotated, "bgr8");
        ++rviz_published_total_;
      }

      const auto finished = std::chrono::steady_clock::now();
      const double elapsed_ms = std::chrono::duration<double, std::milli>(finished - started).count();
      double instantaneous_fps = 1000.0 / std::max(1.0, elapsed_ms);
      if (last_inference_finished_.time_since_epoch().count() != 0) {
        const double completion_period =
          std::chrono::duration<double>(finished - last_inference_finished_).count();
        if (completion_period > 1.0e-6) instantaneous_fps = 1.0 / completion_period;
      }
      last_inference_finished_ = finished;
      performance_fps_ = performance_fps_ <= 0.0 ? instantaneous_fps :
        0.15 * instantaneous_fps + 0.85 * performance_fps_;
      timing_window_ms_.push_back(elapsed_ms);
      while (timing_window_ms_.size() > 120U) timing_window_ms_.pop_front();
      std::vector<double> timing(timing_window_ms_.begin(), timing_window_ms_.end());
      std::sort(timing.begin(), timing.end());
      const double mean_ms = std::accumulate(timing.begin(), timing.end(), 0.0) /
        static_cast<double>(std::max<size_t>(1U, timing.size()));
      const size_t p95_index = timing.empty() ? 0U :
        std::min(timing.size() - 1U, static_cast<size_t>(std::ceil(0.95 * timing.size()) - 1.0));
      const double p95_ms = timing.empty() ? elapsed_ms : timing[p95_index];
      const double min_ms = timing.empty() ? elapsed_ms : timing.front();
      const double max_ms = timing.empty() ? elapsed_ms : timing.back();
      double variance_ms = 0.0;
      for (const double value : timing) {
        const double delta = value - mean_ms;
        variance_ms += delta * delta;
      }
      const double std_ms = timing.empty() ? 0.0 :
        std::sqrt(variance_ms / static_cast<double>(timing.size()));

      std_msgs::msg::String performance;
      std::ostringstream json;
      json << std::fixed << std::setprecision(3)
           << "{\"backend\":\"cpu\",\"window_frames\":" << timing.size()
           << ",\"pipeline_fps\":" << performance_fps_
           << ",\"pipeline_fps_ema\":" << performance_fps_
           << ",\"pipeline_ms_per_frame\":" << mean_ms
           << ",\"pipeline_ms\":" << elapsed_ms
           << ",\"pipeline_p95_ms\":" << p95_ms
           << ",\"pipeline_min_ms\":" << min_ms
           << ",\"pipeline_max_ms\":" << max_ms
           << ",\"pipeline_std_ms\":" << std_ms
           << ",\"target_fps\":" << inference_fps_
           << ",\"cpu_threads\":" << cpu_threads_
           << ",\"capture_dropped_total\":" << capture_dropped_total_
           << ",\"rviz_published_total\":" << rviz_published_total_
           << ",\"rviz_dropped_total\":" << rviz_dropped_total_
           << ",\"raw_detection_count\":" << detections.size()
           << ",\"metric_candidate_count\":" << metric_obstacles.size()
           << ",\"confirmed_obstacle_count\":" << metric_obstacles.size()
           << ",\"object_points\":" << object_points.size()
           << ",\"camera_mean_luma\":" << mean_luma
           << ",\"camera_stddev\":" << image_stddev
           << ",\"camera_mean_gradient\":" << mean_gradient
           << ",\"extreme_fraction\":" << extreme_fraction
           << ",\"timing_scope\":\"inferenceTick_total\"}";
      performance.data = json.str();
      performance_pub_->publish(performance);
      if (elapsed_ms > 900.0) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 30000,
          "CPU inference %.1f ms > 0.9 s safety freshness target; perception tetap berjalan tetapi autonomous safety fail-closed. Jetson GPU direkomendasikan untuk motion authority.",
          elapsed_ms);
      }
    } catch (const cv::Exception & error) {
      publishHealth(false, "OPENCV_INFERENCE_ERROR");
      publishEmergency(true);
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "OpenCV CPU perception error: %s", error.what());
    } catch (const std::exception & error) {
      publishHealth(false, "CPU_PIPELINE_ERROR");
      publishEmergency(true);
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "CPU perception error: %s", error.what());
    }
  }

  void controlTick()
  {
    ++control_sequence_;
    geometry_msgs::msg::Twist nav;
    geometry_msgs::msg::Twist desired;
    geometry_msgs::msg::Twist output;
    std::string lane_state = safety::LANE_LOST;
    std::string decision = "WAIT_NAV_CMD";
    bool nav_fresh = false;
    bool lane_fresh = false;
    bool lane_valid = false;
    bool lane_critical = false;
    bool camera_connected = false;
    bool camera_healthy = false;
    bool emergency = true;
    double center_error = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      nav_fresh = have_nav_cmd_ && last_nav_cmd_time_.nanoseconds() > 0 &&
        (now() - last_nav_cmd_time_).seconds() >= 0.0 &&
        (now() - last_nav_cmd_time_).seconds() <= cmd_timeout_sec_;
      if (nav_fresh) nav = last_nav_cmd_;
      lane_fresh = have_lane_ && latest_lane_time_.nanoseconds() > 0 &&
        (now() - latest_lane_time_).seconds() >= 0.0 &&
        (now() - latest_lane_time_).seconds() <= lane_state_timeout_sec_;
      if (lane_fresh) {
        lane_state = latest_lane_state_;
        lane_valid = latest_lane_valid_;
        lane_critical = latest_lane_critical_;
        center_error = latest_center_error_;
      }
      camera_connected = camera_connected_;
      camera_healthy = camera_healthy_;
      emergency = emergency_stop_;
    }

    desired = nav;
    if (!nav_fresh) {
      decision = "NAV_CMD_STALE_STOP";
      desired = geometry_msgs::msg::Twist{};
    } else if (emergency) {
      decision = "EMERGENCY_STOP";
      desired = geometry_msgs::msg::Twist{};
    } else if (!camera_connected) {
      decision = "CAMERA_DISCONNECTED_STOP";
      desired = geometry_msgs::msg::Twist{};
    } else if (!camera_healthy) {
      decision = "CAMERA_UNHEALTHY_STOP";
      desired = geometry_msgs::msg::Twist{};
    } else if (!lane_safety_enabled_) {
      decision = "DISABLED_PASSTHROUGH";
    } else if (!camera_metric_calibration_validated_) {
      decision = "CALIBRATION_REQUIRED_PASSTHROUGH";
    } else if (!lane_fresh || !lane_valid || lane_state == safety::LANE_LOST) {
      decision = safety::LANE_LOST;
      desired = geometry_msgs::msg::Twist{};
    } else if (lane_state == safety::RECENTER_LEFT || lane_state == safety::RECENTER_RIGHT) {
      const auto mixed = safety::mixRecenterCommand(
        nav.linear.x, nav.angular.z, center_error, 0.0, lane_critical, mixer_config_);
      desired = geometry_msgs::msg::Twist{};
      desired.linear.x = mixed.linear_x;
      desired.angular.z = mixed.angular_z;
      decision = lane_state;
    } else {
      decision = safety::NORMAL;
    }

    const bool applied = lane_safety_enabled_ && camera_metric_calibration_validated_ &&
      control_mode_ == "active";
    output = applied ? desired : nav;
    advisory_pub_->publish(output);
    std_msgs::msg::String detail;
    std::ostringstream json;
    json << std::boolalpha << std::fixed << std::setprecision(5)
         << "{\"sequence\":" << control_sequence_
         << ",\"backend\":\"cpu\""
         << ",\"mode\":\"" << control_mode_
         << "\",\"enabled\":" << lane_safety_enabled_
         << ",\"metric_calibrated\":" << camera_metric_calibration_validated_
         << ",\"applied\":" << applied
         << ",\"decision\":\"" << decision
         << "\",\"lane_state\":\"" << lane_state
         << "\",\"lane_valid\":" << lane_valid
         << ",\"lane_fresh\":" << lane_fresh
         << ",\"nav_cmd_fresh\":" << nav_fresh
         << ",\"critical\":" << lane_critical
         << ",\"center_error_m\":" << center_error
         << ",\"raw_recenter_blocked\":false"
         << ",\"recenter_blocked\":false"
         << ",\"nav_cmd\":{\"linear_x\":" << nav.linear.x
         << ",\"angular_z\":" << nav.angular.z
         << "},\"calculated_cmd\":{\"linear_x\":" << desired.linear.x
         << ",\"angular_z\":" << desired.angular.z
         << "},\"output_cmd\":{\"linear_x\":" << output.linear.x
         << ",\"angular_z\":" << output.angular.z << "}}";
    detail.data = json.str();
    lane_control_state_pub_->publish(detail);
  }

  torch::jit::script::Module module_;
  cv::VideoCapture capture_;
  cv::Mat homography_;
  int homography_width_{0};
  int homography_height_{0};
  float gain_{1.0F};
  int pad_x_{0};
  int pad_y_{0};
  int resized_width_{MODEL_WIDTH};
  int resized_height_{MODEL_HEIGHT};

  std::string pt_model_path_;
  double inference_fps_{5.0};
  int cpu_threads_{4};
  std::string rgb_device_{"auto"};
  int requested_width_{1280};
  int requested_height_{720};
  int camera_fps_{30};
  std::string pixel_format_{"MJPEG"};
  bool strict_camera_mode_{false};
  bool flip_horizontal_{true};
  bool hotplug_retry_{true};
  double retry_sec_{2.0};
  float confidence_threshold_{0.10F};
  float iou_threshold_{0.45F};
  float lane_threshold_{0.50F};
  int max_candidates_{16384};
  int max_detections_{300};
  bool publish_annotated_{true};
  bool publish_raw_{true};
  bool publish_drivable_{false};
  bool publish_lane_{false};
  bool publish_detections_{false};
  double overlay_alpha_{0.45};
  int box_thickness_{2};
  std::string frame_id_;
  std::string metric_frame_id_;
  std::string annotated_topic_, raw_topic_, camera_info_topic_, detections_topic_;
  std::string drivable_topic_, lane_topic_, performance_topic_;
  std::string camera_connected_topic_, camera_health_topic_, camera_health_state_topic_;
  std::string emergency_topic_, object_points_topic_, clearing_points_topic_;
  std::string drivable_boundary_topic_, lane_state_topic_, lane_control_state_topic_;
  std::string lane_metrics_topic_, drivable_space_topic_, obstacle_metrics_topic_, near_field_state_topic_;
  std::string raw_detection_topic_, nav_cmd_topic_, safe_cmd_topic_;
  double control_rate_hz_{20.0};
  double cmd_timeout_sec_{0.50};
  double lane_state_timeout_sec_{0.50};
  bool camera_metric_calibration_validated_{false};
  bool lane_safety_enabled_{false};
  std::string control_mode_{"active"};
  safety::MixerConfig mixer_config_{};

  int ground_calibration_width_{1280};
  int ground_calibration_height_{720};
  std::vector<double> ground_src_, ground_dst_;
  int ground_canvas_width_{1280};
  int ground_canvas_height_{720};
  double ground_origin_x_{596.8667};
  double ground_origin_y_{719.0};
  double ground_scale_x_{0.0039093041};
  double ground_scale_y_{0.0097357441};
  double forward_offset_{0.0};
  double lateral_offset_{0.0};
  double min_forward_{0.20};
  double max_forward_{4.0};
  double max_abs_left_{2.5};
  double minimum_obstacle_confidence_{0.30};
  bool accept_all_obstacles_{false};
  std::vector<int64_t> safety_classes_;
  bool require_drivable_contact_{true};
  int drivable_contact_min_samples_{3};
  double drivable_contact_min_fraction_{0.20};
  int points_per_box_{5};
  int clearing_ray_count_{41};
  double clearing_fov_rad_{1.7453};
  double clearing_range_m_{4.5};
  double clearing_obstacle_margin_m_{0.20};
  double emergency_distance_m_{0.65};
  double emergency_half_width_m_{0.55};
  double lane_vehicle_width_m_{0.55};
  safety::LaneThresholds lane_thresholds_;
  std::unique_ptr<safety::ConfirmedState> lane_state_filter_;
  int health_dark_{18};
  int health_bright_{245};
  double health_extreme_fraction_{0.97};
  double health_min_stddev_{5.0};
  double health_min_gradient_{2.0};
  double camera_fx_{910.0}, camera_fy_{910.0}, camera_cx_{640.0}, camera_cy_{360.0};
  std::vector<double> camera_distortion_;

  std::mutex state_mutex_;
  geometry_msgs::msg::Twist last_nav_cmd_;
  rclcpp::Time last_nav_cmd_time_{0, 0, RCL_ROS_TIME};
  bool have_nav_cmd_{false};
  bool camera_connected_{false};
  bool camera_healthy_{false};
  bool emergency_stop_{true};
  bool latest_lane_valid_{false};
  bool latest_lane_critical_{false};
  std::string latest_lane_state_{safety::LANE_LOST};
  double latest_center_error_{0.0};
  rclcpp::Time latest_lane_time_{0, 0, RCL_ROS_TIME};
  bool have_lane_{false};
  uint64_t control_sequence_{0U};
  double performance_fps_{0.0};
  uint64_t capture_dropped_total_{0U};
  uint64_t rviz_published_total_{0U};
  uint64_t rviz_dropped_total_{0U};
  std::deque<double> timing_window_ms_;
  std::chrono::steady_clock::time_point last_inference_finished_{};
  std::chrono::steady_clock::time_point last_open_attempt_{};

  rclcpp::TimerBase::SharedPtr inference_timer_, control_timer_;
  rclcpp::CallbackGroup::SharedPtr inference_group_, control_group_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_, raw_pub_, drivable_pub_, lane_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr performance_pub_, health_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lane_state_pub_, lane_control_state_pub_, raw_detection_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lane_metrics_pub_, drivable_space_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_metrics_pub_, near_field_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_, health_pub_, emergency_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pub_, clearing_pub_, boundary_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr advisory_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
};

}  // namespace perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<perception::AstraYolopCpuNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "perception_cpu_node fatal: %s\n", error.what());
    exit_code = 2;
  }
  rclcpp::shutdown();
  return exit_code;
}
