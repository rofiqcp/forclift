#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "yolo_obstacle_detection_ros2/msg/obstacle.hpp"
#include "yolo_obstacle_detection_ros2/msg/obstacle_array.hpp"
#include "yolo_obstacle_detection_ros2/msg/pallet_pose.hpp"
#include "yolo_obstacle_detection_ros2/msg/floor_marking.hpp"

namespace yolo_obstacle_detection_ros2
{
namespace
{
constexpr double kHalfPi = 1.57079632679489661923;

struct Track
{
  double x{0.0};
  double y{0.0};
  int age{0};
  int frames{0};
};

struct TrackResult
{
  int id{-1};
  int frames{0};
};

class SpatialTracker
{
public:
  SpatialTracker(double max_distance, int max_age)
  : max_distance_(max_distance), max_age_(max_age) {}

  std::vector<TrackResult> update(const std::vector<std::pair<double, double>> & detections)
  {
    for (auto & kv : tracks_) {
      kv.second.age++;
    }
    for (auto it = tracks_.begin(); it != tracks_.end();) {
      if (it->second.age > max_age_) {
        it = tracks_.erase(it);
      } else {
        ++it;
      }
    }

    std::vector<TrackResult> out;
    out.reserve(detections.size());
    std::unordered_map<int, bool> used;

    for (const auto & det : detections) {
      int best_id = -1;
      double best_dist = max_distance_;
      for (const auto & kv : tracks_) {
        if (used[kv.first]) continue;
        const double dx = det.first - kv.second.x;
        const double dy = det.second - kv.second.y;
        const double d = std::hypot(dx, dy);
        if (d < best_dist) {
          best_dist = d;
          best_id = kv.first;
        }
      }

      if (best_id < 0) {
        best_id = next_id_++;
        tracks_[best_id] = Track{det.first, det.second, 0, 1};
      } else {
        auto & tr = tracks_[best_id];
        tr.x = det.first;
        tr.y = det.second;
        tr.age = 0;
        tr.frames++;
      }
      used[best_id] = true;
      out.push_back({best_id, tracks_[best_id].frames});
    }
    return out;
  }

private:
  double max_distance_{0.5};
  int max_age_{10};
  int next_id_{0};
  std::unordered_map<int, Track> tracks_;
};

struct Position3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double quality{0.0};
};
}  // namespace

class PalletPoseEstimator final : public rclcpp::Node
{
public:
  PalletPoseEstimator()
  : Node("pallet_pose_estimator"), pallet_tracker_(0.5, 10)
  {
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    obstacles_topic_ = declare_parameter<std::string>("obstacles_topic", "/obstacle_detection/obstacles");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    camera_height_ = declare_parameter<double>("camera_height", 0.50);
    camera_tilt_ = declare_parameter<double>("camera_tilt", 0.35);
    min_depth_ = declare_parameter<double>("min_depth", 0.30);
    max_depth_ = declare_parameter<double>("max_depth", 5.0);
    pallet_depth_ = declare_parameter<double>("pallet_depth", 0.80);
    hole_width_ = declare_parameter<double>("hole_width", 0.19);
    hole_height_ = declare_parameter<double>("hole_height", 0.10);
    hole_spacing_ = declare_parameter<double>("hole_spacing", 0.60);
    entry_offset_ = declare_parameter<double>("entry_offset", 0.30);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, sensor_qos,
      std::bind(&PalletPoseEstimator::onDepth, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos,
      std::bind(&PalletPoseEstimator::onInfo, this, std::placeholders::_1));
    obstacle_sub_ = create_subscription<msg::ObstacleArray>(
      obstacles_topic_, rclcpp::QoS(10),
      std::bind(&PalletPoseEstimator::onObstacles, this, std::placeholders::_1));

    pallet_pub_ = create_publisher<msg::PalletPose>("/pallet_detection/poses", 10);
    floor_pub_ = create_publisher<msg::FloorMarking>("/floor_marking/poses", 10);

    RCLCPP_INFO(get_logger(), "C++ pallet pose estimator ready: %s + %s",
      depth_topic_.c_str(), obstacles_topic_.c_str());
  }

private:
  void onDepth(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      if (msg->encoding == "16UC1" || msg->encoding == "mono16") {
        depth_image_ = cv_bridge::toCvCopy(msg, msg->encoding)->image.clone();
        depth_scale_ = 0.001;
      } else if (msg->encoding == "32FC1") {
        depth_image_ = cv_bridge::toCvCopy(msg, msg->encoding)->image.clone();
        depth_scale_ = 1.0;
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Unsupported depth encoding: %s", msg->encoding.c_str());
        return;
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Depth conversion failed: %s", e.what());
    }
  }

  void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    camera_info_ = msg;
  }

  bool readDepthMeters(int u, int v, double & depth_m, double & quality) const
  {
    if (depth_image_.empty()) return false;
    const int half_w = 8;
    const int half_h = 8;
    const int x1 = std::max(0, u - half_w);
    const int x2 = std::min(depth_image_.cols - 1, u + half_w);
    const int y1 = std::max(0, v - half_h);
    const int y2 = std::min(depth_image_.rows - 1, v + half_h);
    if (x1 > x2 || y1 > y2) return false;

    std::vector<double> values;
    values.reserve(static_cast<size_t>((x2 - x1 + 1) * (y2 - y1 + 1)));
    for (int y = y1; y <= y2; ++y) {
      for (int x = x1; x <= x2; ++x) {
        double d = 0.0;
        if (depth_image_.type() == CV_16UC1) {
          d = static_cast<double>(depth_image_.at<uint16_t>(y, x)) * depth_scale_;
        } else if (depth_image_.type() == CV_32FC1) {
          d = static_cast<double>(depth_image_.at<float>(y, x)) * depth_scale_;
        } else {
          return false;
        }
        if (std::isfinite(d) && d >= min_depth_ && d <= max_depth_) values.push_back(d);
      }
    }
    if (values.size() < 5) return false;
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    depth_m = *middle;
    quality = std::min(1.0, static_cast<double>(values.size()) / 50.0);
    return true;
  }

  bool estimatePosition(const msg::Obstacle & obs, Position3D & out) const
  {
    if (!camera_info_) return false;
    double depth = 0.0;
    double quality = 0.0;
    const int u = static_cast<int>(std::lround(obs.center_x));
    const int v = static_cast<int>(std::lround(obs.center_y));
    if (!readDepthMeters(u, v, depth, quality)) return false;

    const double fx = camera_info_->k[0];
    const double fy = camera_info_->k[4];
    const double cx = camera_info_->k[2];
    const double cy = camera_info_->k[5];
    if (fx <= 1e-6 || fy <= 1e-6) return false;

    const double x_cam = (static_cast<double>(u) - cx) * depth / fx;
    const double y_cam = (static_cast<double>(v) - cy) * depth / fy;
    const double z_cam = depth;

    // Same geometry as the user's original Python estimator. The final frame is
    // base_footprint, whose model subtree is REP-103 aligned in navigation URDF.
    out.x = z_cam * std::cos(camera_tilt_) + y_cam * std::sin(camera_tilt_);
    out.y = -x_cam;
    out.z = camera_height_ - z_cam * std::sin(camera_tilt_) + y_cam * std::cos(camera_tilt_);
    out.quality = quality;
    return std::isfinite(out.x) && std::isfinite(out.y) && std::isfinite(out.z);
  }

  void onObstacles(const msg::ObstacleArray::SharedPtr obstacles)
  {
    if (!camera_info_ || depth_image_.empty()) return;

    std::vector<const msg::Obstacle *> pallet_obs;
    std::vector<Position3D> pallet_positions;
    std::vector<std::pair<double, double>> track_inputs;

    for (const auto & obs : obstacles->obstacles) {
      if (obs.class_name == "pallet" || obs.class_name == "hole pallet") {
        Position3D p;
        if (estimatePosition(obs, p)) {
          pallet_obs.push_back(&obs);
          pallet_positions.push_back(p);
          track_inputs.emplace_back(p.x, p.y);
        }
      }
    }

    const auto tracks = pallet_tracker_.update(track_inputs);
    for (size_t i = 0; i < pallet_obs.size() && i < tracks.size(); ++i) {
      publishPallet(*obstacles, *pallet_obs[i], pallet_positions[i], tracks[i]);
    }

    for (const auto & obs : obstacles->obstacles) {
      if (obs.class_name == "floor marking") publishFloor(*obstacles, obs);
    }
  }

  void publishPallet(
    const msg::ObstacleArray & array, const msg::Obstacle & obs,
    const Position3D & p, const TrackResult & track)
  {
    msg::PalletPose out;
    out.header = array.header;
    out.header.frame_id = base_frame_;
    out.pallet_id = track.id;
    out.class_name = obs.class_name;
    out.confidence = obs.confidence;

    const double width = std::max(1.0F, obs.x2 - obs.x1);
    const double height = std::max(1.0F, obs.y2 - obs.y1);
    const double aspect = width / height;
    double yaw = 0.0;
    if (aspect < 0.7) yaw = kHalfPi;

    out.pallet_pose.x = static_cast<float>(p.x);
    out.pallet_pose.y = static_cast<float>(p.y);
    out.pallet_pose.z = static_cast<float>(p.z);
    out.pallet_pose.yaw = static_cast<float>(yaw);
    out.pallet_pose.yaw_confidence = 0.30F;
    out.pallet_pose.yaw_valid = false;

    out.entry_pose.x = static_cast<float>(p.x - entry_offset_ * std::cos(yaw));
    out.entry_pose.y = static_cast<float>(p.y - entry_offset_ * std::sin(yaw));
    out.entry_pose.z = 0.0F;
    out.entry_pose.yaw = static_cast<float>(yaw);
    out.entry_pose.yaw_confidence = 0.30F;
    out.entry_pose.yaw_valid = false;

    out.left_hole.x = p.x;
    out.left_hole.y = p.y + hole_spacing_ * 0.5;
    out.left_hole.z = 0.16;
    out.right_hole.x = p.x;
    out.right_hole.y = p.y - hole_spacing_ * 0.5;
    out.right_hole.z = 0.16;
    out.hole_width = static_cast<float>(hole_width_);
    out.hole_height = static_cast<float>(hole_height_);
    out.hole_depth = static_cast<float>(pallet_depth_);
    out.depth_quality = static_cast<float>(p.quality);
    out.frames_tracked = track.frames;
    out.tracking_stable = obs.confidence > 0.7F && p.quality > 0.5 && track.frames > 3;
    pallet_pub_->publish(out);
  }

  void publishFloor(const msg::ObstacleArray & array, const msg::Obstacle & obs)
  {
    Position3D p;
    if (!estimatePosition(obs, p)) return;
    msg::FloorMarking out;
    out.header = array.header;
    out.header.frame_id = base_frame_;
    out.marking_id = obs.class_id;
    out.marking_type = obs.class_name;
    out.confidence = obs.confidence;
    const double width = std::max(1.0F, obs.x2 - obs.x1);
    const double height = std::max(1.0F, obs.y2 - obs.y1);
    const double theta = width >= height ? 0.0 : kHalfPi;
    out.marking_pose.x = p.x;
    out.marking_pose.y = p.y;
    out.marking_pose.theta = theta;
    const double half = 0.25;
    out.line_start.x = p.x - half * std::cos(theta);
    out.line_start.y = p.y - half * std::sin(theta);
    out.line_start.z = 0.01;
    out.line_end.x = p.x + half * std::cos(theta);
    out.line_end.y = p.y + half * std::sin(theta);
    out.line_end.z = 0.01;
    out.tracking_stable = obs.confidence > 0.6F;
    out.frames_tracked = 1;
    floor_pub_->publish(out);
  }

  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string obstacles_topic_;
  std::string base_frame_;
  double camera_height_{0.5};
  double camera_tilt_{0.35};
  double min_depth_{0.3};
  double max_depth_{5.0};
  double pallet_depth_{0.8};
  double hole_width_{0.19};
  double hole_height_{0.10};
  double hole_spacing_{0.60};
  double entry_offset_{0.30};
  double depth_scale_{0.001};
  cv::Mat depth_image_;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  SpatialTracker pallet_tracker_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<msg::PalletPose>::SharedPtr pallet_pub_;
  rclcpp::Publisher<msg::FloorMarking>::SharedPtr floor_pub_;
};
}  // namespace yolo_obstacle_detection_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<yolo_obstacle_detection_ros2::PalletPoseEstimator>());
  rclcpp::shutdown();
  return 0;
}
