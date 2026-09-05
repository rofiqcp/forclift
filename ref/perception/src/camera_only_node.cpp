/*
 * Camera-only backend for ROS 2 Humble.
 *
 * Used by perception_mode:=off.  It intentionally does not load any ML model
 * and does not publish perception safety authority.  It keeps the physical
 * camera, GUI preview, camera-info, connection and health telemetry alive.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace perception
{

namespace
{

rclcpp::QoS stateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

}  // namespace

class CameraOnlyNode final : public rclcpp::Node
{
public:
  CameraOnlyNode()
  : Node("perception_camera")
  {
    declareParameters();
    readParameters();
    createInterfaces();
    publishState(false, false, "STARTUP");

    const auto period = std::chrono::duration<double>(1.0 / std::max(1, publish_fps_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CameraOnlyNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "camera-only aktif: device=%s requested=%dx%d@%d model=OFF",
      rgb_device_.c_str(), width_, height_, publish_fps_);
  }

  ~CameraOnlyNode() override
  {
    if (capture_.isOpened()) {
      capture_.release();
    }
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("rgb_device", "auto");
    declare_parameter<int>("rgb_width", 1280);
    declare_parameter<int>("rgb_height", 720);
    declare_parameter<int>("fps", 30);
    declare_parameter<std::string>("v4l2_pixel_format", "MJPEG");
    declare_parameter<bool>("strict_camera_mode", false);
    declare_parameter<bool>("flip_horizontal", true);
    declare_parameter<bool>("camera_hotplug_retry", true);
    declare_parameter<double>("camera_retry_interval_sec", 2.0);
    declare_parameter<bool>("publish_raw_rgb", true);
    declare_parameter<bool>("publish_annotated", true);
    declare_parameter<std::string>("frame_id", "camera_color_optical_frame");
    declare_parameter<std::string>("raw_topic", "/camera/astra/image_raw");
    declare_parameter<std::string>("annotated_topic", "/camera/yolop/image_annotated");
    declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    declare_parameter<std::string>("camera_connected_topic", "/perception/camera_connected");
    declare_parameter<std::string>("camera_health_topic", "/perception/camera_healthy");
    declare_parameter<std::string>("camera_health_state_topic", "/perception/camera_health_state");
    declare_parameter<int>("camera_health_dark_luma", 18);
    declare_parameter<int>("camera_health_bright_luma", 245);
    declare_parameter<double>("camera_health_extreme_fraction", 0.97);
    declare_parameter<double>("camera_health_min_stddev", 5.0);
    declare_parameter<double>("camera_fx", 910.0);
    declare_parameter<double>("camera_fy", 910.0);
    declare_parameter<double>("camera_cx", 640.0);
    declare_parameter<double>("camera_cy", 360.0);
    declare_parameter<std::vector<double>>(
      "camera_distortion", {0.0, 0.0, 0.0, 0.0, 0.0});
  }

  void readParameters()
  {
    rgb_device_ = get_parameter("rgb_device").as_string();
    width_ = std::max(320, static_cast<int>(get_parameter("rgb_width").as_int()));
    height_ = std::max(240, static_cast<int>(get_parameter("rgb_height").as_int()));
    publish_fps_ = std::clamp(static_cast<int>(get_parameter("fps").as_int()), 1, 60);
    pixel_format_ = get_parameter("v4l2_pixel_format").as_string();
    strict_mode_ = get_parameter("strict_camera_mode").as_bool();
    flip_horizontal_ = get_parameter("flip_horizontal").as_bool();
    hotplug_retry_ = get_parameter("camera_hotplug_retry").as_bool();
    retry_sec_ = std::clamp(get_parameter("camera_retry_interval_sec").as_double(), 0.5, 30.0);
    publish_raw_ = get_parameter("publish_raw_rgb").as_bool();
    publish_annotated_ = get_parameter("publish_annotated").as_bool();
    frame_id_ = get_parameter("frame_id").as_string();
    raw_topic_ = get_parameter("raw_topic").as_string();
    annotated_topic_ = get_parameter("annotated_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    connected_topic_ = get_parameter("camera_connected_topic").as_string();
    healthy_topic_ = get_parameter("camera_health_topic").as_string();
    health_state_topic_ = get_parameter("camera_health_state_topic").as_string();
    dark_luma_ = std::clamp(static_cast<int>(get_parameter("camera_health_dark_luma").as_int()), 0, 255);
    bright_luma_ = std::clamp(static_cast<int>(get_parameter("camera_health_bright_luma").as_int()), 0, 255);
    extreme_fraction_ = std::clamp(get_parameter("camera_health_extreme_fraction").as_double(), 0.1, 1.0);
    min_stddev_ = std::max(0.0, get_parameter("camera_health_min_stddev").as_double());
    fx_ = get_parameter("camera_fx").as_double();
    fy_ = get_parameter("camera_fy").as_double();
    cx_ = get_parameter("camera_cx").as_double();
    cy_ = get_parameter("camera_cy").as_double();
    distortion_ = get_parameter("camera_distortion").as_double_array();
  }

  void createInterfaces()
  {
    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    raw_pub_ = create_publisher<sensor_msgs::msg::Image>(raw_topic_, image_qos);
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic_, image_qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, image_qos);
    connected_pub_ = create_publisher<std_msgs::msg::Bool>(connected_topic_, stateQos());
    healthy_pub_ = create_publisher<std_msgs::msg::Bool>(healthy_topic_, stateQos());
    health_state_pub_ = create_publisher<std_msgs::msg::String>(health_state_topic_, stateQos());
  }

  std::vector<std::string> candidateDevices() const
  {
    if (rgb_device_ != "auto") {
      return {rgb_device_};
    }

    // Astra/depth cameras commonly expose several /dev/video* nodes. Prefer
    // persistent V4L2 by-id links (especially index0/color-capable endpoints)
    // before probing numeric device nodes, so tty/video enumeration changes do
    // not silently select a depth/metadata stream after reboot.
    std::vector<std::string> devices;
    std::error_code error;
    const fs::path by_id("/dev/v4l/by-id");
    if (fs::is_directory(by_id, error)) {
      for (const auto & entry : fs::directory_iterator(by_id, error)) {
        if (error) break;
        const std::string name = entry.path().filename().string();
        if (name.find("index0") != std::string::npos || name.find("video") != std::string::npos) {
          devices.push_back(entry.path().string());
        }
      }
    }
    std::sort(devices.begin(), devices.end());

    for (int index = 0; index < 64; ++index) {
      const fs::path path = fs::path("/dev") / ("video" + std::to_string(index));
      std::error_code exists_error;
      if (fs::exists(path, exists_error) && !exists_error) devices.push_back(path.string());
    }

    std::vector<std::string> unique;
    for (const auto & device : devices) {
      if (std::find(unique.begin(), unique.end(), device) == unique.end()) unique.push_back(device);
    }
    return unique;
  }

  bool configureCapture(cv::VideoCapture & capture, const std::string & device)
  {
    bool opened = false;
    if (!device.empty() && std::all_of(device.begin(), device.end(), [](unsigned char c) {
        return std::isdigit(c) != 0;
      })) {
      opened = capture.open(std::stoi(device), cv::CAP_V4L2);
    } else {
      opened = capture.open(device, cv::CAP_V4L2);
    }
    if (!opened) return false;

    if (pixel_format_ == "MJPEG") {
      capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    } else if (pixel_format_ == "YUYV") {
      capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    }
    capture.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    capture.set(cv::CAP_PROP_FPS, publish_fps_);
    capture.set(cv::CAP_PROP_BUFFERSIZE, 1);

    cv::Mat probe;
    if (!capture.read(probe) || probe.empty() || probe.type() != CV_8UC3) {
      // All downstream publishing/health code is intentionally BGR8. Reject
      // depth (typically 16UC1), IR/mono, and metadata nodes here instead of
      // mislabelling/copying them as bgr8.
      capture.release();
      return false;
    }
    if (strict_mode_ && (probe.cols != width_ || probe.rows != height_)) {
      RCLCPP_ERROR(
        get_logger(), "kamera %s menghasilkan %dx%d, diminta strict %dx%d",
        device.c_str(), probe.cols, probe.rows, width_, height_);
      capture.release();
      return false;
    }
    selected_device_ = device;
    return true;
  }

  bool openCamera()
  {
    if (capture_.isOpened()) return true;
    const auto now_steady = std::chrono::steady_clock::now();
    if (last_open_attempt_.time_since_epoch().count() != 0) {
      const double elapsed = std::chrono::duration<double>(now_steady - last_open_attempt_).count();
      if (elapsed < retry_sec_) return false;
    }
    last_open_attempt_ = now_steady;

    for (const auto & device : candidateDevices()) {
      cv::VideoCapture candidate;
      if (configureCapture(candidate, device)) {
        capture_ = std::move(candidate);
        RCLCPP_INFO(get_logger(), "kamera aktif: %s", selected_device_.c_str());
        publishState(true, true, "OK_CAMERA_ONLY");
        return true;
      }
    }
    publishState(false, false, "CAMERA_NOT_FOUND");
    if (!hotplug_retry_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "kamera tidak ditemukan dan retry disabled");
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "kamera belum ditemukan; menunggu hot-plug");
    }
    return false;
  }

  bool frameHealthy(const cv::Mat & frame, double & stddev_value, double & extreme) const
  {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(gray, mean, stddev);
    stddev_value = stddev[0];
    const int total = std::max(1, gray.rows * gray.cols);
    int extremes = 0;
    for (int row = 0; row < gray.rows; ++row) {
      const auto * ptr = gray.ptr<uint8_t>(row);
      for (int col = 0; col < gray.cols; ++col) {
        if (ptr[col] <= dark_luma_ || ptr[col] >= bright_luma_) ++extremes;
      }
    }
    extreme = static_cast<double>(extremes) / total;
    return stddev_value >= min_stddev_ && extreme <= extreme_fraction_;
  }

  sensor_msgs::msg::Image makeImage(const rclcpp::Time & stamp, const cv::Mat & frame) const
  {
    sensor_msgs::msg::Image message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.height = static_cast<uint32_t>(frame.rows);
    message.width = static_cast<uint32_t>(frame.cols);
    message.encoding = "bgr8";
    message.is_bigendian = false;
    message.step = static_cast<uint32_t>(frame.cols * 3);
    const size_t bytes = static_cast<size_t>(message.step) * message.height;
    message.data.resize(bytes);
    if (frame.isContinuous()) {
      std::memcpy(message.data.data(), frame.data, bytes);
    } else {
      for (int row = 0; row < frame.rows; ++row) {
        std::memcpy(
          message.data.data() + static_cast<size_t>(row) * message.step,
          frame.ptr(row), message.step);
      }
    }
    return message;
  }

  void publishCameraInfo(const rclcpp::Time & stamp, int width, int height)
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame_id_;
    info.width = static_cast<uint32_t>(width);
    info.height = static_cast<uint32_t>(height);
    info.distortion_model = "plumb_bob";
    info.d = distortion_;
    info.k = {fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0};
    info.p = {fx_, 0.0, cx_, 0.0, 0.0, fy_, cy_, 0.0, 0.0, 0.0, 1.0, 0.0};
    camera_info_pub_->publish(info);
  }

  void publishState(bool connected, bool healthy, const std::string & reason)
  {
    std_msgs::msg::Bool connected_msg;
    connected_msg.data = connected;
    connected_pub_->publish(connected_msg);
    std_msgs::msg::Bool healthy_msg;
    healthy_msg.data = healthy;
    healthy_pub_->publish(healthy_msg);
    std_msgs::msg::String state;
    state.data = "backend=camera_only;model=off;connected=" +
      std::string(connected ? "true" : "false") + ";healthy=" +
      std::string(healthy ? "true" : "false") + ";reason=" + reason;
    health_state_pub_->publish(state);
  }

  void tick()
  {
    if (!openCamera()) return;
    cv::Mat frame;
    if (!capture_.read(frame) || frame.empty() || frame.type() != CV_8UC3) {
      capture_.release();
      publishState(false, false, "FRAME_READ_FAILED_OR_NOT_BGR8");
      return;
    }
    if (flip_horizontal_) cv::flip(frame, frame, 1);
    const auto stamp = now();
    double stddev_value = 0.0;
    double extreme = 1.0;
    const bool healthy = frameHealthy(frame, stddev_value, extreme);
    publishState(true, healthy, healthy ? "OK_CAMERA_ONLY" : "IMAGE_DEGRADED");

    const auto image = makeImage(stamp, frame);
    if (publish_raw_ && raw_pub_->get_subscription_count() > 0U) raw_pub_->publish(image);
    if (publish_annotated_ && annotated_pub_->get_subscription_count() > 0U) annotated_pub_->publish(image);
    if (camera_info_pub_->get_subscription_count() > 0U) publishCameraInfo(stamp, frame.cols, frame.rows);

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "camera-only %dx%d stddev=%.2f extreme=%.3f", frame.cols, frame.rows, stddev_value, extreme);
  }

  std::string rgb_device_;
  std::string pixel_format_;
  std::string frame_id_;
  std::string raw_topic_;
  std::string annotated_topic_;
  std::string camera_info_topic_;
  std::string connected_topic_;
  std::string healthy_topic_;
  std::string health_state_topic_;
  std::string selected_device_;
  int width_{1280};
  int height_{720};
  int publish_fps_{30};
  bool strict_mode_{false};
  bool flip_horizontal_{true};
  bool hotplug_retry_{true};
  bool publish_raw_{true};
  bool publish_annotated_{true};
  double retry_sec_{2.0};
  int dark_luma_{18};
  int bright_luma_{245};
  double extreme_fraction_{0.97};
  double min_stddev_{5.0};
  double fx_{910.0};
  double fy_{910.0};
  double cx_{640.0};
  double cy_{360.0};
  std::vector<double> distortion_;
  cv::VideoCapture capture_;
  std::chrono::steady_clock::time_point last_open_attempt_{};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr healthy_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_state_pub_;
};

}  // namespace perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception::CameraOnlyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
