#pragma once
// Astra RGB V4L2 Camera Driver for ROS 2 Humble
// Provides /camera/color/image_raw + /camera/color/camera_info via Linux V4L2 API.
// No external astra_camera_ros2 dependency.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

// V4L2
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>

// GStreamer GPU decode (NVIDIA nvv4l2decoder)
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <glib.h>

namespace astra_rgb_driver
{

// ─── Device Info ───────────────────────────────────────────────────────────────

struct V4L2DeviceInfo {
  std::string device_path;      // e.g. /dev/video2
  std::string by_id;           // e.g. /dev/v4l/by-id/...Astra...
  std::string by_path;         // e.g. /dev/v4l/by-path/...usb...
  std::string card;            // V4L2 card name
  std::string bus_info;        // V4L2 bus_info (USB bus location)
  std::string driver;          // uvcvideo, gspca, etc.
  uint16_t vid{0};           // from sysfs USB PRODUCT
  uint16_t pid{0};           // from sysfs USB PRODUCT
  std::string manufacturer;
  std::string product;
  std::string serial;
  uint32_t capabilities{0};
  int score{0};                // higher = more likely an Astra

  // Canonical path computed from actual device (resolves by-id/by-path symlinks)
  std::string canonical_path() const;
};

// ─── Camera Config ────────────────────────────────────────────────────────────

struct CameraConfig {
  std::string device{"auto"};      // "auto" or specific /dev/videoX path
  int width{1280};
  int height{720};
  int fps{30};
  std::string pixel_format{"MJPG"};  // MJPG preferred, YUYV fallback
  std::string frame_id{"camera_color_optical_frame"};
  std::string calibration_url{""};   // path to .yaml from camera_calibration
  int reconnect_interval_ms{1000};
  int frame_timeout_ms{3000};
  bool enable_depth{false};         // RGB-only for now; depth is OpenNI/Astra depth
};

// ─── Internal State ───────────────────────────────────────────────────────────

enum class StreamState { CLOSED, OPENING, STREAMING };

struct V4L2FrameBuffer {
  void * start{nullptr};
  size_t length{0};
  int index{-1};
};

// ─── Main Node ───────────────────────────────────────────────────────────────

class AstraRGBNode : public rclcpp::Node
{
public:
  explicit AstraRGBNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~AstraRGBNode() override;

private:
  // Discovery
  std::vector<V4L2DeviceInfo> discover_devices();
  bool score_device(const V4L2DeviceInfo & info, int * out_score) const;
  std::string resolve_device(const std::string & hint) const;
  bool test_device_streaming(const std::string & path, int width, int height,
                              const std::string & format, int fps);

  // Connection lifecycle
  bool open_device(const V4L2DeviceInfo & dev);
  bool configure_format();
  bool reqbuf_and_queue();
  bool stream_on();
  void stream_off_and_unmap();

  // Read loop
  void read_loop();

  // Reconnect
  void schedule_reconnect();
  void reconnect_loop();

  // ROS publish
  void publish_frame(const uint8_t * data, size_t size,
                    const rclcpp::Time & stamp);
  void publish_status(const std::string & text);

  // GStreamer GPU decode (NVIDIA nvv4l2decoder)
  bool open_gst_pipeline(const std::string & device);
  void close_gst_pipeline();
  void gst_read_loop();
  void publish_gst_frame(GstSample * sample);

  // Utility
  std::optional<std::string> get_usb_vid_pid(const std::string & dev_path);
  bool try_format(int fd, uint32_t pixel_format, int width, int height,
                  int * actual_width, int * actual_height,
                  uint32_t * actual_format);

  // ── Parameters ──────────────────────────────────────────────────────────────
  CameraConfig config_;

  // ── ROS ─────────────────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // ── V4L2 ───────────────────────────────────────────────────────────────────
  int fd_{-1};
  int lock_fd_{-1};
  std::vector<V4L2FrameBuffer> buffers_;
  std::string active_device_path_;
  std::string active_by_id_;
  std::string active_by_path_;
  std::atomic<StreamState> stream_state_{StreamState::CLOSED};

  // ── Thread ─────────────────────────────────────────────────────────────────
  std::thread reader_thread_;
  std::thread reconnect_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> reconnect_requested_{false};
  std::mutex stream_mutex_;  // serialize V4L2 teardown/configure/read against reconnect

  // ── Stats ───────────────────────────────────────────────────────────────────
  std::atomic<uint64_t> frames_received_{0};
  std::atomic<uint64_t> frames_published_{0};
  std::atomic<uint64_t> errors_{0};
  std::atomic<uint64_t> reconnect_count_{0};
  // Keep timer state as a node member. ROS 2 Humble's rclcpp::function_traits
  // does not accept a mutable lambda callback for create_wall_timer().
  std::atomic<uint64_t> last_stats_published_{0};
  std::chrono::steady_clock::time_point last_frame_wall_;
  std::chrono::steady_clock::time_point last_stats_wall_;

  // ── CameraInfo ──────────────────────────────────────────────────────────────
  sensor_msgs::msg::CameraInfo camera_info_;
  void build_camera_info(int width, int height);

  // ── GStreamer GPU decode ────────────────────────────────────────────────────
  std::atomic<bool> gst_running_{false};
  std::thread gst_thread_;
  // gst_pipeline_ is a GstElement* stored as void* for simpler Opaque declaration.
  void * gst_pipeline_{nullptr};
  std::atomic<bool> use_gpu_decode_{false};
  std::atomic<bool> require_gpu_decode_{false};
};

}  // namespace astra_rgb_driver
