#include "yolo_obstacle_detection_ros2/astra_rgb_driver.hpp"

#include <opencv2/opencv.hpp>
#include <cmath>
#include <cctype>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <glob.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/stat.h>

// GStreamer GPU decode (NVIDIA nvv4l2decoder)
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <glib.h>

namespace astra_rgb_driver
{

// ─── Pixel format registry ────────────────────────────────────────────────────

static uint32_t fourcc_from_str(const char s[5])
{
  return (static_cast<uint32_t>(s[0]) |
          (static_cast<uint32_t>(s[1]) << 8) |
          (static_cast<uint32_t>(s[2]) << 16) |
          (static_cast<uint32_t>(s[3]) << 24));
}

static const char * fourcc_to_str(uint32_t f)
{
  static char buf[5] = {0};
  buf[0] = static_cast<char>(f & 0xFF);
  buf[1] = static_cast<char>((f >> 8) & 0xFF);
  buf[2] = static_cast<char>((f >> 16) & 0xFF);
  buf[3] = static_cast<char>((f >> 24) & 0xFF);
  return buf;
}

// ─── Device info ──────────────────────────────────────────────────────────────

std::string V4L2DeviceInfo::canonical_path() const
{
  char resolved[PATH_MAX];
  if (realpath(device_path.c_str(), resolved) != nullptr) {
    return std::string(resolved);
  }
  return device_path;
}

// ─── Constructor / Destructor ───────────────────────────────────────────────

AstraRGBNode::AstraRGBNode(const rclcpp::NodeOptions & options)
: Node("astra_rgb_driver", options)
{
  declare_parameter("device", std::string("auto"));
  declare_parameter("width", 1280);
  declare_parameter("height", 720);
  declare_parameter("fps", 30);
  declare_parameter("pixel_format", std::string("MJPG"));
  declare_parameter("frame_id", std::string("camera_color_optical_frame"));
  declare_parameter("calibration_url", std::string(""));
  declare_parameter("reconnect_interval_ms", 1000);
  declare_parameter("frame_timeout_ms", 3000);
  declare_parameter("enable_depth", false);
  declare_parameter("use_gpu_decode", true);
  declare_parameter("require_gpu_decode", false);

  config_.device = get_parameter("device").as_string();
  config_.width = get_parameter("width").as_int();
  config_.height = get_parameter("height").as_int();
  config_.fps = std::max(1, static_cast<int>(get_parameter("fps").as_int()));
  config_.pixel_format = get_parameter("pixel_format").as_string();
  config_.frame_id = get_parameter("frame_id").as_string();
  config_.calibration_url = get_parameter("calibration_url").as_string();
  config_.reconnect_interval_ms = std::max(200, static_cast<int>(get_parameter("reconnect_interval_ms").as_int()));
  config_.frame_timeout_ms = std::max(500, static_cast<int>(get_parameter("frame_timeout_ms").as_int()));
  config_.enable_depth = get_parameter("enable_depth").as_bool();
  use_gpu_decode_ = get_parameter("use_gpu_decode").as_bool();
  require_gpu_decode_ = get_parameter("require_gpu_decode").as_bool();
  // A strict GPU request must never be able to start the CPU V4L2 path by
  // configuration accident.  Force the hardware-decoder path on.
  if (require_gpu_decode_.load() && !use_gpu_decode_.load()) {
    use_gpu_decode_ = true;
    RCLCPP_INFO(get_logger(),
      "[CAMERA-CUDA] require_gpu_decode=true forces use_gpu_decode=true");
  }

  RCLCPP_INFO(get_logger(),
    "AstraRGB V4L2: device=%s %dx%d@%d fmt=%s frame=%s",
    config_.device.c_str(), config_.width, config_.height,
    config_.fps, config_.pixel_format.c_str(), config_.frame_id.c_str());

  // Publishers
  auto qos = rclcpp::SensorDataQoS().keep_last(1);
  pub_image_ = create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", qos);
  pub_camera_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", qos);
  auto status_qos = rclcpp::QoS(1).transient_local().reliable();
  pub_status_ = create_publisher<std_msgs::msg::String>(
    "/camera/color/status", status_qos);

  build_camera_info(config_.width, config_.height);

  // Start workers
  running_ = true;
  // Start reconnect thread. It will wait until reconnect_requested_ is set.
  // Initial discovery happens via the first schedule_reconnect() below.
  reader_thread_ = std::thread(&AstraRGBNode::read_loop, this);
  reconnect_thread_ = std::thread(&AstraRGBNode::reconnect_loop, this);

  // Trigger initial camera discovery
  schedule_reconnect();

  // Stats timer
  stats_timer_ = create_wall_timer(
    std::chrono::seconds(5),
    [this]() {
      const uint64_t r = frames_received_.load();
      const uint64_t p = frames_published_.load();
      const uint64_t e = errors_.load();
      const uint64_t last_published = last_stats_published_.load();
      const auto now = std::chrono::steady_clock::now();
      if (last_stats_wall_.time_since_epoch().count() != 0) {
        const double elapsed = std::chrono::duration<double>(
          now - last_stats_wall_).count();
        if (elapsed > 1.0) {
          const double fps = static_cast<double>(p - last_published) / elapsed;
          const char * backend = gst_running_.load() ? "CUDA-NVV4L2" : "V4L2-CPU";
          RCLCPP_INFO(get_logger(),
            "[CAMERA-STATS] rx=%lu pub=%lu err=%lu fps=%.1f target=%d backend=%s",
            r, p, e, fps, config_.fps, backend);
          // Refresh the transient-local status with measured FPS so the GUI can
          // report camera health without subscribing to the full-rate image topic
          // while a non-camera page is visible.
          std::ostringstream status;
          status << "backend=" << backend
                 << " device=" << (active_device_path_.empty() ? config_.device : active_device_path_)
                 << " size=" << config_.width << "x" << config_.height
                 << " target_fps=" << config_.fps
                 << " measured_fps=" << std::fixed << std::setprecision(1) << fps
                 << " errors=" << e;
          publish_status(status.str());
        }
      }
      last_stats_published_.store(p);
      last_stats_wall_ = now;
    });
}

AstraRGBNode::~AstraRGBNode()
{
  running_ = false;
  reconnect_requested_ = true;

  close_gst_pipeline();

  if (reader_thread_.joinable()) reader_thread_.join();
  if (reconnect_thread_.joinable()) reconnect_thread_.join();

  stream_off_and_unmap();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  if (lock_fd_ >= 0) {
    ::close(lock_fd_);
    lock_fd_ = -1;
  }
}

// ─── Device Discovery ────────────────────────────────────────────────────────

std::vector<V4L2DeviceInfo> AstraRGBNode::discover_devices()
{
  std::vector<V4L2DeviceInfo> candidates;

  auto try_open_device = [&](const std::string & path) -> std::optional<V4L2DeviceInfo> {
    int dev_fd = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (dev_fd < 0) {
      // Query-only fallback makes discovery robust when a transient owner still
      // holds write access. open_device() below remains the authority for actual
      // streaming and reports the concrete permission/busy error.
      dev_fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    }
    if (dev_fd < 0) return std::nullopt;

    struct v4l2_capability cap{};
    if (ioctl(dev_fd, VIDIOC_QUERYCAP, &cap) != 0) {
      ::close(dev_fd);
      return std::nullopt;
    }

    // Must be a video capture device
    const uint32_t cap_flags = cap.capabilities;
    if ((cap_flags & V4L2_CAP_VIDEO_CAPTURE) == 0 &&
        (cap_flags & V4L2_CAP_VIDEO_CAPTURE_MPLANE) == 0) {
      ::close(dev_fd);
      return std::nullopt;
    }

    V4L2DeviceInfo info;
    info.device_path = path;
    info.card = reinterpret_cast<const char*>(cap.card);
    info.bus_info = reinterpret_cast<const char*>(cap.bus_info);
    info.driver = reinterpret_cast<const char*>(cap.driver);
    info.vid = 0;
    info.pid = 0;
    info.capabilities = cap_flags;

    // Resolve symlinks to get the canonical /dev/videoX
    char resolved[PATH_MAX];
    if (realpath(path.c_str(), resolved) != nullptr) {
      info.device_path = std::string(resolved);
    }

    // Read USB metadata from sysfs
    const std::string dev_name = info.device_path.substr(
      info.device_path.find_last_of('/') + 1);
    const std::string sysfs_base =
      "/sys/class/video4linux/" + dev_name + "/device";

    struct stat st{};
    if (stat(sysfs_base.c_str(), &st) == 0) {
      const std::string uevent_path = sysfs_base + "/uevent";
      std::ifstream uf(uevent_path);
      std::string line;
      while (std::getline(uf, line)) {
        if (line.rfind("PRODUCT=", 0) == 0) {
          std::string val = line.substr(8);
          std::replace(val.begin(), val.end(), '/', ' ');
          std::istringstream ss(val);
          unsigned int v = 0, p = 0;
          ss >> std::hex >> v >> p;
          info.vid = static_cast<uint16_t>(v & 0xFFFF);
          info.pid = static_cast<uint16_t>(p & 0xFFFF);
        } else if (line.rfind("ID_SERIAL_SHORT=", 0) == 0) {
          info.serial = line.substr(16);
        }
      }
    }

    ::close(dev_fd);
    return info;
  };

  // When the launch layer has already resolved a physical USB-hub role, obey
  // that exact alias. Do NOT scan/open every /dev/videoX node again. The alias
  // can itself point to /dev/v4l/by-id or /dev/v4l/by-path, so it remains stable
  // even when the kernel renumbers video0/video1/video2 after a hub reset.
  if (!config_.device.empty() && config_.device != "auto") {
    auto info = try_open_device(config_.device);
    if (!info) {
      RCLCPP_INFO(get_logger(),
        "[CAMERA-ROLE] assigned device %s is not ready yet; retrying only this role",
        config_.device.c_str());
      return candidates;
    }
    info->by_path = config_.device;
    info->score = 10000;
    candidates.push_back(*info);
    RCLCPP_INFO(get_logger(),
      "[CAMERA-ROLE] fixed camera alias %s -> %s",
      config_.device.c_str(), info->canonical_path().c_str());
    return candidates;
  }

  // Priority 1: /dev/v4l/by-id/* — stable USB-device identity aliases
  {
    DIR * d = opendir("/dev/v4l/by-id");
    if (d) {
      struct dirent * ent;
      while ((ent = readdir(d)) != nullptr) {
        if (ent->d_name[0] == '.') continue;
        const std::string p = std::string("/dev/v4l/by-id/") + ent->d_name;
        auto info = try_open_device(p);
        if (info) {
          info->by_id = p;
          candidates.push_back(*info);
        }
      }
      closedir(d);
    }
  }

  // Priority 2: /dev/v4l/by-path/* — stable USB bus-location aliases
  {
    DIR * d = opendir("/dev/v4l/by-path");
    if (d) {
      struct dirent * ent;
      while ((ent = readdir(d)) != nullptr) {
        if (ent->d_name[0] == '.') continue;
        const std::string p = std::string("/dev/v4l/by-path/") + ent->d_name;
        // Skip if canonical path already in candidates
        char resolved[PATH_MAX];
        if (realpath(p.c_str(), resolved) != nullptr) {
          bool already = false;
          for (const auto & c : candidates) {
            if (c.canonical_path() == std::string(resolved)) {
              already = true; break;
            }
          }
          if (already) continue;
        }
        auto info = try_open_device(p);
        if (info) {
          info->by_path = p;
          candidates.push_back(*info);
        }
      }
      closedir(d);
    }
  }

  // Priority 3: /dev/video0..31 — raw device nodes (last resort)
  for (int i = 0; i < 32; ++i) {
    const std::string p = "/dev/video" + std::to_string(i);
    struct stat st{};
    if (stat(p.c_str(), &st) != 0) continue;
    // Skip if already discovered
    bool already = false;
    for (const auto & c : candidates) {
      if (c.canonical_path() == p) { already = true; break; }
    }
    if (already) continue;
    auto info = try_open_device(p);
    if (info) candidates.push_back(*info);
  }

  // Score and log all candidates
  for (auto & info : candidates) {
    int s = 0;
    score_device(info, &s);
    info.score = s;

    std::cout << "[CAMERA-CANDIDATE] device=" << info.device_path
              << " by_id=" << info.by_id
              << " card=" << info.card
              << " driver=" << info.driver
              << " vid=0x" << std::hex << info.vid << std::dec
              << " pid=0x" << std::hex << info.pid << std::dec
              << " score=" << info.score << std::endl;
  }

  // Sort descending by score
  std::sort(candidates.begin(), candidates.end(),
    [](const V4L2DeviceInfo & a, const V4L2DeviceInfo & b) {
      return a.score > b.score;
    });

  return candidates;
}

bool AstraRGBNode::score_device(const V4L2DeviceInfo & info, int * out_score) const
{
  int s = 0;

  std::string combined = info.card + " " + info.bus_info + " " +
    info.by_id + " " + info.by_path + " " + info.manufacturer + " " +
    info.product + " " + info.serial;
  std::transform(combined.begin(), combined.end(), combined.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  // Astra/Orbbec brand identifiers — highest priority. Comparison is
  // case-insensitive because V4L2 card names commonly contain "Astra".
  if (combined.find("astra") != std::string::npos) s += 100;
  if (combined.find("orbbec") != std::string::npos) s += 90;
  if (combined.find("astrapro") != std::string::npos) s += 95;

  // Known USB VID/PID for depth/RGB cameras
  if (info.vid == 0x05e0) s += 80;    // Orbbec
  if (info.vid == 0x2bc5) s += 80;    // Orbbec/Astra
  if (info.vid == 0x1de1) s += 70;    // RealSense / generic UVC

  // Stable by-id alias — prevents USB hub index swaps. UVC devices commonly
  // expose index0 as the real RGB capture node and index1 as an auxiliary /
  // metadata node. The 20260819 runtime showed both tied at score=250, causing
  // index1 to be tried first and VIDIOC_G_FMT to fail before index0 succeeded.
  if (!info.by_id.empty()) s += 40;
  if (!info.by_path.empty()) s += 30;
  if (info.by_id.find("video-index0") != std::string::npos) s += 120;
  if (info.by_id.find("video-index1") != std::string::npos) s -= 120;

  // UVC driver preferred for USB webcams
  if (info.driver == "uvcvideo") s += 20;

  // USB bus presence
  if (!info.bus_info.empty() && info.bus_info.find("usb") != std::string::npos) {
    s += 10;
  }

  // Penalty: metadata-only or sub-device nodes
  if (info.card.find("meta") != std::string::npos) s -= 200;
  if (info.card.find("depth") != std::string::npos &&
      info.card.find("rgb") == std::string::npos) s -= 50;

  *out_score = s;
  return s > 0;
}

std::string AstraRGBNode::resolve_device(const std::string & hint) const
{
  if (hint.empty() || hint == "auto") return "";

  char resolved[PATH_MAX];
  if (realpath(hint.c_str(), resolved) != nullptr) {
    return std::string(resolved);
  }
  return hint;
}

// ─── Format Configuration ────────────────────────────────────────────────────

bool AstraRGBNode::configure_format()
{
  struct v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (ioctl(fd_, VIDIOC_G_FMT, &fmt) != 0) {
    RCLCPP_ERROR(get_logger(), "VIDIOC_G_FMT failed: %s", strerror(errno));
    return false;
  }

  uint32_t desired_fmt = fourcc_from_str(config_.pixel_format.c_str());
  fmt.fmt.pix.width = static_cast<uint32_t>(config_.width);
  fmt.fmt.pix.height = static_cast<uint32_t>(config_.height);
  fmt.fmt.pix.pixelformat = desired_fmt;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;

  if (ioctl(fd_, VIDIOC_S_FMT, &fmt) != 0) {
    // Try fallback formats if requested one is not supported
    const char * fallbacks[] = {"YUYV", "MJPG", "RGB24", "BGR3"};
    for (const char * f : fallbacks) {
      if (config_.pixel_format == f) continue;
      fmt.fmt.pix.pixelformat = fourcc_from_str(f);
      if (ioctl(fd_, VIDIOC_S_FMT, &fmt) == 0) {
        RCLCPP_INFO(get_logger(),
          "Format fallback: requested=%s -> actual=%s",
          config_.pixel_format.c_str(), f);
        break;
      }
    }
    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) != 0) {
      RCLCPP_ERROR(get_logger(), "VIDIOC_S_FMT failed: %s", strerror(errno));
      return false;
    }
  }

  // Set framerate
  struct v4l2_streamparm sp{};
  sp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_G_PARM, &sp) == 0) {
    if ((sp.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) != 0) {
      sp.parm.capture.timeperframe.numerator = 1;
      sp.parm.capture.timeperframe.denominator = static_cast<uint32_t>(config_.fps);
      ioctl(fd_, VIDIOC_S_PARM, &sp);
    }
  }

  // Read back actual
  struct v4l2_format query_fmt{};
  query_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(fd_, VIDIOC_G_FMT, &query_fmt);

  const uint32_t actual_w = query_fmt.fmt.pix.width;
  const uint32_t actual_h = query_fmt.fmt.pix.height;
  const char * actual_fmt_str = fourcc_to_str(query_fmt.fmt.pix.pixelformat);

  std::cout << "[CAMERA-CONFIG] requested=" << config_.width << "x" << config_.height
             << "@" << config_.fps << " fmt=" << config_.pixel_format
             << " | actual=" << actual_w << "x" << actual_h
             << " fmt=" << actual_fmt_str << std::endl;

  RCLCPP_INFO(get_logger(),
    "[CAMERA-CONFIG] requested=%dx%d@%d fmt=%s | actual=%dx%d fmt=%s",
    config_.width, config_.height, config_.fps, config_.pixel_format.c_str(),
    actual_w, actual_h, actual_fmt_str);

  build_camera_info(static_cast<int>(actual_w), static_cast<int>(actual_h));
  return true;
}

// ─── Buffer Management ────────────────────────────────────────────────────────

bool AstraRGBNode::reqbuf_and_queue()
{
  struct v4l2_requestbuffers req{};
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd_, VIDIOC_REQBUFS, &req) != 0) {
    RCLCPP_ERROR(get_logger(), "VIDIOC_REQBUFS failed: %s", strerror(errno));
    return false;
  }

  if (req.count < 2) {
    RCLCPP_ERROR(get_logger(), "Driver allocated only %u buffers", req.count);
    return false;
  }

  buffers_.resize(req.count);
  for (unsigned int i = 0; i < req.count; ++i) {
    struct v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) != 0) {
      RCLCPP_ERROR(get_logger(), "VIDIOC_QUERYBUF[%u] failed", i);
      buffers_.clear();
      return false;
    }

    void * ptr = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                       MAP_SHARED, fd_, buf.m.offset);
    if (ptr == MAP_FAILED) {
      RCLCPP_ERROR(get_logger(), "mmap buffer[%u] failed: %s", i, strerror(errno));
      buffers_.clear();
      return false;
    }

    buffers_[i].start = ptr;
    buffers_[i].length = buf.length;
    buffers_[i].index = static_cast<int>(i);

    if (ioctl(fd_, VIDIOC_QBUF, &buf) != 0) {
      RCLCPP_ERROR(get_logger(), "VIDIOC_QBUF[%u] failed", i);
      munmap(ptr, buf.length);
      buffers_.clear();
      return false;
    }
  }

  return true;
}

bool AstraRGBNode::stream_on()
{
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_STREAMON, &type) != 0) {
    RCLCPP_ERROR(get_logger(), "VIDIOC_STREAMON failed: %s", strerror(errno));
    return false;
  }
  std::cout << "[CAMERA-STREAM-ON]" << std::endl;
  RCLCPP_INFO(get_logger(), "[CAMERA-STREAM-ON]");
  return true;
}

void AstraRGBNode::stream_off_and_unmap()
{
  if (fd_ >= 0) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd_, VIDIOC_STREAMOFF, &type);
  }
  for (auto & b : buffers_) {
    if (b.start && b.length > 0) {
      munmap(b.start, b.length);
    }
  }
  buffers_.clear();
}

// ─── Open Device ─────────────────────────────────────────────────────────────

bool AstraRGBNode::open_device(const V4L2DeviceInfo & dev)
{
  // Inter-process lock: prevent two camera processes from racing to open the same device
  const std::string lock_name =
    dev.device_path.substr(dev.device_path.find_last_of('/') + 1) + ".lock";
  const std::string lock_path = "/tmp/agv_camera_" + lock_name;

  if (lock_fd_ >= 0) {
    ::close(lock_fd_);
    lock_fd_ = -1;
  }
  lock_fd_ = open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0666);
  if (lock_fd_ >= 0) {
    struct flock fl{};
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    if (fcntl(lock_fd_, F_SETLK, &fl) != 0) {
      RCLCPP_WARN(get_logger(), "Device %s already locked", dev.device_path.c_str());
      ::close(lock_fd_);
      lock_fd_ = -1;
      return false;
    }
    // Keep the descriptor as a member so the lock remains valid for the active
    // camera and can be released cleanly before rediscovery.
  }

  fd_ = ::open(dev.device_path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to open %s: %s",
      dev.device_path.c_str(), strerror(errno));
    if (lock_fd_ >= 0) { ::close(lock_fd_); lock_fd_ = -1; }
    return false;
  }

  std::cout << "[CAMERA-OPEN-OK] device=" << dev.device_path << std::endl;
  RCLCPP_INFO(get_logger(), "[CAMERA-OPEN-OK] device=%s", dev.device_path.c_str());

  active_device_path_ = dev.device_path;
  active_by_id_ = dev.by_id;
  active_by_path_ = dev.by_path;
  return true;
}

// ─── Reconnect ───────────────────────────────────────────────────────────────

void AstraRGBNode::schedule_reconnect()
{
  reconnect_requested_ = true;
}

void AstraRGBNode::reconnect_loop()
{
  while (running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(
      static_cast<int>(config_.reconnect_interval_ms)));

    if (!running_) break;
    if (!reconnect_requested_.load()) continue;
    reconnect_requested_ = false;

    RCLCPP_INFO(get_logger(), "[CAMERA-RECONNECT] Attempting camera re-discovery...");

    // V16: reader and reconnect must never use the same fd/buffers concurrently.
    // Holding this lock through open/configure/STREAMON prevents the reader from
    // observing fd_ while the device is only OPENING (the V15 EINVAL storm).
    std::lock_guard<std::mutex> stream_lock(stream_mutex_);
    stream_state_ = StreamState::CLOSED;
    close_gst_pipeline();
    stream_off_and_unmap();
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    if (lock_fd_ >= 0) {
      ::close(lock_fd_);
      lock_fd_ = -1;
    }

    auto candidates = discover_devices();
    bool connected = false;

    for (const auto & cand : candidates) {
      if (cand.score < 10) continue;
      if (!open_device(cand)) continue;

      if (use_gpu_decode_.load()) {
        // GPU path: GStreamer owns the V4L2 stream. close the discovery fd so
        // v4l2src is the single live opener while retaining our process lock.
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        if (open_gst_pipeline(cand.device_path)) {
          stream_state_ = StreamState::STREAMING;
          reconnect_count_++;
          last_frame_wall_ = std::chrono::steady_clock::now();
          RCLCPP_INFO(get_logger(),
            "[CAMERA-CONNECTED] device=%s by_id=%s by_path=%s (GPU decode)",
            cand.device_path.c_str(), cand.by_id.c_str(), cand.by_path.c_str());
          publish_status("backend=CUDA-NVV4L2 decoder=nvv4l2decoder device=" +
                         cand.device_path + " resolution=" +
                         std::to_string(config_.width) + "x" + std::to_string(config_.height) +
                         " fps=" + std::to_string(config_.fps));
          connected = true;
          reconnect_requested_ = false;
          break;
        }
        if (require_gpu_decode_.load()) {
          RCLCPP_INFO(get_logger(),
            "[CAMERA-CUDA] hardware decode not ready for %s; strict CUDA mode will retry (CPU fallback disabled)",
            cand.device_path.c_str());
          if (lock_fd_ >= 0) { ::close(lock_fd_); lock_fd_ = -1; }
          continue;
        }
        RCLCPP_INFO(get_logger(),
          "[CAMERA-FALLBACK] GPU decode unavailable; CPU V4L2 fallback explicitly permitted for %s",
          cand.device_path.c_str());
        if (lock_fd_ >= 0) { ::close(lock_fd_); lock_fd_ = -1; }
        if (!open_device(cand)) continue;
      }

      stream_state_ = StreamState::OPENING;
      bool ok = configure_format() && reqbuf_and_queue() && stream_on();
      if (!ok) {
        stream_state_ = StreamState::CLOSED;
        // Clean up after failed attempt
        stream_off_and_unmap();
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
        if (lock_fd_ >= 0) { ::close(lock_fd_); lock_fd_ = -1; }
        continue;
      }

      stream_state_ = StreamState::STREAMING;
      reconnect_count_++;
      last_frame_wall_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(get_logger(),
        "[CAMERA-CONNECTED] device=%s by_id=%s by_path=%s",
        cand.device_path.c_str(), cand.by_id.c_str(), cand.by_path.c_str());
      RCLCPP_INFO(get_logger(),
        "[CAMERA-RECONNECTED] device=%s score=%d attempt=%lu",
        cand.device_path.c_str(), cand.score,
        static_cast<unsigned long>(reconnect_count_.load()));
      publish_status("backend=V4L2-CPU device=" + cand.device_path +
                     " resolution=" + std::to_string(config_.width) + "x" +
                     std::to_string(config_.height) + " fps=" + std::to_string(config_.fps));
      connected = true;
      // Signal success — stop reconnect loop until error triggers it again
      reconnect_requested_ = false;
      break;
    }

    if (!connected) {
      int video_nodes = 0;
      for (int i = 0; i < 32; ++i) {
        const std::string path = "/dev/video" + std::to_string(i);
        struct stat st{};
        if (stat(path.c_str(), &st) == 0) ++video_nodes;
      }
      RCLCPP_INFO(get_logger(),
        "[CAMERA-RECONNECT] No suitable device found; /dev/video*= %d; retrying in %d ms",
        video_nodes, config_.reconnect_interval_ms);
      publish_status(
        "state=WAITING_ENUMERATION device=" + config_.device +
        " video_nodes=" + std::to_string(video_nodes) +
        " target_fps=" + std::to_string(config_.fps));
      reconnect_requested_ = true;
    }
  }
}

// ─── Read Loop ───────────────────────────────────────────────────────────────

void AstraRGBNode::read_loop()
{
  // Map V4L2 capture timestamps to ROS time once per reader thread.  Using the
  // kernel capture clock preserves inter-frame timing and avoids stamping every
  // frame at the later decode/publish instant.  Re-anchor if the device clock
  // resets after a USB reconnect.
  bool capture_anchor_valid = false;
  int64_t capture_anchor_ns = 0;
  rclcpp::Time ros_anchor = this->now();
  while (running_) {
    // V16 serializes the complete V4L2 dequeue/requeue transaction with reconnect.
    // The reconnect thread may close/unmap only after this transaction releases
    // the mutex. Conversely, the reader cannot see a freshly opened fd until
    // configure_format + REQBUFS + STREAMON have all completed.
    std::unique_lock<std::mutex> stream_lock(stream_mutex_);
    if (fd_ < 0 || stream_state_.load() != StreamState::STREAMING) {
      stream_lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    const int active_fd = fd_;
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(active_fd, &fds);
    struct timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 500000;

    int ret = select(active_fd + 1, &fds, nullptr, nullptr, &tv);
    if (ret < 0) {
      if (errno == EINTR) continue;
      RCLCPP_INFO(get_logger(), "[CAMERA-RECOVERY] select() transport event: %s", strerror(errno));
      errors_++;
      stream_state_ = StreamState::CLOSED;
      schedule_reconnect();
      continue;
    }
    if (ret == 0) {
      continue;
    }

    struct v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(active_fd, VIDIOC_DQBUF, &buf) != 0) {
      if (errno == EAGAIN) continue;
      RCLCPP_INFO(get_logger(),
        "[CAMERA-RECOVERY] VIDIOC_DQBUF transport event: %s; reconnect scheduled",
        strerror(errno));
      errors_++;
      stream_state_ = StreamState::CLOSED;
      schedule_reconnect();
      continue;
    }

    if (static_cast<size_t>(buf.index) >= buffers_.size()) {
      ioctl(active_fd, VIDIOC_QBUF, &buf);
      continue;
    }

    const uint8_t * frame_data = static_cast<const uint8_t *>(buffers_[buf.index].start);
    const size_t frame_size = buf.bytesused;
    if (frame_size > 0) {
      frames_received_++;
      last_frame_wall_ = std::chrono::steady_clock::now();
      const int64_t capture_ns =
        static_cast<int64_t>(buf.timestamp.tv_sec) * 1000000000LL +
        static_cast<int64_t>(buf.timestamp.tv_usec) * 1000LL;
      if (!capture_anchor_valid || capture_ns < capture_anchor_ns) {
        capture_anchor_valid = true;
        capture_anchor_ns = capture_ns;
        ros_anchor = this->now();
      }
      const rclcpp::Time capture_stamp = ros_anchor +
        rclcpp::Duration::from_nanoseconds(capture_ns - capture_anchor_ns);
      publish_frame(frame_data, frame_size, capture_stamp);
    }

    if (ioctl(active_fd, VIDIOC_QBUF, &buf) != 0) {
      RCLCPP_INFO(get_logger(),
        "[CAMERA-RECOVERY] VIDIOC_QBUF transport event: %s; reconnect scheduled",
        strerror(errno));
      errors_++;
      stream_state_ = StreamState::CLOSED;
      schedule_reconnect();
    }
  }
}

// ─── GStreamer GPU Decode (NVIDIA nvv4l2decoder) ──────────────────────────────

bool AstraRGBNode::open_gst_pipeline(const std::string & device)
{
  if (!gst_is_initialized()) {
    if (!gst_init_check(nullptr, nullptr, nullptr)) {
      RCLCPP_INFO(get_logger(), "[CAMERA-CUDA] GStreamer init not ready; retrying hardware pipeline");
      return false;
    }
  }
  const int w = config_.width;
  const int h = config_.height;
  const int fps = config_.fps;

  std::ostringstream pipe;
  pipe << "v4l2src device=" << device << " io-mode=2 do-timestamp=true num-buffers=-1 ! "
       << "image/jpeg,width=" << w << ",height=" << h << ",framerate=" << fps << "/1 ! "
       << "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
       << "nvv4l2decoder mjpeg=1 ! "
       << "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
       << "nvvidconv ! video/x-raw,format=BGRx ! "
       << "videoconvert ! video/x-raw,format=BGR ! "
       << "queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
       << "appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false enable-last-sample=false";

  GError * err = nullptr;
  GstElement * pipeline = gst_parse_launch(pipe.str().c_str(), &err);
  if (!pipeline || err) {
    RCLCPP_INFO(get_logger(), "[CAMERA-CUDA] pipeline parse not ready: %s",
                err ? err->message : "null");
    if (err) g_error_free(err);
    return false;
  }

  GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_INFO(get_logger(), "[CAMERA-CUDA] set_state PLAYING not ready; will retry");
    gst_object_unref(pipeline);
    return false;
  }

  // Wait for preroll/negotiation to complete (avoids empty-caps crash).
  GstState state = GST_STATE_VOID_PENDING;
  gst_element_get_state(pipeline, &state, nullptr, 3 * GST_SECOND);

  gst_pipeline_ = static_cast<void *>(pipeline);
  gst_running_ = true;
  gst_thread_ = std::thread(&AstraRGBNode::gst_read_loop, this);

  RCLCPP_INFO(get_logger(),
    "[CAMERA-CUDA] GST-GPU pipeline active: device=%s %dx%d@%d via nvv4l2decoder (NVIDIA hardware MJPG decode)",
    device.c_str(), w, h, fps);
  return true;
}

void AstraRGBNode::close_gst_pipeline()
{
  gst_running_ = false;
  if (gst_thread_.joinable()) {
    gst_thread_.join();
  }
  if (gst_pipeline_) {
    GstElement * pipeline = static_cast<GstElement *>(gst_pipeline_);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    gst_pipeline_ = nullptr;
  }
}

void AstraRGBNode::gst_read_loop()
{
  GstElement * pipeline = static_cast<GstElement *>(gst_pipeline_);
  GstAppSink * sink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline), "sink"));
  if (!sink) {
    RCLCPP_INFO(get_logger(), "[CAMERA-CUDA] appsink 'sink' unavailable; reconnect requested");
    stream_state_ = StreamState::CLOSED;
    reconnect_requested_ = true;
    gst_running_ = false;
    return;
  }

  // GStreamer PTS is pipeline running-time, not ROS epoch time. Anchor the first
  // valid PTS to ROS now and preserve subsequent PTS deltas. This makes image
  // header stamps represent capture/decode pipeline timing instead of callback
  // execution time.
  bool pts_anchor_valid = false;
  GstClockTime pts_anchor = GST_CLOCK_TIME_NONE;
  rclcpp::Time pts_ros_anchor = this->now();

  while (gst_running_.load()) {
    GstSample * sample = gst_app_sink_try_pull_sample(sink, 100 * GST_MSECOND);
    if (!sample) {
      if (last_frame_wall_.time_since_epoch().count() != 0) {
        const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - last_frame_wall_).count();
        if (age_ms > config_.frame_timeout_ms) {
          RCLCPP_INFO(get_logger(),
            "[CAMERA-CUDA] frame timeout (%ld ms); scheduling clean GPU reconnect",
            static_cast<long>(age_ms));
          stream_state_ = StreamState::CLOSED;
          reconnect_requested_ = true;
          gst_running_ = false;
          break;
        }
      }
      continue;
    }

    GstCaps * caps = gst_sample_get_caps(sample);
    if (!caps) { gst_sample_unref(sample); continue; }

    GstStructure * s = gst_caps_get_structure(caps, 0);
    int w = 0, h = 0;
    gst_structure_get_int(s, "width", &w);
    gst_structure_get_int(s, "height", &h);

    GstBuffer * buf = gst_sample_get_buffer(sample);
    GstMapInfo info;
    if (!buf || !gst_buffer_map(buf, &info, GST_MAP_READ)) {
      gst_sample_unref(sample);
      continue;
    }

    // The GStreamer pipeline already outputs BGR. Publish bgr8 directly to
    // avoid an extra full-frame cv::cvtColor + allocation on the CPU.
    cv::Mat bgr(h, w, CV_8UC3, const_cast<guint8 *>(info.data));

    rclcpp::Time stamp = this->now();
    const GstClockTime pts = GST_BUFFER_PTS(buf);
    if (GST_CLOCK_TIME_IS_VALID(pts)) {
      if (!pts_anchor_valid || pts < pts_anchor) {
        pts_anchor_valid = true;
        pts_anchor = pts;
        pts_ros_anchor = this->now();
      }
      stamp = pts_ros_anchor + rclcpp::Duration::from_nanoseconds(
        static_cast<int64_t>(pts - pts_anchor));
    }
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = config_.frame_id;
    msg.height = static_cast<uint32_t>(bgr.rows);
    msg.width = static_cast<uint32_t>(bgr.cols);
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = static_cast<uint32_t>(bgr.cols * 3);
    msg.data.assign(bgr.datastart, bgr.dataend);

    pub_image_->publish(msg);

    sensor_msgs::msg::CameraInfo ci = camera_info_;
    ci.header.stamp = stamp;
    ci.header.frame_id = config_.frame_id;
    pub_camera_info_->publish(ci);

    frames_received_++;
    frames_published_++;
    last_frame_wall_ = std::chrono::steady_clock::now();

    static bool first = false;
    if (!first) {
      RCLCPP_INFO(get_logger(),
        "[CAMERA-FIRST-FRAME] GST-GPU device=%s size=%ux%u ros_encoding=bgr8 topic=/camera/color/image_raw",
        active_device_path_.c_str(), msg.width, msg.height);
      first = true;
    }

    gst_buffer_unmap(buf, &info);
    gst_sample_unref(sample);
  }
  gst_object_unref(sink);
}

void AstraRGBNode::publish_status(const std::string & text)
{
  if (!pub_status_) return;
  std_msgs::msg::String msg;
  msg.data = text;
  pub_status_->publish(msg);
}

// ─── Publish Frame ───────────────────────────────────────────────────────────

void AstraRGBNode::publish_frame(const uint8_t * data, size_t size,
                                const rclcpp::Time & stamp)
{
  cv::Mat img;
  std::string actual_format = "UNKNOWN";

  // Detect format and decode
  if (size > 2 && data[0] == 0xFF && data[1] == 0xD8) {
    // JPEG/MJPG — decode to BGR using OpenCV. Wrap the V4L2 buffer directly
    // instead of allocating/copying a temporary std::vector every frame.
    actual_format = "MJPG";
    cv::Mat jpeg_view(1, static_cast<int>(size), CV_8UC1, const_cast<uint8_t *>(data));
    img = cv::imdecode(jpeg_view, cv::IMREAD_COLOR);
    if (img.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "[CAMERA-DECODE-FAIL] MJPG decode failed, size=%zu", size);
      errors_++;
      return;
    }
  } else {
    // Raw format: need to know exact pixel format from V4L2
    // Query actual format from fd_
    struct v4l2_format query_fmt{};
    query_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_G_FMT, &query_fmt) == 0) {
      const uint32_t fourcc = query_fmt.fmt.pix.pixelformat;
      const int w = static_cast<int>(query_fmt.fmt.pix.width);
      const int h = static_cast<int>(query_fmt.fmt.pix.height);

      if (fourcc == fourcc_from_str("YUYV")) {
        // YUYV 4:2:2 → RGB
        actual_format = "YUYV";
        cv::Mat yuyv(h, w, CV_8UC2, const_cast<uint8_t*>(data));
        img = cv::Mat(h, w, CV_8UC3);
        cv::cvtColor(yuyv, img, cv::COLOR_YUV2BGR_YUYV);
      } else if (fourcc == fourcc_from_str("UYVY")) {
        // UYVY 4:2:2 → RGB
        actual_format = "UYVY";
        cv::Mat uyvy(h, w, CV_8UC2, const_cast<uint8_t*>(data));
        img = cv::Mat(h, w, CV_8UC3);
        cv::cvtColor(uyvy, img, cv::COLOR_YUV2BGR_UYVY);
      } else if (fourcc == fourcc_from_str("RGB3")) {
        // Convert the uncommon RGB24 fallback once into the same BGR format
        // used by every other camera backend.
        actual_format = "RGB3";
        cv::Mat rgb24(h, w, CV_8UC3, const_cast<uint8_t*>(data));
        cv::cvtColor(rgb24, img, cv::COLOR_RGB2BGR);
      } else if (fourcc == fourcc_from_str("BGR3")) {
        // BGR24 → already BGR, OpenCV native
        actual_format = "BGR3";
        cv::Mat bgr24(h, w, CV_8UC3, const_cast<uint8_t*>(data));
        img = bgr24.clone();
      } else {
        const char * fourcc_str = fourcc_to_str(fourcc);
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
          "[CAMERA-FORMAT-UNSUPPORTED] fourcc=%s (0x%08x) not supported",
          fourcc_str, fourcc);
        errors_++;
        return;
      }
    } else {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "[CAMERA-FORMAT-QUERY-FAIL] Cannot query V4L2 format");
      errors_++;
      return;
    }
  }

  if (img.empty()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
      "[CAMERA-DECODE-FAIL] format=%s size=%zu", actual_format.c_str(), size);
    errors_++;
    return;
  }

  // Publish BGR directly. The GPU path already uses bgr8 and all perception
  // consumers support it, so a full-frame BGR->RGB conversion here only wastes
  // CPU on the V4L2 fallback path.
  sensor_msgs::msg::Image msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = config_.frame_id;
  msg.height = static_cast<uint32_t>(img.rows);
  msg.width = static_cast<uint32_t>(img.cols);
  msg.encoding = "bgr8";
  msg.is_bigendian = false;
  msg.step = static_cast<uint32_t>(img.cols * 3);
  msg.data.assign(img.datastart, img.dataend);

  pub_image_->publish(msg);

  // Also publish CameraInfo with every frame (static, rebuilt on format change)
  sensor_msgs::msg::CameraInfo ci = camera_info_;
  ci.header.stamp = stamp;
  ci.header.frame_id = config_.frame_id;
  pub_camera_info_->publish(ci);

  frames_published_++;

  // Log first frame
  static bool first_frame_logged = false;
  if (!first_frame_logged) {
    RCLCPP_INFO(get_logger(),
      "[CAMERA-FIRST-FRAME] device=%s size=%ux%u source_fmt=%s ros_encoding=bgr8 topic=/camera/color/image_raw",
      active_device_path_.c_str(), msg.width, msg.height, actual_format.c_str());
    std::cout << "[CAMERA-FIRST-FRAME] device=" << active_device_path_
              << " size=" << msg.width << "x" << msg.height
              << " source_fmt=" << actual_format
              << " ros_encoding=bgr8"
              << " topic=/camera/color/image_raw" << std::endl;
    first_frame_logged = true;
  }
}

// ─── CameraInfo ─────────────────────────────────────────────────────────────

void AstraRGBNode::build_camera_info(int width, int height)
{
  sensor_msgs::msg::CameraInfo ci;
  ci.width = static_cast<uint32_t>(width);
  ci.height = static_cast<uint32_t>(height);

  // Approximate pinhole intrinsic — replace with camera_calibration output
  // fx ≈ fy ≈ width (wide-angle ~90° HFOV at 1280)
  ci.k[0] = static_cast<double>(width);          // fx
  ci.k[2] = static_cast<double>(width) / 2.0;    // cx
  ci.k[4] = static_cast<double>(width);           // fy
  ci.k[5] = static_cast<double>(height) / 2.0;   // cy
  ci.k[8] = 1.0;

  ci.distortion_model = "none";

  camera_info_ = ci;
}

// ─── Entry point ─────────────────────────────────────────────────────────────

}  // namespace astra_rgb_driver

#ifdef YOLO_COMPONENT_BUILD
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(astra_rgb_driver::AstraRGBNode)
#else
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<astra_rgb_driver::AstraRGBNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
