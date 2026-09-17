#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <cctype>
#include <cstring>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <fcntl.h>
#include <glob.h>
#include <optional>
#include <string>
#include <stdexcept>
#include <system_error>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <sys/ioctl.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
std::string trim_copy(std::string value)
{
  const auto not_space = [](unsigned char c) {return !std::isspace(c);};
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::string upper_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return value;
}

std::vector<std::string> glob_paths(const char * pattern)
{
  glob_t result{};
  std::vector<std::string> paths;
  if (::glob(pattern, GLOB_NOSORT, nullptr, &result) == 0) {
    paths.reserve(result.gl_pathc);
    for (std::size_t i = 0; i < result.gl_pathc; ++i) {
      paths.emplace_back(result.gl_pathv[i]);
    }
  }
  ::globfree(&result);
  std::sort(paths.begin(), paths.end());
  return paths;
}

bool contains_case_insensitive(const std::string & haystack, const std::string & needle)
{
  return upper_copy(haystack).find(upper_copy(needle)) != std::string::npos;
}


uint32_t wire_crc32(const std::string & data)
{
  uint32_t crc = 0xFFFFFFFFU;
  for (const unsigned char c : data) {
    crc ^= c;
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

double kv_double(const std::string & text, const std::string & key, double fallback = 0.0)
{
  const std::string marker = key + "=";
  const auto pos = text.find(marker);
  if (pos == std::string::npos) {
    return fallback;
  }
  const auto start = pos + marker.size();
  const auto end = text.find_first_of(" \t,;", start);
  try {
    return std::stod(text.substr(start, end == std::string::npos ? std::string::npos : end - start));
  } catch (const std::exception &) {
    return fallback;
  }
}

}  // namespace

class WinchSerialNode final : public rclcpp::Node
{
public:
  WinchSerialNode()
  : Node("winch_serial_node")
  {
    port_parameter_ = declare_parameter<std::string>("port", "auto");
    const std::int64_t baud = declare_parameter<std::int64_t>("baud", 115200);
    reconnect_interval_ms_ = std::max<std::int64_t>(
      200, declare_parameter<std::int64_t>("reconnect_interval_ms", 1000));
    status_period_ms_ = std::max<std::int64_t>(
      250, declare_parameter<std::int64_t>("status_period_ms", 1000));
    rx_timeout_ms_ = std::max<std::int64_t>(
      1000, declare_parameter<std::int64_t>("rx_timeout_ms", 5000));
    hmi_sync_enabled_ = declare_parameter<bool>("hmi_sync_enabled", true);
    hmi_sync_period_ms_ = std::max<std::int64_t>(
      200, declare_parameter<std::int64_t>("hmi_sync_period_ms", 500));

    if (baud != 115200 && baud != 1000000) {
      throw std::invalid_argument("F411 CDC baud must be 115200 or 1000000");
    }
    serial_baud_ = baud;

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    connected_pub_ = create_publisher<std_msgs::msg::Bool>("/winch/connected", latched);
    port_pub_ = create_publisher<std_msgs::msg::String>("/winch/port", latched);
    state_pub_ = create_publisher<std_msgs::msg::String>("/winch/state", latched);
    top_pub_ = create_publisher<std_msgs::msg::Bool>("/winch/top_limit", latched);
    bottom_pub_ = create_publisher<std_msgs::msg::Bool>("/winch/bottom_limit", latched);
    pwm_pub_ = create_publisher<std_msgs::msg::Float64>("/winch/pwm_pct", latched);
    direction_pub_ = create_publisher<std_msgs::msg::Int32>("/winch/direction", latched);
    raw_pub_ = create_publisher<std_msgs::msg::String>("/winch/raw", rclcpp::QoS(20).reliable());

    command_sub_ = create_subscription<std_msgs::msg::String>(
      "/winch/command", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {handle_command(msg->data);});

    // Display-only telemetry used by the STM32 HMI. These subscriptions never
    // command ESC/Nav2/perception; they only mirror already-published ROS state.
    esc_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/ready", 10, [this](std_msgs::msg::Bool::SharedPtr m) {esc_ready_ = m->data; last_esc_ready_rx_ = std::chrono::steady_clock::now();});
    drive_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/drive/connected", 10, [this](std_msgs::msg::Bool::SharedPtr m) {drive_connected_ = m->data; last_drive_connected_rx_ = std::chrono::steady_clock::now();});
    steer_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/steer/connected", 10, [this](std_msgs::msg::Bool::SharedPtr m) {steer_connected_ = m->data; last_steer_connected_rx_ = std::chrono::steady_clock::now();});
    encoder_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/steer/encoder_ready", 10, [this](std_msgs::msg::Bool::SharedPtr m) {encoder_ready_ = m->data; last_encoder_ready_rx_ = std::chrono::steady_clock::now();});
    motion_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/system/motion_ready", 10, [this](std_msgs::msg::Bool::SharedPtr m) {motion_ready_ = m->data; last_motion_ready_rx_ = std::chrono::steady_clock::now();});
    nav2_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/system/nav2_ready", 10, [this](std_msgs::msg::Bool::SharedPtr m) {nav2_ready_ = m->data; last_nav2_ready_rx_ = std::chrono::steady_clock::now();});
    localization_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/system/motion_localization_ready", 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {localization_ready_ = m->data; last_localization_ready_rx_ = std::chrono::steady_clock::now();});
    imu_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/imu/connected", 10, [this](std_msgs::msg::Bool::SharedPtr m) {imu_connected_ = m->data;});
    camera_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/perception/camera_connected", 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {
        camera_connected_ = m->data;
        last_camera_connected_rx_ = std::chrono::steady_clock::now();
      });
    camera_healthy_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/perception/camera_healthy", 10,
      [this](std_msgs::msg::Bool::SharedPtr m) {
        camera_healthy_ = m->data;
        last_camera_healthy_rx_ = std::chrono::steady_clock::now();
      });

    // These publishers use SensorDataQoS / BEST_EFFORT in the runtime stack.
    // Matching their QoS prevents the HMI bridge from silently missing fresh
    // telemetry while the local GUI still sees it.
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);
    imu_data_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos, [this](sensor_msgs::msg::Imu::SharedPtr m) {
        const auto & q = m->orientation;
        const double siny = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        imu_yaw_rad_ = std::atan2(siny, cosy);
        imu_connected_ = true;
        last_imu_data_rx_ = std::chrono::steady_clock::now();
      });
    camera_frame_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", sensor_qos,
      [this](sensor_msgs::msg::Image::SharedPtr) {
        last_camera_frame_rx_ = std::chrono::steady_clock::now();
      });
    camera_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/camera/color/status", sensor_qos,
      [this](std_msgs::msg::String::SharedPtr) {
        last_camera_status_rx_ = std::chrono::steady_clock::now();
      });
    drive_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/drive_actual_mps", sensor_qos, [this](std_msgs::msg::Float64::SharedPtr m) {
        drive_speed_mps_ = m->data; last_drive_speed_rx_ = std::chrono::steady_clock::now();
      });
    drive_target_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/drive_target_mps", sensor_qos, [this](std_msgs::msg::Float64::SharedPtr m) {
        drive_target_mps_ = m->data; last_drive_target_rx_ = std::chrono::steady_clock::now();
      });
    steering_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/steering_actual_rad", sensor_qos, [this](std_msgs::msg::Float64::SharedPtr m) {
        steering_rad_ = m->data; last_steering_rx_ = std::chrono::steady_clock::now();
      });
    steering_target_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/steering_target_rad", sensor_qos, [this](std_msgs::msg::Float64::SharedPtr m) {
        steering_target_rad_ = m->data; last_steering_target_rx_ = std::chrono::steady_clock::now();
      });
    yolo_perf_sub_ = create_subscription<std_msgs::msg::String>(
      "/obstacle_detection/performance", sensor_qos,
      [this](std_msgs::msg::String::SharedPtr m) {
        yolo_perf_seen_ = true;
        last_yolo_rx_ = std::chrono::steady_clock::now();
        yolo_fps_ = kv_double(m->data, "fps", 0.0);
        yolo_inference_ms_ = kv_double(m->data, "inference_ms", 0.0);
        yolo_detections_ = static_cast<int>(std::max(0.0, kv_double(m->data, "detections", 0.0)));
      });
    person_perf_sub_ = create_subscription<std_msgs::msg::String>(
      "/warehouse_obstacle/performance", sensor_qos,
      [this](std_msgs::msg::String::SharedPtr m) {
        person_count_ = static_cast<int>(std::max(0.0, kv_double(m->data, "detections", 0.0)));
      });
    goal_state_sub_ = create_subscription<std_msgs::msg::String>(
      "/navigation/goal_state", 10, [this](std_msgs::msg::String::SharedPtr m) {goal_state_ = m->data; last_goal_state_rx_ = std::chrono::steady_clock::now();});
    planner_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/navigation/planner_status", 10, [this](std_msgs::msg::String::SharedPtr m) {
        planner_status_ = m->data;
        last_planner_status_rx_ = std::chrono::steady_clock::now();
      });
    amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10, [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
        pose_x_ = m->pose.pose.position.x;
        pose_y_ = m->pose.pose.position.y;
        const auto & q = m->pose.pose.orientation;
        const double siny = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        heading_rad_ = std::atan2(siny, cosy);
        have_pose_ = true; last_pose_rx_ = std::chrono::steady_clock::now();
      });
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
        goal_x_ = m->pose.position.x;
        goal_y_ = m->pose.position.y;
        have_goal_ = true; last_goal_rx_ = std::chrono::steady_clock::now();
      });

    publish_connected(false);
    publish_string(port_pub_, "-");
    publish_string(state_pub_, "DISCONNECTED");

    const std::int64_t tick_ms = std::min<std::int64_t>(
      50, std::max<std::int64_t>(10, reconnect_interval_ms_ / 10));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(tick_ms), std::bind(&WinchSerialNode::tick, this));
  }

  ~WinchSerialNode() override
  {
    if (fd_ >= 0) {
      write_line("STOP");
    }
    close_serial(false);
  }

private:
  void tick()
  {
    const auto now = std::chrono::steady_clock::now();
    if (fd_ < 0) {
      if (now >= next_reconnect_) {
        try_connect();
        next_reconnect_ = now + std::chrono::milliseconds(reconnect_interval_ms_);
      }
      return;
    }

    read_serial();
    if (fd_ < 0) {
      return;
    }

    if (!handshake_confirmed_) {
      if (now >= next_host_hello_ && now < handshake_deadline_) {
        if (!write_line("HOST:HELLO:" + std::to_string(host_session_token_))) {
          close_serial(true);
          next_reconnect_ = now;
          return;
        }
        next_host_hello_ = now + std::chrono::milliseconds(500);
      }
      if (now >= handshake_deadline_) {
        close_serial(true);
        next_reconnect_ = now;
      }
      return;
    }

    if (now - last_rx_ > std::chrono::milliseconds(rx_timeout_ms_)) {
      close_serial(true);
      next_reconnect_ = now;
      return;
    }

    if (now >= next_status_) {
      if (!write_line("WINCH STATUS")) {
        close_serial(true);
        next_reconnect_ = now;
        return;
      }
      next_status_ = now + std::chrono::milliseconds(status_period_ms_);
    }

    if (hmi_sync_enabled_ && now >= next_hmi_sync_) {
      if (!publish_hmi_sync()) {
        close_serial(true);
        next_reconnect_ = now;
        return;
      }
      next_hmi_sync_ = now + std::chrono::milliseconds(hmi_sync_period_ms_);
    }
  }

  std::vector<std::string> candidates() const
  {
    if (!port_parameter_.empty() && port_parameter_ != "auto") {
      return {port_parameter_};
    }

    std::vector<std::string> result;
    if (std::filesystem::exists("/dev/winch")) {
      result.emplace_back("/dev/winch");
    }

    for (const auto & path : glob_paths("/dev/serial/by-id/*")) {
      const std::string name = std::filesystem::path(path).filename().string();
      if (contains_case_insensitive(name, "STM") ||
        contains_case_insensitive(name, "STMicro") ||
        contains_case_insensitive(name, "BlackPill") ||
        contains_case_insensitive(name, "WeAct") ||
        contains_case_insensitive(name, "CDC"))
      {
        result.push_back(path);
      }
    }

    const auto acm = glob_paths("/dev/ttyACM*");
    result.insert(result.end(), acm.begin(), acm.end());

    std::vector<std::string> unique;
    for (const auto & path : result) {
      if (std::find(unique.begin(), unique.end(), path) == unique.end()) {
        unique.push_back(path);
      }
    }
    return unique;
  }

  void try_connect()
  {
    const auto ports = candidates();
    if (ports.empty()) {
      return;
    }
    if (candidate_cursor_ >= ports.size()) {
      candidate_cursor_ = 0;
    }

    for (std::size_t attempt = 0; attempt < ports.size(); ++attempt) {
      const std::size_t index = (candidate_cursor_ + attempt) % ports.size();
      const auto & candidate = ports[index];
      const int fd = ::open(candidate.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      if (fd < 0) {
        continue;
      }

      if (::ioctl(fd, TIOCEXCL) < 0) {
        ::close(fd);
        continue;
      }

      termios tty{};
      if (::tcgetattr(fd, &tty) != 0) {
        ::close(fd);
        continue;
      }

      ::cfmakeraw(&tty);
      const speed_t serial_speed = serial_baud_ == 1000000 ? B1000000 : B115200;
      ::cfsetispeed(&tty, serial_speed);
      ::cfsetospeed(&tty, serial_speed);
      tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
      tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
      tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
      tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
      tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
      tty.c_cflag |= CS8;
      tty.c_cc[VMIN] = 0;
      tty.c_cc[VTIME] = 0;
      if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        ::close(fd);
        continue;
      }
      ::tcflush(fd, TCIOFLUSH);

      candidate_cursor_ = (index + 1) % ports.size();
      fd_ = fd;
      active_port_ = candidate;
      rx_buffer_.clear();
      handshake_confirmed_ = false;
      awaiting_host_session_ = true;
      const auto now = std::chrono::steady_clock::now();
      last_rx_ = now;
      const auto token64 = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
      host_session_token_ = static_cast<std::uint32_t>(token64 & 0xFFFFFFFFU);
      if (host_session_token_ == 0U) host_session_token_ = 1U;
      handshake_deadline_ = now + std::chrono::milliseconds(3500);
      next_host_hello_ = now + std::chrono::milliseconds(500);
      next_status_ = now + std::chrono::milliseconds(status_period_ms_);
      next_hmi_sync_ = now + std::chrono::milliseconds(hmi_sync_period_ms_);
      publish_connected(false);
      publish_string(port_pub_, active_port_);
      publish_string(state_pub_, "SYNCING_HOST_SESSION");

      // Current F411 firmware is session-bound: every normal command is rejected
      // until HOST:HELLO is acknowledged. Do not send legacy STATUS/STOP first.
      const char resync = '\n';
      const ssize_t resync_written = ::write(fd_, &resync, 1);
      if (resync_written != 1) {
        RCLCPP_WARN(this->get_logger(), "Failed to write serial resync byte on %s", active_port_.c_str());
      }
      (void)::tcdrain(fd_);
      if (!write_line("HOST:HELLO:" + std::to_string(host_session_token_))) {
        close_serial(true);
      }
      return;
    }
  }

  void close_serial(bool publish_state)
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    rx_buffer_.clear();
    active_port_.clear();
    handshake_confirmed_ = false;
    awaiting_host_session_ = false;
    host_session_token_ = 0U;
    host_transport_generation_ = 0U;
    publish_connected(false);
    publish_string(port_pub_, "-");
    if (publish_state) {
      publish_string(state_pub_, "DISCONNECTED");
    }
  }

  bool write_line(const std::string & command)
  {
    if (fd_ < 0) {
      return false;
    }
    const std::string frame = command + "\n";
    std::size_t offset = 0;
    while (offset < frame.size()) {
      const ssize_t written = ::write(fd_, frame.data() + offset, frame.size() - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        continue;
      }
      return false;
    }
    return true;
  }

  void read_serial()
  {
    char buffer[512];
    for (;;) {
      const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
      if (count > 0) {
        last_rx_ = std::chrono::steady_clock::now();
        rx_buffer_.append(buffer, static_cast<std::size_t>(count));
        consume_lines();
        continue;
      }
      if (count == 0) {
        return;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return;
      }
      close_serial(true);
      return;
    }
  }

  void consume_lines()
  {
    std::size_t newline = 0;
    while ((newline = rx_buffer_.find('\n')) != std::string::npos) {
      std::string line = trim_copy(rx_buffer_.substr(0, newline));
      rx_buffer_.erase(0, newline + 1);
      if (!line.empty()) {
        publish_string(raw_pub_, line);
        parse_line(line);
      }
    }
    if (rx_buffer_.size() > 4096) {
      rx_buffer_.erase(0, rx_buffer_.size() - 1024);
    }
  }

  void parse_line(const std::string & line)
  {
    try {
      if (awaiting_host_session_) {
        const std::string expected = "ACK:HOST:SESSION:" + std::to_string(host_session_token_) + ":";
        if (line.rfind(expected, 0) == 0) {
          const std::string generation_text = trim_copy(line.substr(expected.size()));
          std::size_t used = 0;
          const unsigned long generation = std::stoul(generation_text, &used, 10);
          if (used == generation_text.size() && generation > 0UL) {
            host_transport_generation_ = static_cast<std::uint32_t>(generation);
            awaiting_host_session_ = false;
            confirm_connection();
            // Reconnect is fail-safe: stop fork motion, then establish ROS/HMI state.
            (void)write_line("STOP");
            (void)write_line("ROS:1");
            (void)write_line("WINCH STATUS");
            (void)publish_hmi_sync();
          }
        }
        return;
      }
      if (line.rfind("WINCH:STATE=", 0) == 0) {
        const auto field = [&line](const std::string & key) -> std::string {
          const std::string marker = key + "=";
          const auto pos = line.find(marker);
          if (pos == std::string::npos) return {};
          const auto start = pos + marker.size();
          const auto end = line.find(':', start);
          return line.substr(start, end == std::string::npos ? std::string::npos : end - start);
        };
        const std::string state = field("STATE");
        if (!state.empty()) publish_string(state_pub_, state);
        const std::string pwm = field("APPLIED").empty() ? field("PWM") : field("APPLIED");
        if (!pwm.empty()) publish_double(pwm_pub_, std::stod(pwm));
        const std::string top = field("TOP");
        const std::string bottom = field("BOTTOM");
        if (!top.empty()) publish_bool(top_pub_, top == "1");
        if (!bottom.empty()) publish_bool(bottom_pub_, bottom == "1");
        if (!state.empty()) {
          std_msgs::msg::Int32 dir;
          const std::string u = upper_copy(state);
          dir.data = u.find("UP") != std::string::npos ? 1 : (u.find("DOWN") != std::string::npos ? -1 : 0);
          direction_pub_->publish(dir);
        }
      } else if (line.rfind("STATE:", 0) == 0) {
        confirm_connection();
        publish_string(state_pub_, trim_copy(line.substr(6)));
      } else if (line.rfind("TOP:", 0) == 0) {
        publish_bool(top_pub_, trim_copy(line.substr(4)) == "1");
      } else if (line.rfind("BOTTOM:", 0) == 0) {
        publish_bool(bottom_pub_, trim_copy(line.substr(7)) == "1");
      } else if (line.rfind("PWM:", 0) == 0) {
        publish_double(pwm_pub_, std::stod(trim_copy(line.substr(4))));
      } else if (line.rfind("DIR:", 0) == 0) {
        std_msgs::msg::Int32 msg;
        msg.data = std::stoi(trim_copy(line.substr(4)));
        direction_pub_->publish(msg);

      }
    } catch (const std::exception &) {
      // Firmware also emits human-readable diagnostic lines; malformed numeric
      // telemetry is ignored without turning normal serial chatter into warnings.
    }
  }

  void confirm_connection()
  {
    if (handshake_confirmed_) {
      return;
    }
    handshake_confirmed_ = true;
    publish_connected(true);
    publish_string(state_pub_, "CONNECTED");
  }

  bool send_f4x3(const char * domain, const char * group, std::uint32_t & seq,
    std::uint32_t age_ms, const std::string & payload)
  {
    if (!handshake_confirmed_ || host_session_token_ == 0U) return true;
    std::ostringstream body;
    body << "F4X3:" << domain << ':' << group << ":3:" << host_session_token_
         << ':' << ++seq << ':' << age_ms << ':' << payload.size() << ':' << payload;
    const std::string signed_part = body.str();
    std::ostringstream line;
    line << signed_part << ':' << std::uppercase << std::hex << std::setw(8)
         << std::setfill('0') << wire_crc32(signed_part);
    return write_line(line.str());
  }

  bool publish_hmi_sync()
  {
    if (fd_ < 0 || !handshake_confirmed_ || awaiting_host_session_) return true;
    const auto now = std::chrono::steady_clock::now();
    const auto age_ms = [now](const std::chrono::steady_clock::time_point &stamp)->std::uint32_t {
      if (stamp.time_since_epoch().count()==0) return 0xFFFFFFFFU;
      if (now <= stamp) return 0U;
      const auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(now-stamp).count();
      return static_cast<std::uint32_t>(std::min<std::int64_t>(ms,0xFFFFFFFELL));
    };
    const auto fresh=[&](const std::chrono::steady_clock::time_point &stamp,std::uint32_t limit){
      const auto a=age_ms(stamp); return a!=0xFFFFFFFFU && a<=limit;
    };
    const auto b=[](bool v){return v?1:0;};
    const auto number_line=[this](const char *key,double value,int precision=3){
      std::ostringstream out; out.setf(std::ios::fixed); out<<key<<std::setprecision(precision)<<value;
      return write_line(out.str());
    };

    const std::uint32_t imu_age=age_ms(last_imu_data_rx_);
    const std::uint32_t cam_age=std::min({
      age_ms(last_camera_frame_rx_), age_ms(last_camera_status_rx_),
      age_ms(last_camera_connected_rx_)});
    const std::uint32_t yolo_age=age_ms(last_yolo_rx_);

    // Keep TFT status aligned with the browser HMI: fresh measured telemetry is
    // authoritative, while connected/ready flags are supplemental diagnostics.
    const bool imu_online=fresh(last_imu_data_rx_,3000U);
    const bool camera_online=fresh(last_camera_frame_rx_,5000U) ||
      fresh(last_camera_status_rx_,8000U) ||
      (camera_connected_ && fresh(last_camera_connected_rx_,5000U));
    const bool camera_health_ok=!fresh(last_camera_healthy_rx_,8000U) || camera_healthy_;
    const bool perception_online=camera_online && camera_health_ok &&
      (!yolo_perf_seen_ || yolo_age<=4000U);
    const bool esc_online=esc_ready_ && fresh(last_esc_ready_rx_,4000U);
    const bool drive_online=drive_connected_ && fresh(last_drive_connected_rx_,4000U);
    const bool steer_online=steer_connected_ && fresh(last_steer_connected_rx_,4000U);
    const bool encoder_online=encoder_ready_ && fresh(last_encoder_ready_rx_,4000U);
    const bool motion_online=motion_ready_ && fresh(last_motion_ready_rx_,4000U);
    const bool pose_fresh=have_pose_ && fresh(last_pose_rx_,5000U);
    const bool nav2_online=(nav2_ready_ && fresh(last_nav2_ready_rx_,4000U)) ||
      fresh(last_planner_status_rx_,5000U);
    const bool localization_online=(localization_ready_ &&
      fresh(last_localization_ready_rx_,4000U)) || pose_fresh;
    const bool drive_value_fresh=fresh(last_drive_speed_rx_,3000U);
    const bool drive_target_fresh=fresh(last_drive_target_rx_,3000U);
    const bool steer_value_fresh=fresh(last_steering_rx_,3000U);
    const bool steer_target_fresh=fresh(last_steering_target_rx_,3000U);
    const bool goal_fresh=have_goal_ && fresh(last_goal_rx_,120000U);

    if (!write_line("ROS:1") ||
        !write_line(std::string("IMU:")+(imu_online?"1":"0")) ||
        !write_line(std::string("IMUSTATUS:")+(imu_online?"READY":"OFFLINE")) ||
        !write_line(std::string("CAM:")+(camera_online?"1":"0")) ||
        !write_line(std::string("PER:")+(perception_online?"1":"0")) ||
        !write_line(std::string("MOTION:")+(motion_online?"1":"0")) ||
        !write_line(std::string("NAV2:")+(nav2_online?"1":"0")) ||
        !write_line(std::string("ESC:")+(esc_online?"1":"0")) ||
        !write_line(std::string("VESC_LINK:")+((drive_online&&steer_online)?"1":"0")) ||
        !write_line(std::string("VESC_DRIVE:")+(drive_online?"1":"0")) ||
        !write_line(std::string("VESC_STEER:")+(steer_online?"1":"0")) ||
        !write_line(std::string("ENC:")+(encoder_online?"1":"0")) ||
        !write_line("FPS:"+std::to_string(static_cast<int>(std::clamp(std::lround(yolo_fps_),0L,255L))))) return false;

    if (drive_value_fresh && (!number_line("SPD:",drive_speed_mps_*3.6,2) || !number_line("DRIVE_ACT:",drive_speed_mps_,3))) return false;
    if (drive_target_fresh && !number_line("DRIVE_TGT:",drive_target_mps_,3)) return false;
    if (steer_value_fresh && !number_line("STEER_ACTUAL:",steering_rad_*180.0/M_PI,2)) return false;
    if (steer_target_fresh && !number_line("STEER_TARGET:",steering_target_rad_*180.0/M_PI,2)) return false;
    if (steer_value_fresh && steer_target_fresh && !number_line("STEER_ERR:",(steering_target_rad_-steering_rad_)*180.0/M_PI,2)) return false;
    if (pose_fresh && !number_line("HEAD:",heading_rad_*180.0/M_PI,2)) return false;
    if (pose_fresh && goal_fresh && !number_line("GOAL_DIST:",std::hypot(goal_x_-pose_x_,goal_y_-pose_y_),3)) return false;
    if (!write_line(std::string("LOCSTATE:")+(localization_online?"READY":"WAIT"))) return false;

    std::string nav="IDLE"; const std::string gs=upper_copy(goal_state_);
    if (gs.find("SUCCEEDED")!=std::string::npos || gs.find("ARRIVED")!=std::string::npos) nav="ARRIVED";
    else if (gs.find("ABORT")!=std::string::npos || gs.find("FAIL")!=std::string::npos || gs.find("REJECT")!=std::string::npos) nav="FAILED";
    else if (gs.find("CANCEL")!=std::string::npos || gs.find("STOP")!=std::string::npos) nav="STOPPED";
    else if (gs.find("ACTIVE")!=std::string::npos || gs.find("NAVIGAT")!=std::string::npos) nav="NAVIGATING";
    if (!write_line("NAV:"+nav) || !write_line(std::string("TARGET:")+(goal_fresh?"NAV GOAL":"NONE")) ||
        !write_line(std::string("OBS:")+(perception_online && yolo_detections_>0?"1":"0")) ||
        !write_line(std::string("OBJ:")+(perception_online && yolo_detections_>0?"DETECTED":"CLEAR"))) return false;

    { std::ostringstream payload; payload.setf(std::ios::fixed);
      payload<<b(camera_online)<<','<<b(perception_online)<<','<<std::setprecision(2)<<yolo_fps_<<','<<yolo_inference_ms_<<",0,0,ROS,"<<b(yolo_perf_seen_);
      if (!send_f4x3("PER","CAM",per_cam_seq_,cam_age,payload.str())) return false; }
    { const double yaw=imu_yaw_rad_*180.0/M_PI; std::ostringstream payload; payload.setf(std::ios::fixed);
      payload<<b(imu_online)<<','<<std::setprecision(2)<<yaw<<','<<yaw<<",0.00";
      if (!send_f4x3("NAV","IMU",nav_imu_seq_,imu_age,payload.str())) return false; }
    { std::ostringstream payload; payload.setf(std::ios::fixed);
      payload<<b(nav2_online||localization_online)<<','<<b(nav2_online)<<",0,0,0.000,0.000,"<<(nav2_online?"READY":"WAIT")<<','<<(nav2_online?"READY":"WAIT")<<",WAIT";
      if (!send_f4x3("NAV","NAV2",nav_nav2_seq_,0U,payload.str())) return false; }
    return true;
  }

  void handle_command(const std::string & input)
  {
    const std::string command = upper_copy(trim_copy(input));
    if (!valid_command(command)) {
      return;
    }
    if (fd_ >= 0 && (handshake_confirmed_ || command == "STOP") && !write_line(command)) {
      close_serial(true);
      next_reconnect_ = std::chrono::steady_clock::now();
    }
  }

  static bool valid_command(const std::string & command)
  {
    if (command == "UP" || command == "DOWN" || command == "STOP" ||
      command == "UP HOME" || command == "UP 1" || command == "UP 2" ||
      command == "DOWN HOME" || command == "DOWN 1" || command == "DOWN 2" ||
      command == "STATUS" || command == "WINCH STATUS" || command == "LIMITS" || command == "CONFIG")
    {
      return true;
    }
    return false;
  }

  void publish_connected(bool connected)
  {
    if (connected_state_.has_value() && connected_state_.value() == connected) {
      return;
    }
    connected_state_ = connected;
    publish_bool(connected_pub_, connected);
  }

  static void publish_bool(
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher, bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    publisher->publish(msg);
  }

  static void publish_double(
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & publisher, double value)
  {
    std_msgs::msg::Float64 msg;
    msg.data = value;
    publisher->publish(msg);
  }

  static void publish_string(
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & publisher,
    const std::string & value)
  {
    std_msgs::msg::String msg;
    msg.data = value;
    publisher->publish(msg);
  }

  std::string port_parameter_;
  std::int64_t serial_baud_{1000000};
  std::int64_t reconnect_interval_ms_{1000};
  std::int64_t status_period_ms_{1000};
  std::int64_t rx_timeout_ms_{5000};
  bool hmi_sync_enabled_{true};
  std::int64_t hmi_sync_period_ms_{500};

  bool esc_ready_{false};
  bool drive_connected_{false};
  bool steer_connected_{false};
  bool encoder_ready_{false};
  bool motion_ready_{false};
  bool nav2_ready_{false};
  bool localization_ready_{false};
  bool imu_connected_{false};
  bool camera_connected_{false};
  bool camera_healthy_{false};
  bool yolo_perf_seen_{false};
  double drive_speed_mps_{0.0};
  double drive_target_mps_{0.0};
  double steering_rad_{0.0};
  double steering_target_rad_{0.0};
  double imu_yaw_rad_{0.0};
  double yolo_fps_{0.0};
  double yolo_inference_ms_{0.0};
  int yolo_detections_{0};
  int person_count_{0};
  std::string goal_state_;
  std::string planner_status_;
  bool have_pose_{false};
  bool have_goal_{false};
  double pose_x_{0.0};
  double pose_y_{0.0};
  double heading_rad_{0.0};
  double goal_x_{0.0};
  double goal_y_{0.0};

  int fd_{-1};
  std::string active_port_;
  std::string rx_buffer_;
  std::optional<bool> connected_state_;
  bool handshake_confirmed_{false};
  bool awaiting_host_session_{false};
  std::uint32_t host_session_token_{0U};
  std::uint32_t host_transport_generation_{0U};
  std::uint32_t per_cam_seq_{0U};
  std::uint32_t nav_imu_seq_{0U};
  std::uint32_t nav_nav2_seq_{0U};
  std::size_t candidate_cursor_{0};

  std::chrono::steady_clock::time_point next_reconnect_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point handshake_deadline_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point next_host_hello_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point next_status_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point next_hmi_sync_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_rx_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_imu_data_rx_{};
  std::chrono::steady_clock::time_point last_camera_connected_rx_{};
  std::chrono::steady_clock::time_point last_camera_healthy_rx_{};
  std::chrono::steady_clock::time_point last_camera_frame_rx_{};
  std::chrono::steady_clock::time_point last_camera_status_rx_{};
  std::chrono::steady_clock::time_point last_yolo_rx_{};
  std::chrono::steady_clock::time_point last_esc_ready_rx_{};
  std::chrono::steady_clock::time_point last_drive_connected_rx_{};
  std::chrono::steady_clock::time_point last_steer_connected_rx_{};
  std::chrono::steady_clock::time_point last_encoder_ready_rx_{};
  std::chrono::steady_clock::time_point last_motion_ready_rx_{};
  std::chrono::steady_clock::time_point last_nav2_ready_rx_{};
  std::chrono::steady_clock::time_point last_localization_ready_rx_{};
  std::chrono::steady_clock::time_point last_drive_speed_rx_{};
  std::chrono::steady_clock::time_point last_drive_target_rx_{};
  std::chrono::steady_clock::time_point last_steering_rx_{};
  std::chrono::steady_clock::time_point last_steering_target_rx_{};
  std::chrono::steady_clock::time_point last_goal_state_rx_{};
  std::chrono::steady_clock::time_point last_planner_status_rx_{};
  std::chrono::steady_clock::time_point last_pose_rx_{};
  std::chrono::steady_clock::time_point last_goal_rx_{};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr esc_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_connected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr steer_connected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr encoder_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav2_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_ready_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr imu_connected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_connected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_healthy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_data_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_frame_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drive_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drive_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_perf_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr person_perf_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr port_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr top_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bottom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pwm_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr direction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WinchSerialNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("winch_serial_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
