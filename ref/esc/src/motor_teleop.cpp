#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <functional>
#include <fcntl.h>
#include <iomanip>
#include <linux/input.h>
#include <limits.h>
#include <memory>
#include <mutex>
#include <poll.h>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr int kBitsPerLong = static_cast<int>(sizeof(unsigned long) * 8U);

struct ControlFractions {
  double forward{0.0};
  double yaw{0.0};
};

double clamp_value(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

bool almost_zero(double value, double eps = 1e-6) {
  return std::abs(value) <= eps;
}

std::string real_path(const std::string &path) {
  char buffer[PATH_MAX];
  return ::realpath(path.c_str(), buffer) ? std::string(buffer) : path;
}

std::string lowercase_copy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

bool bit_is_set(const std::vector<unsigned long> &bits, int bit) {
  if (bit < 0) return false;
  const size_t index = static_cast<size_t>(bit / kBitsPerLong);
  if (index >= bits.size()) return false;
  return (bits[index] & (1UL << (bit % kBitsPerLong))) != 0;
}

bool read_event_bits(int fd, int event_type, int max_bit, std::vector<unsigned long> &bits) {
  bits.assign(static_cast<size_t>((max_bit + kBitsPerLong) / kBitsPerLong), 0UL);
  return ::ioctl(fd, EVIOCGBIT(event_type, bits.size() * sizeof(unsigned long)), bits.data()) >= 0;
}

std::string input_device_name(int fd) {
  char name[256] = {};
  if (::ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) return "unknown";
  return std::string(name);
}

bool input_device_id(int fd, input_id &id) {
  std::memset(&id, 0, sizeof(id));
  return ::ioctl(fd, EVIOCGID, &id) >= 0;
}

bool input_fd_alive(int fd) {
  input_id id{};
  return input_device_id(fd, id);
}

bool get_abs_info(int fd, int code, input_absinfo &info) {
  std::memset(&info, 0, sizeof(info));
  return code >= 0 && code <= ABS_MAX && ::ioctl(fd, EVIOCGABS(code), &info) >= 0;
}

bool fd_supports_abs(int fd, int code) {
  std::vector<unsigned long> bits;
  return read_event_bits(fd, EV_ABS, ABS_MAX, bits) && bit_is_set(bits, code);
}

bool fd_looks_like_keyboard(int fd) {
  std::vector<unsigned long> key_bits;
  if (!read_event_bits(fd, EV_KEY, KEY_MAX, key_bits)) return false;
  return bit_is_set(key_bits, KEY_W) && bit_is_set(key_bits, KEY_A) &&
         bit_is_set(key_bits, KEY_S) && bit_is_set(key_bits, KEY_D);
}

bool name_looks_like_game_controller(const std::string &name) {
  const std::string n = lowercase_copy(name);
  static constexpr const char *kTokens[] = {
      "gamepad", "joystick", "controller", "xbox", "dualshock", "dualsense",
      "playstation", "shanwan", "asteria", "daxa", "rexus", "tyx"};
  for (const char *token : kTokens) {
    if (n.find(token) != std::string::npos) return true;
  }
  return false;
}

const char *abs_code_name(int code) {
  switch (code) {
    case ABS_X: return "ABS_X";
    case ABS_Y: return "ABS_Y";
    case ABS_Z: return "ABS_Z";
    case ABS_RX: return "ABS_RX";
    case ABS_RY: return "ABS_RY";
    case ABS_RZ: return "ABS_RZ";
    case ABS_HAT0X: return "ABS_HAT0X";
    case ABS_HAT0Y: return "ABS_HAT0Y";
    case ABS_GAS: return "ABS_GAS";
    case ABS_BRAKE: return "ABS_BRAKE";
    default: return "ABS_UNKNOWN";
  }
}

std::string hex4(unsigned int value) {
  std::ostringstream oss;
  oss << std::hex << std::setfill('0') << std::setw(4) << (value & 0xffffU);
  return oss.str();
}

class TerminalEchoGuard {
public:
  bool enable() {
    if (enabled_) return true;
    fd_ = ::open("/dev/tty", O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK);
    if (fd_ < 0) return false;
    if (::tcgetattr(fd_, &saved_) != 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    termios silent = saved_;
    silent.c_lflag &= static_cast<tcflag_t>(~(ECHO | ECHONL));
    if (::tcsetattr(fd_, TCSANOW, &silent) != 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    enabled_ = true;
    return true;
  }

  void restore() {
    if (!enabled_ || fd_ < 0) return;
    ::tcflush(fd_, TCIFLUSH);
    ::tcsetattr(fd_, TCSANOW, &saved_);
    ::close(fd_);
    fd_ = -1;
    enabled_ = false;
  }

  ~TerminalEchoGuard() { restore(); }

private:
  int fd_{-1};
  bool enabled_{false};
  termios saved_{};
};

}  // namespace

class MotorTeleop final : public rclcpp::Node {
public:
  MotorTeleop() : Node("motor_teleop") {
    declare_all_parameters();
    load_parameters();

    if (suppress_terminal_echo_) terminal_echo_suppressed_ = terminal_guard_.enable();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    source_pub_ = create_publisher<std_msgs::msg::String>("/teleop/active_source", 10);
    limits_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/teleop/limits", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    estop_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/teleop/emergency_stop_latched",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    running_.store(true);
    keyboard_thread_ = std::thread(&MotorTeleop::keyboard_loop, this);
    gamepad_thread_ = std::thread(&MotorTeleop::gamepad_loop, this);


    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)),
        std::bind(&MotorTeleop::publish_command, this));

    publish_limits();
    publish_estop_state();

    RCLCPP_INFO(get_logger(), "[ESC] teleop ready | L1/L2 speed control + direct steering input enabled");
    RCLCPP_INFO(get_logger(),
                "[TELEOP] DIRECT_EVDEV_STATUS | output=%s | publish=%.1f Hz",
                output_topic_.c_str(), publish_rate_hz_);
    RCLCPP_INFO(get_logger(),
                "[TELEOP] Launch tidak menjalankan joy_node/joy_linux_node; motor_teleop membaca gamepad secara langsung.");
    RCLCPP_INFO(get_logger(),
                "[KEY] GLOBAL evdev | W/S maju-mundur | A/D kiri-kanan | Q/E speed -/+ | F/R yaw -/+ | SPACE/ESC E-STOP | ENTER reset");
    RCLCPP_INFO(get_logger(),
                "[JOY] DIRECT evdev (tanpa joy_node/joy_linux) | old axis[1]=>ABS_Y | old axis[2]=>AUTO ABS_Z/ABS_RX");
    RCLCPP_INFO(get_logger(),
                "[JOY] L1=BTN_TL speed+ | L2=BTN_TL2 speed- | R1=BTN_TR yaw+ | R2=BTN_TR2 yaw-");
    RCLCPP_INFO(get_logger(),
                "[JOY] Axis shaping: forward_deadzone=%.2f forward_full=%.2f | yaw_deadzone=%.2f yaw_full=%.2f",
                joy_forward_deadzone_, joy_forward_full_scale_threshold_,
                joy_yaw_deadzone_, joy_yaw_full_scale_threshold_);
    RCLCPP_INFO(get_logger(),
                "[DRIVE] Teleop speed startup=%.2f m/s max=%.2f m/s | L1 +%.2f m/s (~%.0f RPM/step) | L2 -step | accel=%.2f decel=%.2f m/s^2 smoothing=%s",
                max_speed_, speed_max_, speed_step_,
                speed_max_ > 1.0e-9 ? (speed_step_ / speed_max_) * 300.0 : 0.0,
                teleop_accel_limit_mps2_, teleop_decel_limit_mps2_,
                teleop_velocity_smoothing_enabled_ ? "ON" : "OFF");
    RCLCPP_INFO(get_logger(),
                "[JOY] Auto-detect Rexus/Daxa/Asteria/ShanWan/XInput + hotplug reconnect %.2f s",
                gamepad_rescan_sec_);
    RCLCPP_INFO(get_logger(),
                "[MIX] Keyboard + joystick dijumlahkan sebagai fraction lalu clamp [-1,+1].");
    RCLCPP_INFO(get_logger(), "[TELEOP] Twist-only: UART dimiliki eksklusif oleh esc_ackermann.");
    if (suppress_terminal_echo_) {
      if (terminal_echo_suppressed_) {
        RCLCPP_INFO(get_logger(), "[TTY] Echo terminal OFF selama teleop aktif.");
      } else {
        RCLCPP_WARN(get_logger(), "[TTY] /dev/tty tidak dapat dibuka; global keyboard tetap aktif.");
      }
    }
  }

  ~MotorTeleop() override {
    stop_input_threads();
    terminal_guard_.restore();
  }

  void publish_stop_burst() {
    geometry_msgs::msg::Twist stop;
    std_msgs::msg::String source;
    source.data = "STOP";
    for (int i = 0; i < 3; ++i) {
      if (rclcpp::ok()) {
        cmd_pub_->publish(stop);
        source_pub_->publish(source);
      }
      std::this_thread::sleep_for(20ms);
    }
    stop_input_threads();
  }

private:
  struct KeyboardDevice {
    int fd{-1};
    std::string path;
    std::string name;
    std::set<uint16_t> pressed;
  };

  struct GamepadCandidate {
    std::string path;
    std::string canonical;
    std::string name;
    input_id id{};
    int score{0};
    int forward_code{ABS_Y};
    int yaw_code{-1};
    input_absinfo forward_info{};
    input_absinfo yaw_info{};
  };

  struct GamepadDevice {
    int fd{-1};
    GamepadCandidate candidate;
    double forward_center{0.0};
    double yaw_center{0.0};
    bool ready{false};
    bool wait_center_logged{false};
    std::unordered_map<uint16_t, bool> button_down;
  };

  void declare_all_parameters() {
    declare_parameter<std::string>("output_topic", "/cmd_vel/teleop");
    declare_parameter<double>("publish_rate_hz", 50.0);

    declare_parameter<double>("speed_initial", 0.20);
    declare_parameter<double>("speed_step", 0.10);
    declare_parameter<double>("speed_min", 0.10);
    declare_parameter<double>("speed_max", 1.00);

    declare_parameter<double>("yaw_initial_deg_s", 80.0);
    declare_parameter<double>("yaw_step_deg_s", 5.0);
    declare_parameter<double>("yaw_min_deg_s", 5.0);
    declare_parameter<double>("yaw_max_deg_s", 80.0);

    declare_parameter<std::string>("keyboard_device", "");
    declare_parameter<double>("keyboard_rescan_sec", 2.0);
    declare_parameter<bool>("ignore_controller_keyboard_interfaces", true);
    declare_parameter<bool>("suppress_terminal_echo", true);

    // Empty = auto-scan /dev/input/by-id/*-event-joystick lalu event*.
    declare_parameter<std::string>("gamepad_device", "");
    declare_parameter<double>("gamepad_rescan_sec", 0.10);
    declare_parameter<int>("gamepad_forward_abs_code", ABS_Y);
    declare_parameter<int>("gamepad_yaw_abs_code", -1);  // -1 = auto ABS_Z/ABS_RX
    declare_parameter<bool>("gamepad_invert_forward", true);
    declare_parameter<bool>("gamepad_invert_yaw", false);
    declare_parameter<double>("joy_deadzone", 0.12);
    declare_parameter<double>("joy_forward_deadzone", 0.12);
    declare_parameter<double>("joy_forward_full_scale_threshold", 0.90);
    declare_parameter<bool>("teleop_velocity_smoothing_enabled", true);
    declare_parameter<double>("teleop_accel_limit_mps2", 1.50);
    declare_parameter<double>("teleop_decel_limit_mps2", 2.50);
    declare_parameter<double>("teleop_zero_snap_mps", 0.015);
    // Steering stick gets its own slightly wider center deadzone. Unlike the
    // legacy implementation, values outside this deadzone are re-scaled back
    // to 0..1, so there is no ~10 deg jump immediately after leaving center.
    declare_parameter<double>("joy_yaw_deadzone", 0.18);
    // Physical gamepads often stop short of the kernel-reported ABS endpoint.
    // Treat this normalized magnitude as full stick and continuously rescale
    // the useful range from deadzone..threshold to 0..1.
    declare_parameter<double>("joy_yaw_full_scale_threshold", 0.90);
    declare_parameter<double>("joy_axis_init_tolerance", 0.35);
    declare_parameter<bool>("joy_auto_center", true);
    declare_parameter<double>("joy_auto_center_max_abs", 0.08);
    declare_parameter<double>("button_debounce_sec", 0.20);

    declare_parameter<bool>("log_command_changes", true);
  }

  void load_parameters() {
    output_topic_ = get_parameter("output_topic").as_string();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();

    max_speed_ = get_parameter("speed_initial").as_double();
    speed_step_ = get_parameter("speed_step").as_double();
    speed_min_ = get_parameter("speed_min").as_double();
    speed_max_ = get_parameter("speed_max").as_double();

    max_yaw_deg_s_ = get_parameter("yaw_initial_deg_s").as_double();
    yaw_step_deg_s_ = get_parameter("yaw_step_deg_s").as_double();
    yaw_min_deg_s_ = get_parameter("yaw_min_deg_s").as_double();
    yaw_max_deg_s_ = get_parameter("yaw_max_deg_s").as_double();

    keyboard_device_ = get_parameter("keyboard_device").as_string();
    keyboard_rescan_sec_ = get_parameter("keyboard_rescan_sec").as_double();
    ignore_controller_keyboard_interfaces_ =
        get_parameter("ignore_controller_keyboard_interfaces").as_bool();
    suppress_terminal_echo_ = get_parameter("suppress_terminal_echo").as_bool();

    gamepad_device_ = get_parameter("gamepad_device").as_string();
    gamepad_rescan_sec_ = get_parameter("gamepad_rescan_sec").as_double();
    gamepad_forward_abs_code_ = static_cast<int>(get_parameter("gamepad_forward_abs_code").as_int());
    gamepad_yaw_abs_code_ = static_cast<int>(get_parameter("gamepad_yaw_abs_code").as_int());
    gamepad_invert_forward_ = get_parameter("gamepad_invert_forward").as_bool();
    gamepad_invert_yaw_ = get_parameter("gamepad_invert_yaw").as_bool();
    joy_deadzone_ = get_parameter("joy_deadzone").as_double();
    joy_forward_deadzone_ = get_parameter("joy_forward_deadzone").as_double();
    joy_forward_full_scale_threshold_ = get_parameter("joy_forward_full_scale_threshold").as_double();
    teleop_velocity_smoothing_enabled_ = get_parameter("teleop_velocity_smoothing_enabled").as_bool();
    teleop_accel_limit_mps2_ = get_parameter("teleop_accel_limit_mps2").as_double();
    teleop_decel_limit_mps2_ = get_parameter("teleop_decel_limit_mps2").as_double();
    teleop_zero_snap_mps_ = get_parameter("teleop_zero_snap_mps").as_double();
    joy_yaw_deadzone_ = get_parameter("joy_yaw_deadzone").as_double();
    joy_yaw_full_scale_threshold_ = get_parameter("joy_yaw_full_scale_threshold").as_double();
    joy_axis_init_tolerance_ = get_parameter("joy_axis_init_tolerance").as_double();
    joy_auto_center_ = get_parameter("joy_auto_center").as_bool();
    joy_auto_center_max_abs_ = get_parameter("joy_auto_center_max_abs").as_double();
    button_debounce_sec_ = get_parameter("button_debounce_sec").as_double();

    log_command_changes_ = get_parameter("log_command_changes").as_bool();

    if (publish_rate_hz_ <= 0.0) throw std::runtime_error("publish_rate_hz harus > 0");
    if (keyboard_rescan_sec_ <= 0.0) throw std::runtime_error("keyboard_rescan_sec harus > 0");
    if (gamepad_rescan_sec_ <= 0.0) throw std::runtime_error("gamepad_rescan_sec harus > 0");
    if (joy_deadzone_ < 0.0 || joy_deadzone_ >= 1.0) throw std::runtime_error("joy_deadzone harus 0 <= x < 1");
    if (joy_forward_deadzone_ < 0.0 || joy_forward_deadzone_ >= 1.0) {
      throw std::runtime_error("joy_forward_deadzone harus 0 <= x < 1");
    }
    if (joy_forward_full_scale_threshold_ <= joy_forward_deadzone_ ||
        joy_forward_full_scale_threshold_ > 1.0) {
      throw std::runtime_error("joy_forward_full_scale_threshold harus > joy_forward_deadzone dan <= 1");
    }
    if (teleop_accel_limit_mps2_ <= 0.0 || teleop_decel_limit_mps2_ <= 0.0) {
      throw std::runtime_error("teleop accel/decel limit harus > 0");
    }
    if (teleop_zero_snap_mps_ < 0.0 || teleop_zero_snap_mps_ > speed_max_) {
      throw std::runtime_error("teleop_zero_snap_mps invalid");
    }
    if (joy_yaw_deadzone_ < 0.0 || joy_yaw_deadzone_ >= 1.0) throw std::runtime_error("joy_yaw_deadzone harus 0 <= x < 1");
    if (joy_yaw_full_scale_threshold_ <= joy_yaw_deadzone_ || joy_yaw_full_scale_threshold_ > 1.0) {
      throw std::runtime_error("joy_yaw_full_scale_threshold harus > joy_yaw_deadzone dan <= 1");
    }
    if (joy_axis_init_tolerance_ <= 0.0 || joy_axis_init_tolerance_ >= 1.0) throw std::runtime_error("joy_axis_init_tolerance harus 0 < x < 1");
    if (joy_auto_center_max_abs_ < 0.0 || joy_auto_center_max_abs_ > joy_axis_init_tolerance_) {
      throw std::runtime_error("joy_auto_center_max_abs harus <= joy_axis_init_tolerance");
    }
    if (button_debounce_sec_ < 0.0) throw std::runtime_error("button_debounce_sec tidak boleh negatif");
    if (gamepad_forward_abs_code_ < 0 || gamepad_forward_abs_code_ > ABS_MAX) {
      throw std::runtime_error("gamepad_forward_abs_code invalid");
    }
    if (gamepad_yaw_abs_code_ > ABS_MAX) throw std::runtime_error("gamepad_yaw_abs_code invalid");

    if (!(0.0 <= speed_min_ && speed_min_ <= max_speed_ && max_speed_ <= speed_max_)) {
      throw std::runtime_error("speed_min <= speed_initial <= speed_max harus terpenuhi");
    }
    if (!(0.0 <= yaw_min_deg_s_ && yaw_min_deg_s_ <= max_yaw_deg_s_ && max_yaw_deg_s_ <= yaw_max_deg_s_)) {
      throw std::runtime_error("yaw_min <= yaw_initial <= yaw_max harus terpenuhi");
    }
    if (speed_step_ <= 0.0 || yaw_step_deg_s_ <= 0.0) {
      throw std::runtime_error("speed_step dan yaw_step_deg_s harus > 0");
    }
  }

  // ----------------------------- Keyboard -----------------------------
  std::vector<std::string> keyboard_candidates() const {
    if (!keyboard_device_.empty()) return {keyboard_device_};
    std::vector<std::string> paths;
    std::error_code ec;
    if (!fs::exists("/dev/input", ec)) return paths;
    for (const auto &entry : fs::directory_iterator("/dev/input", ec)) {
      const std::string file = entry.path().filename().string();
      if (file.rfind("event", 0) == 0) paths.push_back(entry.path().string());
    }
    std::sort(paths.begin(), paths.end());
    return paths;
  }

  void refresh_keyboard_devices(std::vector<KeyboardDevice> &devices) {
    std::set<std::string> already;
    for (const auto &dev : devices) already.insert(real_path(dev.path));

    int permission_errors = 0;
    for (const auto &path : keyboard_candidates()) {
      const std::string canonical = real_path(path);
      if (already.count(canonical)) continue;

      const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
      if (fd < 0) {
        if (errno == EACCES || errno == EPERM) ++permission_errors;
        continue;
      }
      if (!fd_looks_like_keyboard(fd)) {
        ::close(fd);
        continue;
      }

      const std::string detected_name = input_device_name(fd);
      const bool explicit_device = !keyboard_device_.empty();
      if (!explicit_device && ignore_controller_keyboard_interfaces_ &&
          name_looks_like_game_controller(detected_name)) {
        ::close(fd);
        continue;
      }

      KeyboardDevice device;
      device.fd = fd;
      device.path = path;
      device.name = detected_name;
      devices.push_back(std::move(device));
      already.insert(canonical);
      RCLCPP_INFO(get_logger(), "[KEY] Global keyboard tersambung: %s (%s)",
                  path.c_str(), devices.back().name.c_str());
    }

    const bool now_connected = !devices.empty();
    if (!keyboard_status_initialized_ || now_connected != keyboard_connected_reported_) {
      keyboard_status_initialized_ = true;
      keyboard_connected_reported_ = now_connected;
      if (!now_connected) {
        if (permission_errors > 0) {
          RCLCPP_INFO(get_logger(),
                      "[KEY] /dev/input tidak bisa dibaca; keyboard evdev dinonaktifkan (opsional). Tambahkan user ke group input jika keyboard global diperlukan.");
        } else {
          RCLCPP_INFO(get_logger(), "[KEY] Belum menemukan keyboard; joystick tetap dapat digunakan.");
        }
      }
    }
  }

  void keyboard_loop() {
    std::vector<KeyboardDevice> devices;
    auto next_scan = std::chrono::steady_clock::now();

    while (running_.load() && rclcpp::ok()) {
      const auto now_steady = std::chrono::steady_clock::now();
      if (now_steady >= next_scan) {
        refresh_keyboard_devices(devices);
        next_scan = now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    std::chrono::duration<double>(keyboard_rescan_sec_));
      }

      if (devices.empty()) {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      std::vector<pollfd> pfds;
      pfds.reserve(devices.size());
      for (const auto &dev : devices) pfds.push_back(pollfd{dev.fd, POLLIN, 0});

      const int pr = ::poll(pfds.data(), pfds.size(), 100);
      if (pr < 0) {
        if (errno == EINTR) continue;
        RCLCPP_WARN(get_logger(), "[KEY] poll error: %s", std::strerror(errno));
        std::this_thread::sleep_for(100ms);
        continue;
      }

      for (size_t i = devices.size(); i-- > 0;) {
        bool remove_device = false;
        if (pfds[i].revents & (POLLERR | POLLHUP | POLLNVAL)) {
          // Beberapa composite HID memberi HUP transient. Verifikasi dengan ioctl
          // sebelum menganggap keyboard benar-benar hilang.
          remove_device = !input_fd_alive(devices[i].fd);
        }
        if (!remove_device && (pfds[i].revents & POLLIN)) {
          input_event events[32];
          const ssize_t bytes = ::read(devices[i].fd, events, sizeof(events));
          if (bytes == 0) {
            remove_device = !input_fd_alive(devices[i].fd);
          } else if (bytes < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
              remove_device = !input_fd_alive(devices[i].fd);
            }
          } else {
            const size_t count = static_cast<size_t>(bytes) / sizeof(input_event);
            for (size_t e = 0; e < count; ++e) {
              if (events[e].type == EV_KEY) {
                handle_keyboard_event(devices[i], events[e].code, events[e].value);
              }
            }
          }
        }

        if (remove_device) {
          release_all_from_device(devices[i]);
          RCLCPP_WARN(get_logger(), "[KEY] Keyboard terputus: %s", devices[i].path.c_str());
          ::close(devices[i].fd);
          devices.erase(devices.begin() + static_cast<std::ptrdiff_t>(i));
          next_scan = std::chrono::steady_clock::now();
        }
      }
    }

    for (auto &dev : devices) {
      release_all_from_device(dev);
      if (dev.fd >= 0) ::close(dev.fd);
    }
  }

  static bool is_motion_key(uint16_t code) {
    return code == KEY_W || code == KEY_S || code == KEY_A || code == KEY_D;
  }

  static bool is_recognized_key(uint16_t code) {
    return is_motion_key(code) || code == KEY_Q || code == KEY_E ||
           code == KEY_F || code == KEY_R || code == KEY_SPACE ||
           code == KEY_ESC || code == KEY_ENTER;
  }

  void handle_keyboard_event(KeyboardDevice &device, uint16_t code, int32_t value) {
    if (!is_recognized_key(code)) return;

    bool first_press = false;
    if (value == 1) {
      first_press = device.pressed.insert(code).second;
      if (first_press && is_motion_key(code)) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++key_press_count_[code];
      }
    } else if (value == 0) {
      const bool was_pressed = device.pressed.erase(code) > 0;
      if (was_pressed && is_motion_key(code)) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        auto it = key_press_count_.find(code);
        if (it != key_press_count_.end() && --it->second <= 0) key_press_count_.erase(it);
      }
      return;
    } else {
      return;  // value=2 repeat tidak mengulang Q/E/F/R.
    }

    if (!first_press) return;
    if (code == KEY_Q) adjust_limit("speed_down", "KEY Q");
    else if (code == KEY_E) adjust_limit("speed_up", "KEY E");
    else if (code == KEY_F) adjust_limit("yaw_down", "KEY F");
    else if (code == KEY_R) adjust_limit("yaw_up", "KEY R");
    else if (code == KEY_SPACE || code == KEY_ESC) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!estop_latched_) {
        estop_latched_ = true;
        RCLCPP_ERROR(get_logger(), "[E-STOP] LATCHED dari keyboard.");
      }
    } else if (code == KEY_ENTER) {
      estop_reset_requested_.store(true);
    }
  }

  void release_all_from_device(KeyboardDevice &device) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (uint16_t code : device.pressed) {
      if (!is_motion_key(code)) continue;
      auto it = key_press_count_.find(code);
      if (it != key_press_count_.end() && --it->second <= 0) key_press_count_.erase(it);
    }
    device.pressed.clear();
  }

  bool key_pressed_locked(uint16_t code) const {
    const auto it = key_press_count_.find(code);
    return it != key_press_count_.end() && it->second > 0;
  }

  ControlFractions keyboard_fractions() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const bool w = key_pressed_locked(KEY_W);
    const bool s = key_pressed_locked(KEY_S);
    const bool a = key_pressed_locked(KEY_A);
    const bool d = key_pressed_locked(KEY_D);
    const double forward = (w == s) ? 0.0 : (w ? 1.0 : -1.0);
    const double yaw = (a == d) ? 0.0 : (a ? -1.0 : 1.0);
    return {forward, yaw};
  }

  // ----------------------------- Gamepad evdev -----------------------------
  std::vector<std::string> gamepad_candidates() const {
    if (!gamepad_device_.empty()) return {gamepad_device_};

    std::vector<std::string> result;
    std::set<std::string> canonical_seen;
    std::error_code ec;

    // Stable symlink lebih disukai daripada eventN karena eventN berubah saat hotplug.
    if (fs::exists("/dev/input/by-id", ec)) {
      for (const auto &entry : fs::directory_iterator("/dev/input/by-id", ec)) {
        const std::string file = entry.path().filename().string();
        if (file.find("event-joystick") == std::string::npos) continue;
        const std::string p = entry.path().string();
        const std::string c = real_path(p);
        if (canonical_seen.insert(c).second) result.push_back(p);
      }
    }

    if (fs::exists("/dev/input", ec)) {
      std::vector<std::string> events;
      for (const auto &entry : fs::directory_iterator("/dev/input", ec)) {
        const std::string file = entry.path().filename().string();
        if (file.rfind("event", 0) == 0) events.push_back(entry.path().string());
      }
      std::sort(events.begin(), events.end());
      for (const auto &p : events) {
        const std::string c = real_path(p);
        if (canonical_seen.insert(c).second) result.push_back(p);
      }
    }
    return result;
  }

  static bool centered_axis(const input_absinfo &info, double tolerance = 0.55) {
    const double range = static_cast<double>(info.maximum) - static_cast<double>(info.minimum);
    if (range <= 1.0) return false;
    const double mid = (static_cast<double>(info.maximum) + static_cast<double>(info.minimum)) * 0.5;
    const double half = range * 0.5;
    return std::abs(static_cast<double>(info.value) - mid) / half <= tolerance;
  }

  int choose_yaw_axis(int fd) const {
    if (gamepad_yaw_abs_code_ >= 0) {
      return fd_supports_abs(fd, gamepad_yaw_abs_code_) ? gamepad_yaw_abs_code_ : -1;
    }

    // AX1/ShanWan Android mode umumnya right-stick X = ABS_Z.
    // XInput/standard Linux umumnya right-stick X = ABS_RX.
    const int preference[] = {ABS_Z, ABS_RX, ABS_RZ, ABS_RY};
    int fallback = -1;
    for (int code : preference) {
      if (code == gamepad_forward_abs_code_ || !fd_supports_abs(fd, code)) continue;
      input_absinfo info{};
      if (!get_abs_info(fd, code, info)) continue;
      if (fallback < 0) fallback = code;
      if (centered_axis(info)) return code;
    }
    return fallback;
  }

  bool probe_gamepad(const std::string &path, GamepadCandidate &candidate) const {
    const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return false;

    std::vector<unsigned long> key_bits;
    const bool has_keys = read_event_bits(fd, EV_KEY, KEY_MAX, key_bits);
    const bool has_game_buttons = has_keys &&
        (bit_is_set(key_bits, BTN_SOUTH) || bit_is_set(key_bits, BTN_EAST) ||
         bit_is_set(key_bits, BTN_TL) || bit_is_set(key_bits, BTN_TR));

    input_absinfo fwd{};
    const bool has_forward = get_abs_info(fd, gamepad_forward_abs_code_, fwd);
    const int yaw_code = choose_yaw_axis(fd);
    input_absinfo yaw{};
    const bool has_yaw = yaw_code >= 0 && get_abs_info(fd, yaw_code, yaw);

    if (!has_game_buttons || !has_forward || !has_yaw) {
      ::close(fd);
      return false;
    }

    candidate.path = path;
    candidate.canonical = real_path(path);
    candidate.name = input_device_name(fd);
    input_device_id(fd, candidate.id);
    candidate.forward_code = gamepad_forward_abs_code_;
    candidate.yaw_code = yaw_code;
    candidate.forward_info = fwd;
    candidate.yaw_info = yaw;

    const std::string n = lowercase_copy(candidate.name);
    int score = 10;
    if (path.find("/by-id/") != std::string::npos) score += 100;
    if (n.find("rexus") != std::string::npos) score += 80;
    if (n.find("daxa") != std::string::npos || n.find("asteria") != std::string::npos) score += 70;
    if (n.find("shanwan") != std::string::npos || n.find("android gamepad") != std::string::npos ||
        n.find("tyx") != std::string::npos) score += 60;
    if (n.find("gamepad") != std::string::npos || n.find("controller") != std::string::npos) score += 30;
    if (candidate.id.vendor == 0x2563) score += 40;  // ShanWan family legacy ID.
    if (candidate.id.vendor == 0x20bc && candidate.id.product == 0x5001) score += 120; // AX1 wireless fallback seen on Linux.
    if (candidate.id.vendor == 0x057e && candidate.id.product == 0x2009) score += 100; // Switch-compatible mode.
    if (candidate.id.vendor == 0x05ac && candidate.id.product == 0x033e) score += 90;  // Alternate multimode fallback.
    if (candidate.yaw_code == ABS_Z) score += 15;   // preserve old axis[2] on Android/ShanWan mode.
    if (candidate.yaw_code == ABS_RX) score += 10;  // standard/XInput fallback.
    candidate.score = score;

    ::close(fd);
    return true;
  }

  bool open_best_gamepad(GamepadDevice &dev) {
    std::vector<GamepadCandidate> candidates;
    int permission_errors = 0;

    for (const auto &path : gamepad_candidates()) {
      GamepadCandidate c;
      if (probe_gamepad(path, c)) {
        candidates.push_back(std::move(c));
      } else if (::access(path.c_str(), R_OK) != 0 && (errno == EACCES || errno == EPERM)) {
        ++permission_errors;
      }
    }

    if (candidates.empty()) {
      if (!gamepad_absent_logged_) {
        gamepad_absent_logged_ = true;
        if (permission_errors > 0) {
          RCLCPP_WARN(get_logger(), "[JOY] Gamepad ada tetapi permission /dev/input ditolak; joystick dinonaktifkan sementara.");
        } else {
          RCLCPP_WARN(get_logger(),
                      "[JOY] Belum menemukan event joystick yang punya ABS_Y + right-stick + tombol gamepad. Auto-scan tetap berjalan.");
        }
      }
      return false;
    }

    std::sort(candidates.begin(), candidates.end(), [](const auto &a, const auto &b) {
      if (a.score != b.score) return a.score > b.score;
      return a.path < b.path;
    });

    for (const auto &best : candidates) {
      const int fd = ::open(best.path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
      if (fd < 0) continue;
      if (!input_fd_alive(fd)) {
        ::close(fd);
        continue;
      }

      dev.fd = fd;
      dev.candidate = best;
      dev.ready = false;
      dev.wait_center_logged = false;
      dev.button_down.clear();

      input_absinfo fwd_now{};
      input_absinfo yaw_now{};
      if (!get_abs_info(fd, best.forward_code, fwd_now) || !get_abs_info(fd, best.yaw_code, yaw_now)) {
        ::close(fd);
        dev.fd = -1;
        continue;
      }

      dev.forward_center = (static_cast<double>(fwd_now.minimum) + static_cast<double>(fwd_now.maximum)) * 0.5;
      dev.yaw_center = (static_cast<double>(yaw_now.minimum) + static_cast<double>(yaw_now.maximum)) * 0.5;

      init_button_snapshot(dev);
      set_gamepad_connected(true, 0.0, 0.0);
      gamepad_absent_logged_ = false;

      RCLCPP_INFO(get_logger(),
                  "[JOY] CONNECT %s | name='%s' | id=%s:%s | forward=%s[%d..%d] yaw=%s[%d..%d]",
                  best.path.c_str(), best.name.c_str(), hex4(best.id.vendor).c_str(),
                  hex4(best.id.product).c_str(), abs_code_name(best.forward_code),
                  fwd_now.minimum, fwd_now.maximum, abs_code_name(best.yaw_code),
                  yaw_now.minimum, yaw_now.maximum);
      RCLCPP_INFO(get_logger(),
                  "[JOY] Mapping aktif: old axis[1]=>%s, old axis[2]=>%s. Menunggu stick netral sekali sebelum enable.",
                  abs_code_name(best.forward_code), abs_code_name(best.yaw_code));
      if (best.id.vendor == 0x20bc && best.id.product == 0x5001) {
        RCLCPP_INFO(get_logger(),
                    "[JOY-USB] ShanWan 20bc:5001 terdeteksi (wireless/fallback AX1); hotplug reconnect aktif.");
      }
      return true;
    }

    return false;
  }

  void init_button_snapshot(GamepadDevice &dev) {
    std::vector<unsigned long> key_state(static_cast<size_t>((KEY_MAX + kBitsPerLong) / kBitsPerLong), 0UL);
    if (::ioctl(dev.fd, EVIOCGKEY(key_state.size() * sizeof(unsigned long)), key_state.data()) < 0) return;
    const uint16_t codes[] = {BTN_TL, BTN_TR, BTN_TL2, BTN_TR2};
    for (uint16_t code : codes) dev.button_down[code] = bit_is_set(key_state, code);
  }

  double normalize_evdev_axis(int raw, const input_absinfo &info, double center,
                              bool invert) const {
    const double min_v = static_cast<double>(info.minimum);
    const double max_v = static_cast<double>(info.maximum);
    const double raw_v = static_cast<double>(raw);
    if (max_v <= min_v) return 0.0;

    double value = 0.0;
    if (raw_v >= center) {
      const double span = max_v - center;
      if (span > 0.0) value = (raw_v - center) / span;
    } else {
      const double span = center - min_v;
      if (span > 0.0) value = (raw_v - center) / span;
    }
    value = clamp_value(value, -1.0, 1.0);
    if (invert) value = -value;

    const double half_range = (max_v - min_v) * 0.5;
    const double kernel_flat = half_range > 0.0 ? static_cast<double>(info.flat) / half_range : 0.0;
    const double dz = std::max(joy_forward_deadzone_, std::max(0.0, kernel_flat));
    const double sat = std::max(dz + 1.0e-6, joy_forward_full_scale_threshold_);
    const double mag = std::abs(value);
    if (mag <= dz) return 0.0;
    if (mag >= sat) return std::copysign(1.0, value);
    const double scaled = (mag - dz) / (sat - dz);
    return std::copysign(clamp_value(scaled, 0.0, 1.0), value);
  }

  double normalize_steering_axis(int raw, const input_absinfo &info, double center,
                                 bool invert) const {
    const double min_v = static_cast<double>(info.minimum);
    const double max_v = static_cast<double>(info.maximum);
    const double raw_v = static_cast<double>(raw);
    if (max_v <= min_v) return 0.0;

    double value = 0.0;
    if (raw_v >= center) {
      const double span = max_v - center;
      if (span > 0.0) value = (raw_v - center) / span;
    } else {
      const double span = center - min_v;
      if (span > 0.0) value = (raw_v - center) / span;
    }
    value = clamp_value(value, -1.0, 1.0);
    if (invert) value = -value;

    const double half_range = (max_v - min_v) * 0.5;
    const double kernel_flat = half_range > 0.0 ? static_cast<double>(info.flat) / half_range : 0.0;
    const double dz = std::max(joy_yaw_deadzone_, std::max(0.0, kernel_flat));
    const double sat = std::max(dz + 1.0e-6, joy_yaw_full_scale_threshold_);
    const double mag = std::abs(value);

    if (mag <= dz) return 0.0;
    if (mag >= sat) return std::copysign(1.0, value);

    // Continuous 0..1 mapping outside the deadzone. The old code returned the
    // original normalized value here, so e.g. 0.121 immediately became 12.1%
    // steering after a 0.12 deadzone. This caused direction-dependent center
    // offsets on sticks with mechanical return hysteresis.
    const double scaled = (mag - dz) / (sat - dz);
    return std::copysign(clamp_value(scaled, 0.0, 1.0), value);
  }

  double raw_center_fraction(int raw, const input_absinfo &info) const {
    const double range = static_cast<double>(info.maximum) - static_cast<double>(info.minimum);
    if (range <= 0.0) return 1.0;
    const double mid = (static_cast<double>(info.maximum) + static_cast<double>(info.minimum)) * 0.5;
    return std::abs(static_cast<double>(raw) - mid) / (range * 0.5);
  }

  void handle_gamepad_button(GamepadDevice &dev, uint16_t code, int32_t value) {
    std::string action;
    std::string label;
    if (code == BTN_TL) { action = "speed_up"; label = "JOY L1"; }
    else if (code == BTN_TL2) { action = "speed_down"; label = "JOY L2"; }
    else if (code == BTN_TR) { action = "yaw_up"; label = "JOY R1"; }
    else if (code == BTN_TR2) { action = "yaw_down"; label = "JOY R2"; }
    else return;

    const bool pressed = value != 0;
    const bool previous = dev.button_down[code];
    dev.button_down[code] = pressed;
    if (!pressed || previous || value == 2) return;

    const double now_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    double &last = button_last_trigger_sec_[code];
    if ((now_sec - last) >= button_debounce_sec_) {
      last = now_sec;
      adjust_limit(action, label);
    }
  }

  bool update_gamepad_axes(GamepadDevice &dev) {
    input_absinfo fwd{};
    input_absinfo yaw{};
    if (!get_abs_info(dev.fd, dev.candidate.forward_code, fwd) ||
        !get_abs_info(dev.fd, dev.candidate.yaw_code, yaw)) {
      return false;
    }

    if (!dev.ready) {
      const double fcenter = raw_center_fraction(fwd.value, fwd);
      const double ycenter = raw_center_fraction(yaw.value, yaw);
      if (fcenter <= joy_axis_init_tolerance_ && ycenter <= joy_axis_init_tolerance_) {
        if (joy_auto_center_) {
          if (fcenter <= joy_auto_center_max_abs_) dev.forward_center = static_cast<double>(fwd.value);
          if (ycenter <= joy_auto_center_max_abs_) dev.yaw_center = static_cast<double>(yaw.value);
        }
        dev.ready = true;
        dev.wait_center_logged = false;
        RCLCPP_INFO(get_logger(), "[JOY] READY center raw: %s=%.1f %s=%.1f",
                    abs_code_name(dev.candidate.forward_code), dev.forward_center,
                    abs_code_name(dev.candidate.yaw_code), dev.yaw_center);
      } else {
        set_gamepad_connected(true, 0.0, 0.0);
        if (!dev.wait_center_logged) {
          dev.wait_center_logged = true;
          RCLCPP_INFO(get_logger(),
                      "[JOY] Menunggu stick netral | %s raw=%d (%.2f dari center), %s raw=%d (%.2f dari center)",
                      abs_code_name(dev.candidate.forward_code), fwd.value, fcenter,
                      abs_code_name(dev.candidate.yaw_code), yaw.value, ycenter);
        }
        return true;
      }
    }

    const double forward = normalize_evdev_axis(
        fwd.value, fwd, dev.forward_center, gamepad_invert_forward_);
    const double yaw_fraction = normalize_steering_axis(
        yaw.value, yaw, dev.yaw_center, gamepad_invert_yaw_);
    set_gamepad_connected(true, forward, yaw_fraction);
    return true;
  }

  void set_gamepad_connected(bool connected, double forward, double yaw_fraction) {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    gamepad_connected_ = connected;
    joy_forward_fraction_ = connected ? forward : 0.0;
    joy_yaw_fraction_ = connected ? yaw_fraction : 0.0;
  }

  void close_gamepad(GamepadDevice &dev, const char *reason) {
    if (dev.fd < 0) return;
    const std::string path = dev.candidate.path;
    const std::string name = dev.candidate.name;
    const input_id id = dev.candidate.id;
    ::close(dev.fd);
    dev.fd = -1;
    dev.ready = false;
    dev.button_down.clear();
    set_gamepad_connected(false, 0.0, 0.0);
    ++gamepad_disconnect_count_;
    RCLCPP_WARN(get_logger(), "[JOY] DISCONNECT #%llu %s (%s) | %s | output joystick langsung 0, auto-reconnect aktif",
                static_cast<unsigned long long>(gamepad_disconnect_count_), path.c_str(), name.c_str(), reason);
    if (id.vendor == 0x20bc && id.product == 0x5001 && gamepad_disconnect_count_ >= 2) {
      RCLCPP_ERROR(get_logger(),
                   "[JOY-USB] 20bc:5001 sudah hilang dari kernel berulang. Driver akan reconnect otomatis, tetapi kestabilan fisik/USB perlu fix power/enumeration; lihat README bagian REXUS 20bc:5001.");
    }
  }

  void gamepad_loop() {
    GamepadDevice dev;
    auto next_scan = std::chrono::steady_clock::now();

    while (running_.load() && rclcpp::ok()) {
      if (dev.fd < 0) {
        const auto now_steady = std::chrono::steady_clock::now();
        if (now_steady >= next_scan) {
          if (open_best_gamepad(dev)) {
            next_scan = now_steady;
          } else {
            next_scan = now_steady + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                         std::chrono::duration<double>(gamepad_rescan_sec_));
          }
        }
        if (dev.fd < 0) {
          std::this_thread::sleep_for(20ms);
          continue;
        }
      }

      pollfd pfd{dev.fd, POLLIN | POLLPRI, 0};
      const int pr = ::poll(&pfd, 1, 20);
      if (pr < 0) {
        if (errno == EINTR) continue;
        close_gamepad(dev, std::strerror(errno));
        next_scan = std::chrono::steady_clock::now();
        continue;
      }

      if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        if (!input_fd_alive(dev.fd)) {
          close_gamepad(dev, "kernel event device hilang");
          next_scan = std::chrono::steady_clock::now();
          continue;
        }
      }

      if (pfd.revents & POLLIN) {
        input_event events[64];
        const ssize_t bytes = ::read(dev.fd, events, sizeof(events));
        if (bytes == 0) {
          if (!input_fd_alive(dev.fd)) {
            close_gamepad(dev, "EOF/ENODEV");
            next_scan = std::chrono::steady_clock::now();
            continue;
          }
        } else if (bytes < 0) {
          if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR && !input_fd_alive(dev.fd)) {
            close_gamepad(dev, std::strerror(errno));
            next_scan = std::chrono::steady_clock::now();
            continue;
          }
        } else {
          const size_t count = static_cast<size_t>(bytes) / sizeof(input_event);
          for (size_t i = 0; i < count; ++i) {
            if (events[i].type == EV_KEY) handle_gamepad_button(dev, events[i].code, events[i].value);
          }
        }
      }

      // Snapshot axis melalui ioctl setiap loop. Tidak membutuhkan autorepeat /joy;
      // stick diam tetap dianggap konek, event yang terlewat tidak membuat state stale.
      if (!update_gamepad_axes(dev)) {
        close_gamepad(dev, "EVIOCGABS gagal/device reset");
        next_scan = std::chrono::steady_clock::now();
      }
    }

    if (dev.fd >= 0) ::close(dev.fd);
    set_gamepad_connected(false, 0.0, 0.0);
  }

  ControlFractions joystick_fractions(bool &connected) const {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    connected = gamepad_connected_;
    return {joy_forward_fraction_, joy_yaw_fraction_};
  }

  // ----------------------------- Shared control -----------------------------
  void process_estop_reset(const ControlFractions &key,
                           double joy_forward, double joy_yaw) {
    if (!estop_reset_requested_.exchange(false)) return;
    if (!almost_zero(key.forward) || !almost_zero(key.yaw)) {
      RCLCPP_WARN(get_logger(), "[E-STOP] Reset ditolak: W/A/S/D masih ditekan.");
      return;
    }
    if (!almost_zero(joy_forward) || !almost_zero(joy_yaw)) {
      RCLCPP_WARN(get_logger(), "[E-STOP] Reset ditolak: joystick belum netral.");
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!estop_latched_) {
      RCLCPP_INFO(get_logger(), "[E-STOP] ENTER: E-STOP memang tidak aktif.");
      return;
    }
    estop_latched_ = false;
    RCLCPP_INFO(get_logger(), "[E-STOP] RESET OK.");
  }

  void adjust_limit(const std::string &action, const std::string &source) {
    double old_value = 0.0;
    double new_value = 0.0;
    std::string unit;
    std::string label;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (action == "speed_up" || action == "speed_down") {
        old_value = max_speed_;
        max_speed_ = clamp_value(max_speed_ + (action == "speed_up" ? speed_step_ : -speed_step_),
                                 speed_min_, speed_max_);
        max_speed_ = std::round(max_speed_ * 1000.0) / 1000.0;
        new_value = max_speed_;
        label = "speed";
        unit = "m/s";
      } else if (action == "yaw_up" || action == "yaw_down") {
        old_value = max_yaw_deg_s_;
        max_yaw_deg_s_ = clamp_value(max_yaw_deg_s_ + (action == "yaw_up" ? yaw_step_deg_s_ : -yaw_step_deg_s_),
                                     yaw_min_deg_s_, yaw_max_deg_s_);
        max_yaw_deg_s_ = std::round(max_yaw_deg_s_ * 1000.0) / 1000.0;
        new_value = max_yaw_deg_s_;
        label = "yaw";
        unit = "deg/s";
      } else {
        return;
      }
    }
    if (log_command_changes_) {
      if (label == "speed") {
        const double old_rpm = speed_max_ > 1.0e-9 ? (old_value / speed_max_) * 300.0 : 0.0;
        const double new_rpm = speed_max_ > 1.0e-9 ? (new_value / speed_max_) * 300.0 : 0.0;
        RCLCPP_INFO(get_logger(),
                    "[LIMIT] %s speed %.2f -> %.2f m/s | STM target scale ~%.0f -> %.0f RPM",
                    source.c_str(), old_value, new_value, old_rpm, new_rpm);
      } else {
        RCLCPP_INFO(get_logger(), "[LIMIT] %s %s %.1f -> %.1f %s",
                    source.c_str(), label.c_str(), old_value, new_value, unit.c_str());
      }
    }
    publish_limits();
  }

  void publish_limits() {
    double speed = 0.0;
    double yaw_deg = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      speed = max_speed_;
      yaw_deg = max_yaw_deg_s_;
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {speed, yaw_deg * kPi / 180.0};
    limits_pub_->publish(msg);
  }

  void publish_estop_state() {
    bool latched = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latched = estop_latched_;
    }
    std_msgs::msg::Bool msg;
    msg.data = latched;
    estop_pub_->publish(msg);
  }

  double shape_linear_velocity(double target_linear, bool estop,
                               bool gamepad_connected, const ControlFractions &key) {
    const auto now_steady = std::chrono::steady_clock::now();

    if (estop) {
      shaped_linear_mps_ = 0.0;
      shaped_linear_initialized_ = true;
      shaped_linear_last_update_ = now_steady;
      return 0.0;
    }

    if (!gamepad_connected && almost_zero(key.forward)) {
      shaped_linear_mps_ = 0.0;
      shaped_linear_initialized_ = true;
      shaped_linear_last_update_ = now_steady;
      return 0.0;
    }

    if (!teleop_velocity_smoothing_enabled_) {
      shaped_linear_mps_ = target_linear;
      shaped_linear_initialized_ = true;
      shaped_linear_last_update_ = now_steady;
      return target_linear;
    }

    if (!shaped_linear_initialized_) {
      shaped_linear_initialized_ = true;
      shaped_linear_mps_ = 0.0;
      shaped_linear_last_update_ = now_steady;
    }

    const double dt = std::clamp(
      std::chrono::duration<double>(now_steady - shaped_linear_last_update_).count(),
      0.0, 0.10);
    shaped_linear_last_update_ = now_steady;

    const bool accelerating_same_direction =
      (shaped_linear_mps_ * target_linear >= 0.0) &&
      (std::abs(target_linear) > std::abs(shaped_linear_mps_));
    const double rate = accelerating_same_direction
      ? teleop_accel_limit_mps2_ : teleop_decel_limit_mps2_;
    const double max_delta = rate * dt;
    const double delta = std::clamp(
      target_linear - shaped_linear_mps_, -max_delta, max_delta);
    shaped_linear_mps_ += delta;

    if (std::abs(target_linear) <= 1.0e-9 &&
        std::abs(shaped_linear_mps_) <= teleop_zero_snap_mps_) {
      shaped_linear_mps_ = 0.0;
    }
    return shaped_linear_mps_;
  }

  void publish_command() {
    const auto key = keyboard_fractions();
    bool gamepad_connected = false;
    const auto joy = joystick_fractions(gamepad_connected);
    const double joy_forward = joy.forward;
    const double joy_yaw = joy.yaw;

    process_estop_reset(key, joy_forward, joy_yaw);

    const double forward_fraction = clamp_value(key.forward + joy_forward, -1.0, 1.0);
    const double yaw_fraction = clamp_value(key.yaw + joy_yaw, -1.0, 1.0);

    double speed_limit = 0.0;
    double yaw_limit_deg = 0.0;
    bool estop = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      speed_limit = max_speed_;
      yaw_limit_deg = max_yaw_deg_s_;
      estop = estop_latched_;
    }

    geometry_msgs::msg::Twist twist;
    std::string source = "STOP";
    if (estop) {
      source = "E_STOP";
      (void)shape_linear_velocity(0.0, true, gamepad_connected, key);
    } else {
      const double target_linear = forward_fraction * speed_limit;
      twist.linear.x = shape_linear_velocity(target_linear, false, gamepad_connected, key);
      twist.angular.z = yaw_fraction * (yaw_limit_deg * kPi / 180.0);
      const bool key_active = !almost_zero(key.forward) || !almost_zero(key.yaw);
      const bool joy_active = gamepad_connected && (!almost_zero(joy_forward) || !almost_zero(joy_yaw));
      if (key_active && joy_active) source = "KEY+JOY";
      else if (key_active) source = "KEYBOARD";
      else if (joy_active) source = "JOYSTICK";
    }

    // Teleop never writes UART. esc_ackermann arbitrates and converts this Twist.

    cmd_pub_->publish(twist);
    std_msgs::msg::String source_msg;
    source_msg.data = source;
    source_pub_->publish(source_msg);
    publish_estop_state();

    if (log_command_changes_) {
      log_output_if_changed(source, twist, key, joy_forward, joy_yaw,
                            speed_limit, yaw_limit_deg, estop, gamepad_connected);
    }
  }

  void log_output_if_changed(const std::string &source, const geometry_msgs::msg::Twist &twist,
                             const ControlFractions &key, double joy_forward, double joy_yaw,
                             double speed_limit, double yaw_limit_deg, bool estop,
                             bool gamepad_connected) {
    const int linear_milli = static_cast<int>(std::lround(twist.linear.x * 1000.0));
    const int yaw_milli = static_cast<int>(std::lround(twist.angular.z * 1000.0));
    const int key_f = static_cast<int>(std::lround(key.forward * 1000.0));
    const int key_y = static_cast<int>(std::lround(key.yaw * 1000.0));
    const int joy_f = static_cast<int>(std::lround(joy_forward * 1000.0));
    const int joy_y = static_cast<int>(std::lround(joy_yaw * 1000.0));
    const int speed_limit_milli = static_cast<int>(std::lround(speed_limit * 1000.0));
    const int yaw_limit_milli = static_cast<int>(std::lround(yaw_limit_deg * 1000.0));

    if (last_log_valid_ && source == last_source_ && linear_milli == last_linear_milli_ &&
        yaw_milli == last_yaw_milli_ && key_f == last_key_f_ && key_y == last_key_y_ &&
        joy_f == last_joy_f_ && joy_y == last_joy_y_ && estop == last_estop_ &&
        gamepad_connected == last_gamepad_connected_ &&
        speed_limit_milli == last_speed_limit_milli_ &&
        yaw_limit_milli == last_yaw_limit_milli_) {
      return;
    }

    last_log_valid_ = true;
    last_source_ = source;
    last_linear_milli_ = linear_milli;
    last_yaw_milli_ = yaw_milli;
    last_key_f_ = key_f;
    last_key_y_ = key_y;
    last_joy_f_ = joy_f;
    last_joy_y_ = joy_y;
    last_estop_ = estop;
    last_gamepad_connected_ = gamepad_connected;
    last_speed_limit_milli_ = speed_limit_milli;
    last_yaw_limit_milli_ = yaw_limit_milli;

    RCLCPP_INFO(get_logger(),
                "[OUT] %-8s | joyLink=%s status=%s | key=(%+.2f,%+.2f) joy=(%+.2f,%+.2f) | speedMax=%.2f yawMax=%.1fdeg/s | cmd=(%+.3f,%+.3f)",
                source.c_str(),
                gamepad_connected ? "UP" : "DOWN",
                gamepad_connected ? "CONNECTED" : "DISCONNECTED",
                key.forward, key.yaw, joy_forward, joy_yaw, speed_limit, yaw_limit_deg,
                twist.linear.x, twist.angular.z);
  }

  void stop_input_threads() {
    const bool was_running = running_.exchange(false);
    if (!was_running) return;
    if (keyboard_thread_.joinable()) keyboard_thread_.join();
    if (gamepad_thread_.joinable()) gamepad_thread_.join();
  }

  // ROS
  std::string output_topic_{"/cmd_vel/teleop"};
  double publish_rate_hz_{50.0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr limits_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex state_mutex_;
  std::unordered_map<uint16_t, int> key_press_count_;
  double max_speed_{1.00};
  double speed_step_{0.10};
  double speed_min_{0.10};
  double speed_max_{1.00};
  double max_yaw_deg_s_{80.0};
  double yaw_step_deg_s_{5.0};
  double yaw_min_deg_s_{5.0};
  double yaw_max_deg_s_{80.0};
  bool estop_latched_{false};
  std::atomic<bool> estop_reset_requested_{false};

  // Keyboard
  std::string keyboard_device_;
  double keyboard_rescan_sec_{2.0};
  bool ignore_controller_keyboard_interfaces_{true};
  bool suppress_terminal_echo_{true};
  bool terminal_echo_suppressed_{false};
  TerminalEchoGuard terminal_guard_;
  bool keyboard_connected_reported_{false};
  bool keyboard_status_initialized_{false};

  // Rexus/evdev gamepad
  std::string gamepad_device_;
  double gamepad_rescan_sec_{0.10};
  uint64_t gamepad_disconnect_count_{0};
  int gamepad_forward_abs_code_{ABS_Y};
  int gamepad_yaw_abs_code_{-1};
  bool gamepad_invert_forward_{true};
  bool gamepad_invert_yaw_{false};
  double joy_deadzone_{0.12};
  double joy_forward_deadzone_{0.12};
  double joy_forward_full_scale_threshold_{0.90};
  bool teleop_velocity_smoothing_enabled_{true};
  double teleop_accel_limit_mps2_{1.50};
  double teleop_decel_limit_mps2_{2.50};
  double teleop_zero_snap_mps_{0.015};
  double shaped_linear_mps_{0.0};
  bool shaped_linear_initialized_{false};
  std::chrono::steady_clock::time_point shaped_linear_last_update_{std::chrono::steady_clock::now()};
  double joy_yaw_deadzone_{0.18};
  double joy_yaw_full_scale_threshold_{0.90};
  double joy_axis_init_tolerance_{0.35};
  bool joy_auto_center_{true};
  double joy_auto_center_max_abs_{0.08};
  double button_debounce_sec_{0.20};
  bool gamepad_absent_logged_{false};
  std::unordered_map<uint16_t, double> button_last_trigger_sec_;

  mutable std::mutex joy_mutex_;
  bool gamepad_connected_{false};
  double joy_forward_fraction_{0.0};
  double joy_yaw_fraction_{0.0};

  std::atomic<bool> running_{false};
  std::thread keyboard_thread_;
  std::thread gamepad_thread_;

  // Logging
  bool log_command_changes_{true};
  bool last_log_valid_{false};
  std::string last_source_;
  int last_linear_milli_{0};
  int last_yaw_milli_{0};
  int last_key_f_{0};
  int last_key_y_{0};
  int last_joy_f_{0};
  int last_joy_y_{0};
  bool last_estop_{false};
  bool last_gamepad_connected_{false};
  int last_speed_limit_milli_{0};
  int last_yaw_limit_milli_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MotorTeleop>();
    rclcpp::spin(node);
    node->publish_stop_burst();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_teleop"), "Fatal: %s", e.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}