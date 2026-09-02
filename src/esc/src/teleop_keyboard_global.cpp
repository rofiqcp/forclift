#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <fcntl.h>
#include <glob.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace
{

rclcpp::QoS stateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

bool testBit(const unsigned long * bits, int bit)
{
  constexpr int kBitsPerLong = static_cast<int>(sizeof(unsigned long) * 8U);
  return (bits[bit / kBitsPerLong] & (1UL << (bit % kBitsPerLong))) != 0UL;
}

}  // namespace

class GlobalKeyboardTeleop final : public rclcpp::Node
{
public:
  GlobalKeyboardTeleop()
  : Node("esc_keyboard_teleop")
  {
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel/keyboard");
    estop_topic_ = declare_parameter<std::string>("estop_request_topic", "/esc/keyboard/estop_request");
    input_device_ = declare_parameter<std::string>("input_device", "auto");
    enable_global_evdev_ = declare_parameter<bool>("enable_global_evdev", true);
    enable_tty_fallback_ = declare_parameter<bool>("enable_tty_fallback", false);
    require_deadman_ = declare_parameter<bool>("require_deadman", true);
    publish_rate_hz_ = std::clamp(declare_parameter<double>("publish_rate_hz", 40.0), 10.0, 100.0);
    forward_speed_mps_ = std::clamp(declare_parameter<double>("forward_speed_mps", 0.45), 0.05, 1.2);
    reverse_speed_mps_ = std::clamp(declare_parameter<double>("reverse_speed_mps", 0.20), 0.05, 0.5);
    steering_angle_rad_ = std::clamp(declare_parameter<double>("steering_angle_rad", 0.22), 0.03, 0.34);
    wheelbase_m_ = std::max(0.01, declare_parameter<double>("wheelbase_m", 0.70));
    speed_step_mps_ = std::clamp(declare_parameter<double>("speed_step_mps", 0.05), 0.01, 0.25);
    steering_step_rad_ = std::clamp(declare_parameter<double>("steering_step_rad", 0.02), 0.005, 0.10);
    max_forward_speed_mps_ = std::clamp(declare_parameter<double>("max_forward_speed_mps", 1.20), 0.1, 3.0);
    max_reverse_speed_mps_ = std::clamp(declare_parameter<double>("max_reverse_speed_mps", 0.30), 0.05, 1.0);
    max_steering_angle_rad_ = std::clamp(declare_parameter<double>("max_steering_angle_rad", 0.34), 0.05, 1.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      output_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    estop_pub_ = create_publisher<std_msgs::msg::Bool>(estop_topic_, stateQos());

    if (enable_global_evdev_) {
      tryOpenKeyboard();
    }
    if (keyboard_fd_ < 0 && enable_tty_fallback_) {
      if (require_deadman_) {
        RCLCPP_ERROR(
          get_logger(),
          "TTY fallback diblokir: terminal karakter tidak dapat membuktikan tombol dead-man sedang ditahan. "
          "Gunakan evdev RIGHT-CTRL dead-man atau explicit service mode require_deadman=false.");
      } else {
        enableTtyFallback();
      }
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&GlobalKeyboardTeleop::tick, this));
    reopen_timer_ = create_wall_timer(2s, std::bind(&GlobalKeyboardTeleop::retryOpen, this));

    RCLCPP_INFO(
      get_logger(),
      "ESC keyboard global: tahan RIGHT-CTRL + W/S/A/D; RIGHT-CTRL+E/Q speed +/-; RIGHT-CTRL+R/F steering +/-; SPACE E-stop; RIGHT-CTRL+X clear");
  }

  ~GlobalKeyboardTeleop() override
  {
    // Never publish from the destructor.  On SIGINT the ROS context may
    // already be shutdown by rclcpp's signal handler; publishing while the
    // middleware is tearing down was the source of an intermittent exit -11.
    closeKeyboard();
    restoreTty();
  }

  void prepareShutdown()
  {
    if (shutdown_prepared_) return;
    shutdown_prepared_ = true;

    // Publish a final zero only while the ROS context is still valid.  If
    // SIGINT already shut the context down, the command mux/driver shutdown
    // path owns the final safe stop and we deliberately avoid touching DDS.
    if (rclcpp::ok()) publishStop();
    if (timer_) timer_->cancel();
    if (reopen_timer_) reopen_timer_->cancel();
    closeKeyboard();
    restoreTty();
  }

private:
  std::vector<std::string> candidateDevices() const
  {
    if (input_device_ != "auto" && !input_device_.empty()) return {input_device_};

    std::vector<std::string> out;
    glob_t glob_result{};
    if (::glob("/dev/input/by-id/*-event-kbd", 0, nullptr, &glob_result) == 0) {
      for (size_t i = 0; i < glob_result.gl_pathc; ++i) out.emplace_back(glob_result.gl_pathv[i]);
    }
    ::globfree(&glob_result);
    for (int i = 0; i < 32; ++i) {
      const std::string path = "/dev/input/event" + std::to_string(i);
      if (fs::exists(path)) out.push_back(path);
    }
    return out;
  }

  bool looksLikeKeyboard(int fd) const
  {
    std::array<unsigned long, (KEY_MAX / (sizeof(unsigned long) * 8U)) + 2U> bits{};
    if (::ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(bits)), bits.data()) < 0) return false;
    return testBit(bits.data(), KEY_W) && testBit(bits.data(), KEY_S) &&
      testBit(bits.data(), KEY_A) && testBit(bits.data(), KEY_D);
  }

  void tryOpenKeyboard()
  {
    if (keyboard_fd_ >= 0) return;
    bool permission_denied = false;
    for (const auto & path : candidateDevices()) {
      const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
      if (fd < 0) {
        if (errno == EACCES) permission_denied = true;
        continue;
      }
      if (!looksLikeKeyboard(fd)) {
        ::close(fd);
        continue;
      }
      char name[256]{};
      (void)::ioctl(fd, EVIOCGNAME(sizeof(name)), name);
      keyboard_fd_ = fd;
      keyboard_path_ = path;
      warned_permission_ = false;
      warned_missing_ = false;
      RCLCPP_INFO(
        get_logger(), "Global keyboard aktif tanpa fokus window: %s (%s)",
        path.c_str(), name[0] != '\0' ? name : "keyboard");
      return;
    }

    if (permission_denied) {
      if (!warned_permission_) {
        RCLCPP_INFO(
          get_logger(),
          "Tidak punya izin baca /dev/input/event*. Jalankan tools/setup_global_keyboard_access.sh lalu login ulang; TTY fallback dipakai bila tersedia.");
        warned_permission_ = true;
      }
    } else if (!warned_missing_) {
      RCLCPP_INFO(get_logger(), "Keyboard evdev belum ditemukan; hot-plug scan tetap berjalan.");
      warned_missing_ = true;
    }
  }

  void closeKeyboard()
  {
    if (keyboard_fd_ >= 0) {
      ::close(keyboard_fd_);
      keyboard_fd_ = -1;
    }
    keyboard_path_.clear();
    clearMotionKeys();
  }

  void retryOpen()
  {
    if (keyboard_fd_ < 0 && enable_global_evdev_) tryOpenKeyboard();
  }

  void enableTtyFallback()
  {
    if (!::isatty(STDIN_FILENO)) return;
    if (::tcgetattr(STDIN_FILENO, &old_tty_) != 0) return;
    termios raw = old_tty_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      tty_active_ = true;
      const int flags = ::fcntl(STDIN_FILENO, F_GETFL, 0);
      if (flags >= 0) (void)::fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
      RCLCPP_INFO(get_logger(), "Keyboard TTY fallback aktif (membutuhkan fokus terminal)");
    }
  }

  void restoreTty()
  {
    if (tty_active_) {
      (void)::tcsetattr(STDIN_FILENO, TCSANOW, &old_tty_);
      tty_active_ = false;
    }
  }

  void clearMotionKeys()
  {
    key_w_ = key_s_ = key_a_ = key_d_ = key_deadman_ = false;
    if (motion_active_) {
      publishStop();
      motion_active_ = false;
    }
  }

  void setKey(unsigned int code, bool down)
  {
    switch (code) {
      case KEY_W: key_w_ = down; break;
      case KEY_S: key_s_ = down; break;
      case KEY_A: key_a_ = down; break;
      case KEY_D: key_d_ = down; break;
      case KEY_RIGHTCTRL: key_deadman_ = down; break;
      default: break;
    }
  }

  void handlePress(unsigned int code)
  {
    // E-stop engage is intentionally always available. Every command that can
    // enable motion again or alter commissioning limits requires the dedicated
    // global dead-man (RIGHT CTRL), so normal typing in another application
    // cannot change robot behavior.
    if (code == KEY_SPACE) {
      publishEstop(true);
      return;
    }
    if (require_deadman_ && !key_deadman_) return;

    switch (code) {
      case KEY_E:
        forward_speed_mps_ = std::min(max_forward_speed_mps_, forward_speed_mps_ + speed_step_mps_);
        reverse_speed_mps_ = std::min(max_reverse_speed_mps_, reverse_speed_mps_ + speed_step_mps_);
        RCLCPP_INFO(get_logger(), "Keyboard speed: forward=%.2f reverse=%.2f", forward_speed_mps_, reverse_speed_mps_);
        break;
      case KEY_Q:
        forward_speed_mps_ = std::max(0.05, forward_speed_mps_ - speed_step_mps_);
        reverse_speed_mps_ = std::max(0.05, reverse_speed_mps_ - speed_step_mps_);
        RCLCPP_INFO(get_logger(), "Keyboard speed: forward=%.2f reverse=%.2f", forward_speed_mps_, reverse_speed_mps_);
        break;
      case KEY_R:
        steering_angle_rad_ = std::min(max_steering_angle_rad_, steering_angle_rad_ + steering_step_rad_);
        RCLCPP_INFO(get_logger(), "Keyboard steering max: %.3f rad", steering_angle_rad_);
        break;
      case KEY_F:
        steering_angle_rad_ = std::max(0.03, steering_angle_rad_ - steering_step_rad_);
        RCLCPP_INFO(get_logger(), "Keyboard steering max: %.3f rad", steering_angle_rad_);
        break;
      case KEY_X:
        publishEstop(false);
        break;
      default:
        break;
    }
  }

  void consumeEvdev()
  {
    if (keyboard_fd_ < 0) return;
    pollfd pfd{};
    pfd.fd = keyboard_fd_;
    pfd.events = POLLIN | POLLERR | POLLHUP;
    const int rc = ::poll(&pfd, 1, 0);
    if (rc < 0) {
      if (errno != EINTR) closeKeyboard();
      return;
    }
    if (rc == 0) return;
    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      RCLCPP_WARN(get_logger(), "Keyboard evdev terputus; STOP dan scan ulang");
      closeKeyboard();
      return;
    }

    input_event event{};
    while (true) {
      const ssize_t bytes = ::read(keyboard_fd_, &event, sizeof(event));
      if (bytes == static_cast<ssize_t>(sizeof(event))) {
        if (event.type != EV_KEY) continue;
        const bool down = event.value != 0;
        setKey(event.code, down);
        if (event.value == 1) handlePress(event.code);
      } else {
        if (bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          RCLCPP_WARN(get_logger(), "Read keyboard gagal: %s; STOP dan scan ulang", std::strerror(errno));
          closeKeyboard();
        }
        break;
      }
    }
  }

  void consumeTty()
  {
    if (!tty_active_ || keyboard_fd_ >= 0) return;
    if (require_deadman_) {
      publishStop();
      return;
    }
    char c = 0;
    while (::read(STDIN_FILENO, &c, 1) == 1) {
      geometry_msgs::msg::Twist cmd{};
      switch (c) {
        case 'w': case 'W': cmd.linear.x = forward_speed_mps_; break;
        case 's': case 'S': cmd.linear.x = -reverse_speed_mps_; break;
        case 'a': case 'A': cmd.linear.x = forward_speed_mps_; cmd.angular.z = forward_speed_mps_ * std::tan(steering_angle_rad_) / wheelbase_m_; break;
        case 'd': case 'D': cmd.linear.x = forward_speed_mps_; cmd.angular.z = -forward_speed_mps_ * std::tan(steering_angle_rad_) / wheelbase_m_; break;
        case ' ': publishEstop(true); continue;
        case 'x': case 'X': publishEstop(false); continue;
        case 'q': case 'Q': handlePress(KEY_Q); continue;
        case 'e': case 'E': handlePress(KEY_E); continue;
        case 'r': case 'R': handlePress(KEY_R); continue;
        case 'f': case 'F': handlePress(KEY_F); continue;
        default: cmd = geometry_msgs::msg::Twist{}; break;
      }
      cmd_pub_->publish(cmd);
      tty_last_command_ = now();
    }
  }

  geometry_msgs::msg::Twist currentCommand() const
  {
    geometry_msgs::msg::Twist cmd{};
    if (require_deadman_ && !key_deadman_) return cmd;
    const int direction = (key_w_ ? 1 : 0) - (key_s_ ? 1 : 0);
    const int steering = (key_a_ ? 1 : 0) - (key_d_ ? 1 : 0);
    if (direction > 0) cmd.linear.x = forward_speed_mps_;
    else if (direction < 0) cmd.linear.x = -reverse_speed_mps_;

    if (direction != 0 && steering != 0) {
      const double steer_rad = static_cast<double>(steering) * steering_angle_rad_;
      cmd.angular.z = cmd.linear.x * std::tan(steer_rad) / wheelbase_m_;
    }
    return cmd;
  }

  void publishStop()
  {
    if (cmd_pub_) cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void publishEstop(bool active)
  {
    std_msgs::msg::Bool msg;
    msg.data = active;
    estop_pub_->publish(msg);
    RCLCPP_WARN(get_logger(), "E-STOP %s dari keyboard", active ? "ACTIVE" : "CLEAR");
  }

  void tick()
  {
    consumeEvdev();
    consumeTty();

    if (keyboard_fd_ >= 0) {
      const auto cmd = currentCommand();
      const bool active = std::abs(cmd.linear.x) > 1.0e-6 || std::abs(cmd.angular.z) > 1.0e-6;
      if (active) {
        cmd_pub_->publish(cmd);
      } else if (motion_active_) {
        publishStop();
      }
      motion_active_ = active;
    } else if (tty_active_ && tty_last_command_.nanoseconds() > 0) {
      if ((now() - tty_last_command_).seconds() > 0.20) {
        publishStop();
        tty_last_command_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
    }
  }

  std::string output_topic_;
  std::string estop_topic_;
  std::string input_device_;
  bool enable_global_evdev_{true};
  bool enable_tty_fallback_{true};
  bool require_deadman_{true};
  double publish_rate_hz_{40.0};
  double forward_speed_mps_{0.45};
  double reverse_speed_mps_{0.20};
  double steering_angle_rad_{0.22};
  double wheelbase_m_{0.70};
  double speed_step_mps_{0.05};
  double steering_step_rad_{0.02};
  double max_forward_speed_mps_{1.20};
  double max_reverse_speed_mps_{0.30};
  double max_steering_angle_rad_{0.34};

  int keyboard_fd_{-1};
  std::string keyboard_path_;
  bool warned_permission_{false};
  bool warned_missing_{false};
  bool key_w_{false};
  bool key_s_{false};
  bool key_a_{false};
  bool key_d_{false};
  bool key_deadman_{false};
  bool motion_active_{false};
  bool shutdown_prepared_{false};

  bool tty_active_{false};
  termios old_tty_{};
  rclcpp::Time tty_last_command_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr reopen_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalKeyboardTeleop>();
  rclcpp::spin(node);
  node->prepareShutdown();
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
