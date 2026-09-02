#include <algorithm>
#include <cerrno>
#include <chrono>
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

    if (baud != 115200) {
      throw std::invalid_argument("winch firmware currently supports baud=115200 only");
    }

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    connected_pub_ = create_publisher<std_msgs::msg::Bool>("/winch/connected", latched);
    port_pub_ = create_publisher<std_msgs::msg::String>("/winch/port", latched);
    state_pub_ = create_publisher<std_msgs::msg::String>("/winch/state", latched);
    top_pub_ = create_publisher<std_msgs::msg::Bool>("/winch/top_limit", latched);
    bottom_pub_ = create_publisher<std_msgs::msg::Bool>("/winch/bottom_limit", latched);
    pwm_pub_ = create_publisher<std_msgs::msg::Float64>("/winch/pwm_pct", latched);
    direction_pub_ = create_publisher<std_msgs::msg::Int32>("/winch/direction", latched);
    servo_pub_ = create_publisher<std_msgs::msg::Float64>("/winch/servo_deg", latched);
    raw_pub_ = create_publisher<std_msgs::msg::String>("/winch/raw", rclcpp::QoS(20).reliable());

    command_sub_ = create_subscription<std_msgs::msg::String>(
      "/winch/command", rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {handle_command(msg->data);});

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
      if (!write_line("STATUS")) {
        close_serial(true);
        next_reconnect_ = now;
        return;
      }
      next_status_ = now + std::chrono::milliseconds(status_period_ms_);
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
      ::cfsetispeed(&tty, B115200);
      ::cfsetospeed(&tty, B115200);
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
      const auto now = std::chrono::steady_clock::now();
      last_rx_ = now;
      handshake_deadline_ = now + std::chrono::milliseconds(1500);
      next_status_ = now + std::chrono::milliseconds(status_period_ms_);
      publish_connected(false);
      publish_string(port_pub_, active_port_);
      publish_string(state_pub_, "PROBING");

      // Safe reconnect semantics: never inherit motion after a USB reconnect.
      // A valid firmware answers STATUS with STATE:/TOP:/...; only then is the
      // port advertised as CONNECTED. This avoids attaching to another ttyACM.
      if (!write_line("STOP") || !write_line("STATUS")) {
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
      if (line.rfind("STATE:", 0) == 0) {
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
      } else if (line.rfind("SERVO:", 0) == 0) {
        const std::string value = trim_copy(line.substr(6));
        if (upper_copy(value) == "HOME") {
          publish_double(servo_pub_, 0.0);
        } else if (upper_copy(value) == "TOP") {
          publish_double(servo_pub_, 195.0);
        } else {
          publish_double(servo_pub_, std::stod(value));
        }
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
      command == "STATUS" || command == "LIMITS" || command == "SERVOTEST")
    {
      return true;
    }
    if (command.rfind("SERVO ", 0) != 0) {
      return false;
    }
    try {
      const std::string value = trim_copy(command.substr(6));
      std::size_t used = 0;
      const int angle = std::stoi(value, &used);
      return used == value.size() && angle >= 0 && angle <= 195;
    } catch (const std::exception &) {
      return false;
    }
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
  std::int64_t reconnect_interval_ms_{1000};
  std::int64_t status_period_ms_{1000};
  std::int64_t rx_timeout_ms_{5000};
  int fd_{-1};
  std::string active_port_;
  std::string rx_buffer_;
  std::optional<bool> connected_state_;
  bool handshake_confirmed_{false};
  std::size_t candidate_cursor_{0};

  std::chrono::steady_clock::time_point next_reconnect_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point handshake_deadline_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point next_status_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_rx_{std::chrono::steady_clock::now()};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr port_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr top_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bottom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pwm_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr direction_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub_;
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
