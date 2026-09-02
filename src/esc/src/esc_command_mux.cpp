#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace
{

rclcpp::QoS stateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

bool finiteTwist(const geometry_msgs::msg::Twist & msg)
{
  return std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
    std::isfinite(msg.linear.z) && std::isfinite(msg.angular.x) &&
    std::isfinite(msg.angular.y) && std::isfinite(msg.angular.z);
}

geometry_msgs::msg::Twist zeroTwist()
{
  return geometry_msgs::msg::Twist{};
}

}  // namespace

class EscCommandMux final : public rclcpp::Node
{
public:
  EscCommandMux()
  : Node("esc_command_mux")
  {
    declareParameters();
    readParameters();
    createInterfaces();

    RCLCPP_INFO(
      get_logger(),
      "ESC mux aktif: nav2=%s keyboard=%s joy=/joy -> %s | standalone=%s",
      nav2_topic_.c_str(), keyboard_topic_.c_str(), output_topic_.c_str(),
      standalone_mode_ ? "true" : "false");
  }

private:
  struct CommandSample
  {
    geometry_msgs::msg::Twist cmd{};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
    rclcpp::Time takeover_until{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void declareParameters()
  {
    declare_parameter<std::string>("nav2_topic", "/cmd_vel");
    declare_parameter<std::string>("keyboard_topic", "/cmd_vel/keyboard");
    declare_parameter<std::string>("remote_topic", "/cmd_vel/remote");
    declare_parameter<std::string>("gui_topic", "/cmd_vel/gui");
    declare_parameter<std::string>("output_topic", "/cmd_vel/actuator");
    declare_parameter<std::string>("autonomy_gate_topic", "/system/autonomy_motion_allowed");
    declare_parameter<std::string>("manual_gate_topic", "/system/manual_motion_allowed");
    declare_parameter<std::string>("estop_topic", "/safety/estop");
    declare_parameter<std::string>("keyboard_estop_request_topic", "/esc/keyboard/estop_request");

    declare_parameter<bool>("standalone_mode", false);
    declare_parameter<bool>("enable_nav2", true);
    declare_parameter<bool>("enable_keyboard", true);
    declare_parameter<bool>("enable_joystick", true);
    declare_parameter<bool>("enable_remote", false);
    declare_parameter<bool>("enable_gui", false);
    declare_parameter<bool>("require_autonomy_gate", true);
    declare_parameter<double>("autonomy_gate_timeout_sec", 0.60);
    declare_parameter<bool>("require_manual_gate", true);
    declare_parameter<double>("manual_gate_timeout_sec", 0.60);

    declare_parameter<double>("command_rate_hz", 50.0);
    declare_parameter<double>("nav2_timeout_sec", 0.60);
    declare_parameter<double>("keyboard_timeout_sec", 0.35);
    declare_parameter<double>("joy_timeout_sec", 0.30);
    declare_parameter<double>("remote_timeout_sec", 0.30);
    declare_parameter<double>("gui_timeout_sec", 0.30);
    declare_parameter<double>("manual_release_hold_sec", 0.50);

    declare_parameter<double>("wheelbase_m", 0.70);
    declare_parameter<double>("max_forward_speed_mps", 1.20);
    declare_parameter<double>("max_reverse_speed_mps", 0.30);
    declare_parameter<double>("max_steering_angle_rad", 0.34);
    declare_parameter<double>("max_yaw_rate_rps", 0.60);
    declare_parameter<double>("manual_max_forward_speed_mps", 0.30);
    declare_parameter<double>("manual_max_reverse_speed_mps", 0.15);
    declare_parameter<double>("manual_max_yaw_rate_rps", 0.35);
    declare_parameter<double>("linear_deadband_mps", 0.03);
    declare_parameter<double>("angular_deadband_rps", 0.01);
    declare_parameter<double>("min_speed_for_yaw_mps", 0.05);

    declare_parameter<int>("joy_deadman_button", 4);
    declare_parameter<int>("joy_estop_button", 1);
    declare_parameter<int>("joy_clear_estop_button", 3);
    declare_parameter<int>("joy_speed_axis", 1);
    declare_parameter<int>("joy_steering_axis", 3);
    declare_parameter<int>("joy_turbo_button", 5);
    declare_parameter<double>("joy_normal_max_speed_mps", 0.60);
    declare_parameter<double>("joy_axis_deadband", 0.08);
  }

  void readParameters()
  {
    nav2_topic_ = get_parameter("nav2_topic").as_string();
    keyboard_topic_ = get_parameter("keyboard_topic").as_string();
    remote_topic_ = get_parameter("remote_topic").as_string();
    gui_topic_ = get_parameter("gui_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    autonomy_gate_topic_ = get_parameter("autonomy_gate_topic").as_string();
    manual_gate_topic_ = get_parameter("manual_gate_topic").as_string();
    estop_topic_ = get_parameter("estop_topic").as_string();
    keyboard_estop_request_topic_ = get_parameter("keyboard_estop_request_topic").as_string();

    standalone_mode_ = get_parameter("standalone_mode").as_bool();
    enable_nav2_ = get_parameter("enable_nav2").as_bool();
    enable_keyboard_ = get_parameter("enable_keyboard").as_bool();
    enable_joystick_ = get_parameter("enable_joystick").as_bool();
    enable_remote_ = get_parameter("enable_remote").as_bool();
    enable_gui_ = get_parameter("enable_gui").as_bool();
    require_autonomy_gate_ = get_parameter("require_autonomy_gate").as_bool();
    autonomy_gate_timeout_sec_ = std::clamp(get_parameter("autonomy_gate_timeout_sec").as_double(), 0.10, 2.0);
    require_manual_gate_ = get_parameter("require_manual_gate").as_bool();
    manual_gate_timeout_sec_ = std::clamp(get_parameter("manual_gate_timeout_sec").as_double(), 0.10, 2.0);

    command_rate_hz_ = std::clamp(get_parameter("command_rate_hz").as_double(), 10.0, 100.0);
    nav2_timeout_sec_ = std::clamp(get_parameter("nav2_timeout_sec").as_double(), 0.10, 2.0);
    keyboard_timeout_sec_ = std::clamp(get_parameter("keyboard_timeout_sec").as_double(), 0.10, 2.0);
    joy_timeout_sec_ = std::clamp(get_parameter("joy_timeout_sec").as_double(), 0.10, 2.0);
    remote_timeout_sec_ = std::clamp(get_parameter("remote_timeout_sec").as_double(), 0.10, 2.0);
    gui_timeout_sec_ = std::clamp(get_parameter("gui_timeout_sec").as_double(), 0.10, 2.0);
    manual_release_hold_sec_ = std::clamp(get_parameter("manual_release_hold_sec").as_double(), 0.0, 2.0);

    wheelbase_m_ = std::max(0.01, get_parameter("wheelbase_m").as_double());
    max_forward_speed_mps_ = std::max(0.0, get_parameter("max_forward_speed_mps").as_double());
    max_reverse_speed_mps_ = std::max(0.0, get_parameter("max_reverse_speed_mps").as_double());
    max_steering_angle_rad_ = std::clamp(get_parameter("max_steering_angle_rad").as_double(), 0.01, 1.2);
    max_yaw_rate_rps_ = std::max(0.01, get_parameter("max_yaw_rate_rps").as_double());
    manual_max_forward_speed_mps_ = std::clamp(get_parameter("manual_max_forward_speed_mps").as_double(), 0.0, max_forward_speed_mps_);
    manual_max_reverse_speed_mps_ = std::clamp(get_parameter("manual_max_reverse_speed_mps").as_double(), 0.0, max_reverse_speed_mps_);
    manual_max_yaw_rate_rps_ = std::clamp(get_parameter("manual_max_yaw_rate_rps").as_double(), 0.01, max_yaw_rate_rps_);
    linear_deadband_mps_ = std::max(0.0, get_parameter("linear_deadband_mps").as_double());
    angular_deadband_rps_ = std::max(0.0, get_parameter("angular_deadband_rps").as_double());
    min_speed_for_yaw_mps_ = std::max(0.0, get_parameter("min_speed_for_yaw_mps").as_double());

    joy_deadman_button_ = static_cast<int>(get_parameter("joy_deadman_button").as_int());
    joy_estop_button_ = static_cast<int>(get_parameter("joy_estop_button").as_int());
    joy_clear_estop_button_ = static_cast<int>(get_parameter("joy_clear_estop_button").as_int());
    joy_speed_axis_ = static_cast<int>(get_parameter("joy_speed_axis").as_int());
    joy_steering_axis_ = static_cast<int>(get_parameter("joy_steering_axis").as_int());
    joy_turbo_button_ = static_cast<int>(get_parameter("joy_turbo_button").as_int());
    joy_normal_max_speed_mps_ = std::clamp(
      get_parameter("joy_normal_max_speed_mps").as_double(), 0.0, max_forward_speed_mps_);
    joy_axis_deadband_ = std::clamp(get_parameter("joy_axis_deadband").as_double(), 0.0, 0.5);
  }

  void createInterfaces()
  {
    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

    if (enable_nav2_) {
      nav2_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        nav2_topic_, cmd_qos,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) { storeNav2(*msg); });
    }
    if (enable_keyboard_) {
      keyboard_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        keyboard_topic_, cmd_qos,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) { storeManual(keyboard_, *msg, "keyboard"); });
    }
    if (enable_remote_) {
      remote_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        remote_topic_, cmd_qos,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) { storeManual(remote_, *msg, "remote"); });
    }
    if (enable_gui_) {
      gui_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        gui_topic_, cmd_qos,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) { storeManual(gui_, *msg, "gui"); });
    }
    if (enable_joystick_) {
      joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/joy", sensor_qos, std::bind(&EscCommandMux::onJoy, this, std::placeholders::_1));
    }

    autonomy_gate_sub_ = create_subscription<std_msgs::msg::Bool>(
      autonomy_gate_topic_, stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        autonomy_gate_ = msg->data;
        autonomy_gate_seen_ = true;
        autonomy_gate_received_ = now();
      });
    manual_gate_sub_ = create_subscription<std_msgs::msg::Bool>(
      manual_gate_topic_, stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        manual_gate_ = msg->data;
        manual_gate_seen_ = true;
        manual_gate_received_ = now();
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic_, stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        estop_ = msg->data;
      });

    keyboard_estop_request_sub_ = create_subscription<std_msgs::msg::Bool>(
      keyboard_estop_request_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](std_msgs::msg::Bool::SharedPtr msg) { publishEstop(msg->data); });

    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, cmd_qos);
    selected_pub_ = create_publisher<geometry_msgs::msg::Twist>("/esc/mux/selected", cmd_qos);
    // Manual-only diagnostic untuk HUD. Berisi keyboard/joystick/remote/GUI yang
    // benar-benar memegang mux; nol saat Nav2/idle sehingga tidak ambigu.
    teleop_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/teleop", cmd_qos);
    source_pub_ = create_publisher<std_msgs::msg::String>("/esc/mux/active_source", stateQos());
    status_pub_ = create_publisher<std_msgs::msg::String>("/esc/mux/status", stateQos());
    estop_pub_ = create_publisher<std_msgs::msg::Bool>(estop_topic_, stateQos());

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / command_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&EscCommandMux::tick, this));
  }

  geometry_msgs::msg::Twist clampCommand(const geometry_msgs::msg::Twist & in) const
  {
    geometry_msgs::msg::Twist out{};
    out.linear.x = std::clamp(in.linear.x, -max_reverse_speed_mps_, max_forward_speed_mps_);
    out.angular.z = std::clamp(in.angular.z, -max_yaw_rate_rps_, max_yaw_rate_rps_);
    if (std::abs(out.linear.x) < linear_deadband_mps_) out.linear.x = 0.0;
    if (std::abs(out.angular.z) < angular_deadband_rps_) out.angular.z = 0.0;
    if (std::abs(out.linear.x) < min_speed_for_yaw_mps_) out.angular.z = 0.0;
    return out;
  }

  geometry_msgs::msg::Twist clampManualCommand(const geometry_msgs::msg::Twist & in) const
  {
    auto out = clampCommand(in);
    if (!standalone_mode_) {
      out.linear.x = std::clamp(out.linear.x, -manual_max_reverse_speed_mps_, manual_max_forward_speed_mps_);
      out.angular.z = std::clamp(out.angular.z, -manual_max_yaw_rate_rps_, manual_max_yaw_rate_rps_);
    }
    return out;
  }

  void storeNav2(const geometry_msgs::msg::Twist & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto t = now();
    if (!finiteTwist(msg)) {
      nav2_ = {zeroTwist(), t, rclcpp::Time(0, 0, RCL_ROS_TIME), false};
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Nav2 cmd_vel NaN/Inf -> dibuang");
      return;
    }
    nav2_.cmd = clampCommand(msg);
    nav2_.received = t;
    nav2_.valid = true;
  }

  void storeManual(CommandSample & slot, const geometry_msgs::msg::Twist & msg, const char * source)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto t = now();
    if (!finiteTwist(msg)) {
      slot = {zeroTwist(), t, rclcpp::Time(0, 0, RCL_ROS_TIME), false};
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Manual command %s NaN/Inf -> STOP", source);
      return;
    }

    const bool was_nonzero = slot.valid &&
      (std::abs(slot.cmd.linear.x) > 1.0e-6 || std::abs(slot.cmd.angular.z) > 1.0e-6);
    const geometry_msgs::msg::Twist clamped = clampManualCommand(msg);
    const bool is_nonzero =
      std::abs(clamped.linear.x) > 1.0e-6 || std::abs(clamped.angular.z) > 1.0e-6;

    slot.cmd = clamped;
    slot.received = t;
    slot.valid = true;
    // Important: joy_node/GUI can publish zero heartbeat forever. A zero sample
    // must NOT continuously refresh manual ownership, otherwise Nav2/keyboard is
    // starved while the manual controller is actually idle. Hold is armed only
    // by an active command or the first transition active -> zero.
    if (is_nonzero || was_nonzero) {
      slot.takeover_until = t + rclcpp::Duration::from_seconds(manual_release_hold_sec_);
    }
  }

  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const auto button = [&msg](int index) -> bool {
      return index >= 0 && static_cast<size_t>(index) < msg->buttons.size() && msg->buttons[index] != 0;
    };
    const auto axis = [&msg](int index) -> double {
      return index >= 0 && static_cast<size_t>(index) < msg->axes.size() ? msg->axes[index] : 0.0;
    };

    if (button(joy_estop_button_)) publishEstop(true);
    if (button(joy_clear_estop_button_)) publishEstop(false);

    geometry_msgs::msg::Twist cmd{};
    if (button(joy_deadman_button_)) {
      double speed_axis = axis(joy_speed_axis_);
      double steer_axis = axis(joy_steering_axis_);
      if (std::abs(speed_axis) < joy_axis_deadband_) speed_axis = 0.0;
      if (std::abs(steer_axis) < joy_axis_deadband_) steer_axis = 0.0;

      const double max_speed = button(joy_turbo_button_) ? max_forward_speed_mps_ : joy_normal_max_speed_mps_;
      cmd.linear.x = speed_axis >= 0.0 ? speed_axis * max_speed : speed_axis * max_reverse_speed_mps_;
      const double steering = steer_axis * max_steering_angle_rad_;
      if (std::abs(cmd.linear.x) >= min_speed_for_yaw_mps_) {
        cmd.angular.z = cmd.linear.x * std::tan(steering) / wheelbase_m_;
      }
    }
    storeManual(joy_, cmd, "joystick");
  }

  void publishEstop(bool active)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      estop_ = active;
    }
    std_msgs::msg::Bool msg;
    msg.data = active;
    estop_pub_->publish(msg);
  }

  bool fresh(const CommandSample & slot, double timeout, const rclcpp::Time & t) const
  {
    if (!slot.valid || slot.received.nanoseconds() == 0) return false;
    const double age = (t - slot.received).seconds();
    return age >= 0.0 && age <= timeout;
  }

  bool manualOwns(const CommandSample & slot, double timeout, const rclcpp::Time & t) const
  {
    if (!fresh(slot, timeout, t)) return false;
    const bool nonzero = std::abs(slot.cmd.linear.x) > 1.0e-6 || std::abs(slot.cmd.angular.z) > 1.0e-6;
    return nonzero || t <= slot.takeover_until;
  }

  void tick()
  {
    geometry_msgs::msg::Twist output{};
    std::string source = "idle";
    bool gate = false;
    bool estop = false;
    bool nav2_fresh = false;
    bool autonomy_gate = false;
    bool autonomy_gate_seen = false;
    bool autonomy_gate_fresh = false;
    bool manual_gate = false;
    bool manual_gate_seen = false;
    bool manual_gate_fresh = false;
    bool manual_allowed = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto t = now();
      estop = estop_;
      nav2_fresh = fresh(nav2_, nav2_timeout_sec_, t);
      autonomy_gate = autonomy_gate_;
      autonomy_gate_seen = autonomy_gate_seen_;
      autonomy_gate_fresh = autonomy_gate_seen_ && autonomy_gate_received_.nanoseconds() != 0 &&
        (t - autonomy_gate_received_).seconds() >= 0.0 &&
        (t - autonomy_gate_received_).seconds() <= autonomy_gate_timeout_sec_;
      manual_gate = manual_gate_;
      manual_gate_seen = manual_gate_seen_;
      manual_gate_fresh = manual_gate_seen_ && manual_gate_received_.nanoseconds() != 0 &&
        (t - manual_gate_received_).seconds() >= 0.0 &&
        (t - manual_gate_received_).seconds() <= manual_gate_timeout_sec_;
      manual_allowed = standalone_mode_ || !require_manual_gate_ ||
        (manual_gate_seen_ && manual_gate_fresh && manual_gate_);

      if (estop_) {
        source = "ESTOP";
      } else if (enable_remote_ && manualOwns(remote_, remote_timeout_sec_, t)) {
        source = manual_allowed ? "remote" : "remote_blocked";
        if (manual_allowed) { output = remote_.cmd; gate = true; }
      } else if (enable_joystick_ && manualOwns(joy_, joy_timeout_sec_, t)) {
        source = manual_allowed ? "joystick" : "joystick_blocked";
        if (manual_allowed) { output = joy_.cmd; gate = true; }
      } else if (enable_keyboard_ && manualOwns(keyboard_, keyboard_timeout_sec_, t)) {
        source = manual_allowed ? "keyboard" : "keyboard_blocked";
        if (manual_allowed) { output = keyboard_.cmd; gate = true; }
      } else if (enable_gui_ && manualOwns(gui_, gui_timeout_sec_, t)) {
        source = manual_allowed ? "gui" : "gui_blocked";
        if (manual_allowed) { output = gui_.cmd; gate = true; }
      } else if (enable_nav2_ && nav2_fresh) {
        const bool autonomy_allowed = standalone_mode_ || !require_autonomy_gate_ ||
          (autonomy_gate_seen_ && autonomy_gate_fresh && autonomy_gate_);
        source = autonomy_allowed ? "nav2" : "nav2_blocked";
        if (autonomy_allowed) {
          output = nav2_.cmd;
          gate = true;
        }
      }

      if (!gate) output = zeroTwist();
      output = clampCommand(output);
    }

    output_pub_->publish(output);
    selected_pub_->publish(output);
    geometry_msgs::msg::Twist teleop{};
    if (source == "remote" || source == "joystick" || source == "keyboard" || source == "gui") {
      teleop = output;
    }
    teleop_pub_->publish(teleop);

    if (source != last_source_) {
      std_msgs::msg::String msg;
      msg.data = source;
      source_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "ESC mux source: %s", source.c_str());
      last_source_ = source;
    }

    const auto t = now();
    if (last_status_time_.nanoseconds() == 0 || (t - last_status_time_).seconds() >= 1.0) {
      std_msgs::msg::String msg;
      std::ostringstream ss;
      ss << std::boolalpha
         << "source=" << source
         << ";standalone=" << standalone_mode_
         << ";estop=" << estop
         << ";autonomy_gate=" << autonomy_gate
         << ";autonomy_gate_seen=" << autonomy_gate_seen
         << ";autonomy_gate_fresh=" << autonomy_gate_fresh
         << ";manual_gate=" << manual_gate
         << ";manual_gate_seen=" << manual_gate_seen
         << ";manual_gate_fresh=" << manual_gate_fresh
         << ";manual_allowed=" << manual_allowed
         << ";nav2_fresh=" << nav2_fresh
         << ";out_v=" << output.linear.x
         << ";out_w=" << output.angular.z;
      msg.data = ss.str();
      status_pub_->publish(msg);
      last_status_time_ = t;
    }
  }

  std::mutex mutex_;
  std::string nav2_topic_;
  std::string keyboard_topic_;
  std::string remote_topic_;
  std::string gui_topic_;
  std::string output_topic_;
  std::string autonomy_gate_topic_;
  std::string manual_gate_topic_;
  std::string estop_topic_;
  std::string keyboard_estop_request_topic_;

  bool standalone_mode_{false};
  bool enable_nav2_{true};
  bool enable_keyboard_{true};
  bool enable_joystick_{true};
  bool enable_remote_{false};
  bool enable_gui_{false};
  bool require_autonomy_gate_{true};
  bool autonomy_gate_{false};
  bool autonomy_gate_seen_{false};
  double autonomy_gate_timeout_sec_{0.60};
  rclcpp::Time autonomy_gate_received_{0, 0, RCL_ROS_TIME};
  bool require_manual_gate_{true};
  bool manual_gate_{false};
  bool manual_gate_seen_{false};
  double manual_gate_timeout_sec_{0.60};
  rclcpp::Time manual_gate_received_{0, 0, RCL_ROS_TIME};
  bool estop_{false};

  double command_rate_hz_{50.0};
  double nav2_timeout_sec_{0.60};
  double keyboard_timeout_sec_{0.35};
  double joy_timeout_sec_{0.30};
  double remote_timeout_sec_{0.30};
  double gui_timeout_sec_{0.30};
  double manual_release_hold_sec_{0.50};
  double wheelbase_m_{0.70};
  double max_forward_speed_mps_{1.20};
  double max_reverse_speed_mps_{0.30};
  double max_steering_angle_rad_{0.34};
  double max_yaw_rate_rps_{0.60};
  double manual_max_forward_speed_mps_{0.30};
  double manual_max_reverse_speed_mps_{0.15};
  double manual_max_yaw_rate_rps_{0.35};
  double linear_deadband_mps_{0.03};
  double angular_deadband_rps_{0.01};
  double min_speed_for_yaw_mps_{0.05};

  int joy_deadman_button_{4};
  int joy_estop_button_{1};
  int joy_clear_estop_button_{3};
  int joy_speed_axis_{1};
  int joy_steering_axis_{3};
  int joy_turbo_button_{5};
  double joy_normal_max_speed_mps_{0.60};
  double joy_axis_deadband_{0.08};

  CommandSample nav2_;
  CommandSample keyboard_;
  CommandSample joy_;
  CommandSample remote_;
  CommandSample gui_;
  std::string last_source_;
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr remote_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gui_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomy_gate_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_gate_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr keyboard_estop_request_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr selected_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EscCommandMux>());
  rclcpp::shutdown();
  return 0;
}
