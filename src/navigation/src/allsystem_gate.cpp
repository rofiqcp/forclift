// All-System Readiness Gate for ROS 2 Humble AGV
//
// Reusable readiness gates in autonomous startup:
//   serial-gate : /imu/data + /scan + /lidar/odom + /odometry/filtered
//   camera-gate : /camera/color/image_raw (diagnostic; YOLO starts independently)
//   amcl-gate   : /amcl_pose → activates Nav2 lifecycle
//   costmap-gate: global/local OccupancyGrid → verifies RViz-ready costmaps
//
// The gate node publishes a latched bool on:
//   /allsystem_gate/<name>/ready
//
// No fake messages — only real published data satisfies the gate.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <atomic>
#include <cctype>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

constexpr int DEFAULT_MIN_MSGS = 3;
constexpr int DEFAULT_TIMEOUT_MS = 60000;

// ─── Per-topic state (lock-free: only std::atomic members) ───────────────────
// No std::mutex → vector<shared_ptr<TopicCount>> is movable/copyable.
struct TopicCount
{
  std::atomic<int> count{0};
  std::atomic<bool> reached_min{false};
  std::atomic<bool> first_seen_recorded{false};
};

// ─── Gate Node ───────────────────────────────────────────────────────────────

class AllSystemGate : public rclcpp::Node
{
public:
  explicit AllSystemGate(const rclcpp::NodeOptions & opts)
  : Node("allsystem_gate", opts)
  {
    declare_parameter("gate_name",    std::string("sensor"));
    declare_parameter("topics",       std::vector<std::string>{});
    declare_parameter("min_messages", DEFAULT_MIN_MSGS);
    declare_parameter("timeout_ms",    DEFAULT_TIMEOUT_MS);
    declare_parameter("exit_on_ready", false);

    gate_name_  = get_parameter("gate_name").as_string();
    auto topics_vec = get_parameter("topics").as_string_array();
    min_msgs_   = get_parameter("min_messages").as_int();
    timeout_ms_ = get_parameter("timeout_ms").as_int();
    exit_on_ready_ = get_parameter("exit_on_ready").as_bool();

    if (topics_vec.empty()) {
      RCLCPP_ERROR(get_logger(), "No topics specified for gate '%s'", gate_name_.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(),
      "[ALLSYSTEM-GATE] gate='%s' topics=%zu min_msgs=%d timeout=%dms",
      gate_name_.c_str(), topics_vec.size(), min_msgs_, timeout_ms_);
    std::cout << "[" << gate_name_ << "-GATE] waiting for:" << std::endl;
    for (const auto & t : topics_vec) std::cout << "  - " << t << std::endl;

    // ── Latched ready publisher ────────────────────────────────────────────
    // gate_name is human-readable and may contain '-' (e.g. LOCAL-INFLATION).
    // Hyphens are illegal inside ROS 2 topic tokens, so sanitize only the topic
    // token while preserving the original gate name in logs.
    std::string gate_topic_token = gate_name_;
    for (char & c : gate_topic_token) {
      const auto uc = static_cast<unsigned char>(c);
      if (!(std::isalnum(uc) || c == '_')) {
        c = '_';
      }
    }
    if (gate_topic_token.empty() || std::isdigit(static_cast<unsigned char>(gate_topic_token.front()))) {
      gate_topic_token = "gate_" + gate_topic_token;
    }
    const std::string ready_topic = "/allsystem_gate/" + gate_topic_token + "/ready";
    rclcpp::QoS qos_latched(1);
    qos_latched.transient_local();
    pub_ready_ = create_publisher<std_msgs::msg::Bool>(ready_topic, qos_latched);

    // ── Per-topic state as shared_ptr (avoids movable-only struct in vector)
    for (size_t i = 0; i < topics_vec.size(); ++i) {
      states_.push_back(std::make_shared<TopicCount>());
    }

    // ── Subscribe per topic ─────────────────────────────────────────────────
    for (size_t i = 0; i < topics_vec.size(); ++i) {
      const std::string & topic = topics_vec[i];
      std::shared_ptr<TopicCount> st = states_[i];

      if (topic.find("imu") != std::string::npos) {
        subs_.push_back(
          create_subscription<sensor_msgs::msg::Imu>(
            topic, rclcpp::SensorDataQoS().best_effort().keep_last(2),
            [this, topic, st](const sensor_msgs::msg::Imu::SharedPtr) {
              this->cb(topic, st);
            }));
      } else if (topic.find("scan") != std::string::npos) {
        subs_.push_back(
          create_subscription<sensor_msgs::msg::LaserScan>(
            topic, rclcpp::SensorDataQoS().best_effort().keep_last(2),
            [this, topic, st](const sensor_msgs::msg::LaserScan::SharedPtr) {
              this->cb(topic, st);
            }));
      } else if (topic.find("image") != std::string::npos ||
                 topic.find("camera") != std::string::npos) {
        subs_.push_back(
          create_subscription<sensor_msgs::msg::Image>(
            topic, rclcpp::SensorDataQoS().keep_last(2),
            [this, topic, st](const sensor_msgs::msg::Image::SharedPtr) {
              this->cb(topic, st);
            }));
      } else if (topic == "/amcl_pose" || topic == "amcl_pose") {
        // AMCL publishes geometry_msgs/PoseWithCovarianceStamped.  Supporting
        // this explicitly lets autonomous gate Nav2 on real localization data
        // instead of merely checking that the AMCL process exists.
        subs_.push_back(
          create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            topic, rclcpp::QoS(5).reliable(),
            [this, topic, st](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) {
              this->cb(topic, st);
            }));
      } else if (topic.find("odom") != std::string::npos) {
        // Autonomous gates on /lidar/odom and /odometry/filtered.
        // Match the sensor publishers: BEST_EFFORT (RELIABLE would never
        // receive the driver's BEST_EFFORT stream and the gate would hang).
        subs_.push_back(
          create_subscription<nav_msgs::msg::Odometry>(
            topic, rclcpp::SensorDataQoS().best_effort().keep_last(2),
            [this, topic, st](const nav_msgs::msg::Odometry::SharedPtr) {
              this->cb(topic, st);
            }));
      } else if (topic.find("costmap") != std::string::npos ||
                 topic == "/map" || topic == "map") {
        // map_server and Nav2 Costmap2DPublisher use reliable transient-local
        // OccupancyGrid publication. Matching QoS also receives the latest
        // latched grid when this diagnostic gate starts after activation.
        rclcpp::QoS map_qos(1);
        map_qos.reliable().transient_local();
        subs_.push_back(
          create_subscription<nav_msgs::msg::OccupancyGrid>(
            topic, map_qos,
            [this, topic, st](const nav_msgs::msg::OccupancyGrid::SharedPtr) {
              this->cb(topic, st);
            }));
      } else {
        subs_.push_back(
          create_subscription<std_msgs::msg::Bool>(
            topic, rclcpp::SensorDataQoS().keep_last(2),
            [this, topic, st](const std_msgs::msg::Bool::SharedPtr) {
              this->cb(topic, st);
            }));
      }
    }

    // ── Progress timer ───────────────────────────────────────────────────────
    progress_timer_ = create_wall_timer(2s, [this]() { print_progress(); });

    // ── Timeout watchdog ────────────────────────────────────────────────────
    gate_start_ = std::chrono::steady_clock::now();
    timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(timeout_ms_),
      [this]() {
        if (!ready_.load()) {
          const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - gate_start_).count();
          std::cerr << "[" << gate_name_ << "-GATE] TIMEOUT after " << ms
                    << " ms — some topics did not reach minimum count" << std::endl;
          RCLCPP_ERROR(get_logger(),
            "[ALLSYSTEM-GATE] TIMEOUT — gate='%s' after %ld ms",
            gate_name_.c_str(), static_cast<long>(ms));
        }
      });

    // ── Initial READY=false so late subscribers can observe ─────────────────
    std_msgs::msg::Bool false_msg;
    false_msg.data = false;
    pub_ready_->publish(false_msg);
  }

private:
  void cb(const std::string & topic, std::shared_ptr<TopicCount> st)
  {
    if (ready_.load()) return;

    int now = st->count.fetch_add(1) + 1;

    // Log first-seen (lock-free: only one thread wins the exchange)
    if (st->first_seen_recorded.exchange(true) == false) {
      std::cout << "[" << gate_name_ << "-GATE] " << topic
                << " 1/" << min_msgs_ << std::endl;
      RCLCPP_INFO(get_logger(), "[%s-GATE] %s 1/%d",
        gate_name_.c_str(), topic.c_str(), min_msgs_);
    }

    if (now >= min_msgs_ && !st->reached_min.exchange(true)) {
      std::cout << "[" << gate_name_ << "-GATE] " << topic
                << " " << now << "/" << min_msgs_ << " OK" << std::endl;
    }

    // Check if ALL topics reached minimum
    for (auto & s : states_) {
      if (!s->reached_min.load()) return;
    }
    fire_ready();
  }

  void fire_ready()
  {
    if (!ready_.exchange(true)) {
      std::cout << "[" << gate_name_ << "-GATE] READY" << std::endl;
      for (auto & s : states_) {
        std::cout << "  count=" << s->count.load() << std::endl;
      }
      RCLCPP_INFO(get_logger(), "[ALLSYSTEM-GATE] gate='%s' READY", gate_name_.c_str());

      std_msgs::msg::Bool msg;
      msg.data = true;
      pub_ready_->publish(msg);

      if (exit_on_ready_) {
        // Give DDS a short moment to flush the latched READY message, then let
        // the process exit with code 0. allsystem.launch.py chains the next
        // stage from OnProcessExit, so camera/YOLO are truly data-driven.
        exit_timer_ = create_wall_timer(150ms, [this]() {
          exit_timer_->cancel();
          rclcpp::shutdown();
        });
      }
    }
  }

  void print_progress()
  {
    if (ready_.load()) return;
    std::cout << "[" << gate_name_ << "-GATE] ";
    for (auto & s : states_)
      std::cout << "c=" << s->count.load() << "/" << min_msgs_ << " ";
    std::cout << std::endl;
  }

  // Per-topic state as shared_ptr (avoids movable-only struct in vector)
  std::vector<std::shared_ptr<TopicCount>> states_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ready_;
  std::shared_ptr<rclcpp::TimerBase> progress_timer_;
  std::shared_ptr<rclcpp::TimerBase> timeout_timer_;
  std::shared_ptr<rclcpp::TimerBase> exit_timer_;
  std::atomic<bool> ready_{false};
  std::chrono::steady_clock::time_point gate_start_;
  int min_msgs_{DEFAULT_MIN_MSGS};
  int timeout_ms_{DEFAULT_TIMEOUT_MS};
  bool exit_on_ready_{false};
  std::string gate_name_;
};

// ─── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AllSystemGate>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
