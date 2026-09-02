#include <atomic>
#include <memory>
#include <string>
#include <stdexcept>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_controller/plugins/simple_progress_checker.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>

namespace esc
{

class EscAwareProgressChecker : public nav2_controller::SimpleProgressChecker
{
public:
  EscAwareProgressChecker() = default;
  ~EscAwareProgressChecker() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override
  {
    nav2_controller::SimpleProgressChecker::initialize(parent, plugin_name);

    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("EscAwareProgressChecker: lifecycle node expired saat initialize");
    }

    const std::string topic_param = plugin_name + ".esc_ready_topic";
    const std::string bypass_param = plugin_name + ".bypass_when_esc_not_ready";

    if (!node->has_parameter(topic_param)) {
      node->declare_parameter<std::string>(topic_param, "/esc/ready");
    }
    if (!node->has_parameter(bypass_param)) {
      node->declare_parameter<bool>(bypass_param, true);
    }

    esc_ready_topic_ = node->get_parameter(topic_param).as_string();
    bypass_when_esc_not_ready_ = node->get_parameter(bypass_param).as_bool();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();

    esc_ready_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      esc_ready_topic_, qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        // Callback hanya menulis atomic. State milik SimpleProgressChecker tidak
        // disentuh dari callback subscriber agar tidak ada data race dengan
        // thread controller yang memanggil check().
        esc_ready_.store(msg->data, std::memory_order_relaxed);
      });

    RCLCPP_INFO(
      node->get_logger(),
      "EscAwareProgressChecker aktif: topic=%s bypass_offline=%s; progress normal aktif saat ESC ready",
      esc_ready_topic_.c_str(), bypass_when_esc_not_ready_ ? "true" : "false");
  }

  bool check(geometry_msgs::msg::PoseStamped & current_pose) override
  {
    if (bypass_when_esc_not_ready_ && !esc_ready_.load(std::memory_order_relaxed)) {
      // Tidak memalsukan odometri. Hanya menonaktifkan keputusan "stuck" selama
      // actuator memang tidak tersedia, sehingga Goal/Smac/MPPI/cmd_vel tetap hidup
      // untuk commissioning tanpa ESC. Reset baseline tiap siklus agar ketika ESC
      // menjadi ready, pengawasan progress dimulai dari kondisi fisik terbaru.
      nav2_controller::SimpleProgressChecker::reset();
      return true;
    }
    return nav2_controller::SimpleProgressChecker::check(current_pose);
  }

  void reset() override
  {
    nav2_controller::SimpleProgressChecker::reset();
  }

private:
  std::atomic_bool esc_ready_{false};
  bool bypass_when_esc_not_ready_{true};
  std::string esc_ready_topic_{"/esc/ready"};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr esc_ready_sub_;
};

}  // namespace esc

PLUGINLIB_EXPORT_CLASS(esc::EscAwareProgressChecker, nav2_core::ProgressChecker)
