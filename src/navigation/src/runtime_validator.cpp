#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

namespace navigation
{

class RuntimeValidator final : public rclcpp::Node
{
public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using Navigate = nav2_msgs::action::NavigateToPose;

  RuntimeValidator()
  : Node("navigation_runtime_validator_cpp")
  {
    require_path_ = declare_parameter<bool>("require_path", false);
    require_amcl_ = declare_parameter<bool>("require_amcl", true);
    freshness_sec_ = declare_parameter<double>("freshness_sec", 2.0);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_nav", sensor_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!msg->ranges.empty()) scan_time_ = steadyNow();
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos,
      [this](sensor_msgs::msg::Imu::SharedPtr) { imu_time_ = steadyNow(); });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (msg->info.width > 0U && msg->info.height > 0U && !msg->data.empty()) map_time_ = steadyNow();
      });
    global_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/global_costmap/costmap", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!msg->data.empty()) global_costmap_time_ = steadyNow();
      });
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!msg->data.empty()) local_costmap_time_ = steadyNow();
      });
    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", rclcpp::QoS(10).reliable(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) { amcl_time_ = steadyNow(); });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/smac_plan", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.size() >= 2U) {
          path_time_ = steadyNow();
          path_points_ = msg->poses.size();
        }
      });
    status_sub_ = create_subscription<std_msgs::msg::String>(
      "/navigation/planner_status", 10,
      [this](std_msgs::msg::String::SharedPtr msg) { planner_status_ = msg->data; });

    compute_client_ = rclcpp_action::create_client<ComputePath>(this, "/compute_path_to_pose");
    navigate_client_ = rclcpp_action::create_client<Navigate>(this, "/navigate_to_pose");
  }

  static double steadyNow()
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool fresh(double stamp) const
  {
    return stamp > 0.0 && steadyNow() - stamp <= freshness_sec_;
  }

  bool actionsReady() const
  {
    return compute_client_->action_server_is_ready() && navigate_client_->action_server_is_ready();
  }

  bool baselineReady() const
  {
    const bool localization_ok = !require_amcl_ || fresh(amcl_time_);
    return fresh(scan_time_) && fresh(imu_time_) && map_time_ > 0.0 &&
           fresh(global_costmap_time_) && fresh(local_costmap_time_) &&
           localization_ok && actionsReady();
  }

  bool complete() const
  {
    return baselineReady() && (!require_path_ || path_time_ > 0.0);
  }

  void report() const
  {
    const auto state = [this](double t, bool latched = false) {
      if (latched) return t > 0.0 ? "PASS" : "WAIT";
      return fresh(t) ? "PASS" : "WAIT";
    };
    RCLCPP_INFO(
      get_logger(),
      "[V43-LIVE] scan=%s imu=%s map=%s amcl=%s global=%s local=%s ComputePath=%s Navigate=%s path=%s(%zu)",
      state(scan_time_), state(imu_time_), state(map_time_, true),
      require_amcl_ ? state(amcl_time_) : "SKIP",
      state(global_costmap_time_), state(local_costmap_time_),
      compute_client_->action_server_is_ready() ? "PASS" : "WAIT",
      navigate_client_->action_server_is_ready() ? "PASS" : "WAIT",
      require_path_ ? (path_time_ > 0.0 ? "PASS" : "WAIT") : "OPTIONAL",
      path_points_);
    if (!planner_status_.empty()) {
      RCLCPP_INFO(get_logger(), "[V43-LIVE] planner_status=%s", planner_status_.c_str());
    }
  }

private:
  bool require_path_{false};
  bool require_amcl_{true};
  double freshness_sec_{2.0};
  double scan_time_{0.0};
  double imu_time_{0.0};
  double map_time_{0.0};
  double global_costmap_time_{0.0};
  double local_costmap_time_{0.0};
  double amcl_time_{0.0};
  double path_time_{0.0};
  size_t path_points_{0U};
  std::string planner_status_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp_action::Client<ComputePath>::SharedPtr compute_client_;
  rclcpp_action::Client<Navigate>::SharedPtr navigate_client_;
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation::RuntimeValidator>();
  const double start = navigation::RuntimeValidator::steadyNow();
  double last_report = start - 10.0;
  const double timeout = 45.0;

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    const double now = navigation::RuntimeValidator::steadyNow();
    if (now - last_report >= 2.0) {
      node->report();
      last_report = now;
    }
    if (node->complete()) {
      node->report();
      RCLCPP_INFO(node->get_logger(), "[V43-LIVE] PASS: Nav2 foundation is live%s",
        node->get_parameter("require_path").as_bool() ? " and /smac_plan contains a path" : "");
      rclcpp::shutdown();
      return 0;
    }
    if (now - start >= timeout) {
      node->report();
      RCLCPP_ERROR(node->get_logger(), "[V43-LIVE] FAIL: timeout after %.0f s", timeout);
      rclcpp::shutdown();
      return 2;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return 3;
}
