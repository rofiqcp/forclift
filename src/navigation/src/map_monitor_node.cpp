#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <chrono>

namespace navigation
{

class MapMonitorNode : public rclcpp::Node
{
public:
  MapMonitorNode() : Node("map_monitor_node")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    min_known_cells_ = std::max<int64_t>(1, declare_parameter<int64_t>("min_known_cells", 100));
    min_occupied_cells_ = std::max<int64_t>(1, declare_parameter<int64_t>("min_occupied_cells", 5));
    stats_min_interval_s_ = std::clamp(
      declare_parameter<double>("stats_min_interval", 0.75), 0.1, 10.0);
    occupied_drop_warn_ratio_ = std::clamp(
      declare_parameter<double>("occupied_drop_warn_ratio", 0.20), 0.05, 0.95);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { on_map(*msg); });

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("/mapping/map_valid", state_qos);
    stats_pub_ = create_publisher<std_msgs::msg::String>("/mapping/map_stats", state_qos);

    RCLCPP_INFO(get_logger(), "Watching SLAM OccupancyGrid on %s", map_topic_.c_str());
  }

private:
  void on_map(const nav_msgs::msg::OccupancyGrid & msg)
  {
    // OccupancyGrid can be large. Do not rescan every cell at the full /map
    // refresh rate; that steals CPU from slam_toolbox exactly when fast mapping
    // is desired. Geometry is cheap to validate on every message, detailed cell
    // statistics are throttled independently.
    const auto now_t = now();
    if (last_stats_time_.nanoseconds() > 0 &&
      (now_t - last_stats_time_).seconds() < stats_min_interval_s_)
    {
      return;
    }
    last_stats_time_ = now_t;

    int64_t unknown = 0;
    int64_t free_cells = 0;
    int64_t occupied = 0;
    int64_t intermediate = 0;

    for (const int8_t value : msg.data) {
      if (value < 0) {
        ++unknown;
      } else if (value <= 20) {
        ++free_cells;
      } else if (value >= 50) {
        ++occupied;
      } else {
        ++intermediate;
      }
    }

    const int64_t known = free_cells + occupied + intermediate;
    const bool geometry_ok = msg.info.width > 0 && msg.info.height > 0 && msg.info.resolution > 0.0f;

    if (occupied > occupied_peak_) {
      occupied_peak_ = occupied;
    }
    const double occupied_retention = occupied_peak_ > 0 ?
      static_cast<double>(occupied) / static_cast<double>(occupied_peak_) : 1.0;
    const bool valid = geometry_ok && known >= min_known_cells_ && occupied >= min_occupied_cells_;

    std_msgs::msg::Bool valid_msg;
    valid_msg.data = valid;
    valid_pub_->publish(valid_msg);

    std::ostringstream out;
    out << "frame=" << msg.header.frame_id
        << ";size=" << msg.info.width << "x" << msg.info.height
        << ";resolution=" << msg.info.resolution
        << ";known=" << known
        << ";free=" << free_cells
        << ";occupied=" << occupied
        << ";intermediate=" << intermediate
        << ";unknown=" << unknown
        << ";occupied_peak=" << occupied_peak_
        << ";occupied_retention=" << occupied_retention
        << ";valid=" << std::boolalpha << valid;
    std_msgs::msg::String stats_msg;
    stats_msg.data = out.str();
    stats_pub_->publish(stats_msg);

    if (last_log_.nanoseconds() == 0 || (now_t - last_log_).seconds() >= 2.0) {
      if (valid) {
        RCLCPP_INFO(
          get_logger(), "SLAM MAP VALID | %ux%u @ %.3f m | known=%ld free=%ld occupied=%ld unknown=%ld | occupied-retention=%.1f%%",
          msg.info.width, msg.info.height, msg.info.resolution,
          static_cast<long>(known), static_cast<long>(free_cells),
          static_cast<long>(occupied), static_cast<long>(unknown), occupied_retention * 100.0);
        if (occupied_peak_ >= 100 && occupied_retention < (1.0 - occupied_drop_warn_ratio_)) {
          RCLCPP_WARN(
            get_logger(), "Occupied cells dropped %.1f%% from peak (%ld -> %ld). Check LiDAR odometry/scan alignment if static walls visibly disappear.",
            (1.0 - occupied_retention) * 100.0,
            static_cast<long>(occupied_peak_), static_cast<long>(occupied));
        }
      } else {
        RCLCPP_WARN(
          get_logger(), "SLAM map received but not useful yet | %ux%u | known=%ld occupied=%ld",
          msg.info.width, msg.info.height,
          static_cast<long>(known), static_cast<long>(occupied));
      }
      last_log_ = now_t;
    }
  }

  std::string map_topic_;
  int64_t min_known_cells_{100};
  int64_t min_occupied_cells_{5};
  double stats_min_interval_s_{0.75};
  double occupied_drop_warn_ratio_{0.20};
  int64_t occupied_peak_{0};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub_;
  rclcpp::Time last_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stats_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation::MapMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
