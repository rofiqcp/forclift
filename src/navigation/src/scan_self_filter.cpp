#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <functional>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace navigation
{

class ScanSelfFilter final : public rclcpp::Node
{
public:
  ScanSelfFilter()
  : Node("scan_self_filter_cpp"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan_nav");
    legacy_output_topic_ = declare_parameter<std::string>("legacy_output_topic", "");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    fail_open_on_tf_error_ = declare_parameter<bool>("fail_open_on_tf_error", false);
    min_x_ = declare_parameter<double>("min_x", -0.70);
    max_x_ = declare_parameter<double>("max_x", 0.70);
    min_y_ = declare_parameter<double>("min_y", -0.45);
    max_y_ = declare_parameter<double>("max_y", 0.45);
    max_output_range_ = declare_parameter<double>("max_output_range", 0.0);
    denoise_enabled_ = declare_parameter<bool>("denoise_enabled", false);
    median_radius_bins_ = std::max(1, static_cast<int>(declare_parameter<int>("median_radius_bins", 2)));
    min_neighbor_support_ = std::max(1, static_cast<int>(declare_parameter<int>("min_neighbor_support", 2)));
    outlier_abs_m_ = std::max(0.01, declare_parameter<double>("outlier_abs_m", 0.35));
    outlier_rel_ = std::max(0.0, declare_parameter<double>("outlier_rel", 0.08));
    isolated_min_range_m_ = std::max(0.0, declare_parameter<double>("isolated_min_range_m", 1.0));
    temporal_jump_m_ = std::max(0.01, declare_parameter<double>("temporal_jump_m", 0.60));
    const bool navigation_output = output_topic_ == "/scan_nav";
    output_rate_limit_hz_ = std::max(
      0.0, declare_parameter<double>(
        "output_rate_limit_hz", navigation_output ? 4.0 : 0.0));

    // Navigation consumers (AMCL/costmaps) must prefer the newest scan under
    // Jetson load instead of accumulating stale frames.  Keep the safety
    // instance at its existing depth/rate; only /scan_nav uses depth=1 and a
    // bounded 4 Hz default.  /scan_safety therefore remains full-rate.
    auto qos = rclcpp::SensorDataQoS().keep_last(navigation_output ? 1 : 5);
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, qos);
    if (!legacy_output_topic_.empty() && legacy_output_topic_ != output_topic_) {
      legacy_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(legacy_output_topic_, qos);
    }
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, qos, std::bind(&ScanSelfFilter::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "[SCAN-FILTER-CPP] READY %s -> %s mask=[%.2f,%.2f]x[%.2f,%.2f] denoise=%s cap=%.2f rate_limit=%.1fHz",
      input_topic_.c_str(), output_topic_.c_str(), min_x_, max_x_, min_y_, max_y_,
      denoise_enabled_ ? "on" : "off", max_output_range_, output_rate_limit_hz_);
  }

private:
  static float median(std::vector<float> values)
  {
    if (values.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + static_cast<long>(mid), values.end());
    if ((values.size() % 2U) == 1U) {
      return values[mid];
    }
    const float hi = values[mid];
    std::nth_element(values.begin(), values.begin() + static_cast<long>(mid - 1), values.end());
    return 0.5f * (values[mid - 1] + hi);
  }

  static bool finiteRange(float r, const sensor_msgs::msg::LaserScan & msg)
  {
    return std::isfinite(r) && r >= msg.range_min && r <= msg.range_max;
  }

  size_t wrapIndex(long idx, size_t size) const
  {
    const long n = static_cast<long>(size);
    idx %= n;
    if (idx < 0) {
      idx += n;
    }
    return static_cast<size_t>(idx);
  }

  bool getTransform(const std::string & scan_frame, geometry_msgs::msg::TransformStamped & tf)
  {
    try {
      tf = tf_buffer_.lookupTransform(base_frame_, scan_frame, tf2::TimePointZero);
      cached_tf_ = tf;
      have_cached_tf_ = true;
      return true;
    } catch (const std::exception & ex) {
      if (have_cached_tf_) {
        tf = cached_tf_;
        return true;
      }
      if (fail_open_on_tf_error_) {
        return false;
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[SCAN-FILTER-CPP] waiting for static TF %s <- %s (%s)",
        base_frame_.c_str(), scan_frame.c_str(), ex.what());
      return false;
    }
  }

  void publish(const sensor_msgs::msg::LaserScan & out)
  {
    pub_->publish(out);
    if (legacy_pub_) {
      legacy_pub_->publish(out);
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (msg->header.frame_id.empty() || msg->ranges.empty()) {
      return;
    }

    if (output_rate_limit_hz_ > 0.0) {
      const auto wall_now = std::chrono::steady_clock::now();
      if (last_accepted_wall_.time_since_epoch().count() != 0) {
        const double elapsed =
          std::chrono::duration<double>(wall_now - last_accepted_wall_).count();
        if (elapsed < (1.0 / output_rate_limit_hz_)) {
          return;
        }
      }
      last_accepted_wall_ = wall_now;
    }

    geometry_msgs::msg::TransformStamped tf;
    const bool have_tf = getTransform(msg->header.frame_id[0] == '/' ? msg->header.frame_id.substr(1) : msg->header.frame_id, tf);
    if (!have_tf && !fail_open_on_tf_error_) {
      return;
    }

    sensor_msgs::msg::LaserScan out = *msg;
    if (max_output_range_ > 0.0) {
      out.range_max = static_cast<float>(std::min<double>(msg->range_max, max_output_range_));
    }

    const float nan = std::numeric_limits<float>::quiet_NaN();
    size_t removed_self = 0;
    size_t removed_range = 0;

    if (have_tf) {
      const auto & q = tf.transform.rotation;
      const auto & t = tf.transform.translation;
      const double r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      const double r01 = 2.0 * (q.x * q.y - q.z * q.w);
      const double r10 = 2.0 * (q.x * q.y + q.z * q.w);
      const double r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);

      double angle = msg->angle_min;
      for (size_t i = 0; i < out.ranges.size(); ++i, angle += msg->angle_increment) {
        const float r = out.ranges[i];
        if (!finiteRange(r, *msg)) {
          continue;
        }
        if (max_output_range_ > 0.0 && r > max_output_range_) {
          out.ranges[i] = nan;
          if (i < out.intensities.size()) {
            out.intensities[i] = 0.0f;
          }
          ++removed_range;
          continue;
        }
        const double sx = static_cast<double>(r) * std::cos(angle);
        const double sy = static_cast<double>(r) * std::sin(angle);
        const double bx = t.x + r00 * sx + r01 * sy;
        const double by = t.y + r10 * sx + r11 * sy;
        if (bx >= min_x_ && bx <= max_x_ && by >= min_y_ && by <= max_y_) {
          out.ranges[i] = nan;
          if (i < out.intensities.size()) {
            out.intensities[i] = 0.0f;
          }
          ++removed_self;
        }
      }
    } else {
      // Fail-open is allowed only for the explicitly configured navigation instance.
      for (size_t i = 0; i < out.ranges.size(); ++i) {
        const float r = out.ranges[i];
        if (std::isfinite(r) && max_output_range_ > 0.0 && r > max_output_range_) {
          out.ranges[i] = nan;
          ++removed_range;
        }
      }
    }

    const std::vector<float> current_for_history = out.ranges;
    size_t removed_denoise = 0;
    if (denoise_enabled_ && out.ranges.size() >= 5U) {
      for (size_t i = 0; i < out.ranges.size(); ++i) {
        const float r = current_for_history[i];
        if (!std::isfinite(r) || r < out.range_min || r > out.range_max) {
          continue;
        }

        std::vector<float> neighbors;
        neighbors.reserve(static_cast<size_t>(2 * median_radius_bins_));
        int close_support = 0;
        for (int d = -median_radius_bins_; d <= median_radius_bins_; ++d) {
          if (d == 0) {
            continue;
          }
          const float n = current_for_history[wrapIndex(static_cast<long>(i) + d, out.ranges.size())];
          if (!std::isfinite(n) || n < out.range_min || n > out.range_max) {
            continue;
          }
          neighbors.push_back(n);
          const double tol = outlier_abs_m_ + outlier_rel_ * std::min<double>(r, n);
          if (std::abs(static_cast<double>(n) - static_cast<double>(r)) <= tol) {
            ++close_support;
          }
        }

        bool temporal_support = false;
        if (previous_ranges_.size() == out.ranges.size()) {
          for (int d = -1; d <= 1; ++d) {
            const float p = previous_ranges_[wrapIndex(static_cast<long>(i) + d, out.ranges.size())];
            if (std::isfinite(p) && std::abs(static_cast<double>(p) - static_cast<double>(r)) <= temporal_jump_m_) {
              temporal_support = true;
              break;
            }
          }
        }

        bool reject = false;
        if (close_support < min_neighbor_support_ && !temporal_support && r >= isolated_min_range_m_) {
          reject = true;
        }
        if (!neighbors.empty()) {
          const float med = median(neighbors);
          if (std::isfinite(med)) {
            const double tol = outlier_abs_m_ + outlier_rel_ * static_cast<double>(med);
            // Starburst artifacts are characteristically isolated FAR returns.
            if (static_cast<double>(r) > static_cast<double>(med) + tol &&
                close_support < min_neighbor_support_ && !temporal_support)
            {
              reject = true;
            }
          }
        }
        if (reject) {
          out.ranges[i] = nan;
          if (i < out.intensities.size()) {
            out.intensities[i] = 0.0f;
          }
          ++removed_denoise;
        }
      }
    }

    // Keep only the already-filtered scan as temporal history. Otherwise a
    // repeated starburst at the same bearing would validate itself next frame.
    previous_ranges_ = out.ranges;
    publish(out);

    if (removed_self + removed_range + removed_denoise > 0U) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "[SCAN-FILTER-CPP] filtered self=%zu beyond_range=%zu isolated=%zu",
        removed_self, removed_range, removed_denoise);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string legacy_output_topic_;
  std::string base_frame_;
  bool fail_open_on_tf_error_{false};
  double min_x_{-0.70};
  double max_x_{0.70};
  double min_y_{-0.45};
  double max_y_{0.45};
  double max_output_range_{0.0};
  bool denoise_enabled_{false};
  int median_radius_bins_{2};
  int min_neighbor_support_{2};
  double outlier_abs_m_{0.35};
  double outlier_rel_{0.08};
  double isolated_min_range_m_{1.0};
  double temporal_jump_m_{0.60};
  double output_rate_limit_hz_{0.0};
  std::chrono::steady_clock::time_point last_accepted_wall_{};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr legacy_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool have_cached_tf_{false};
  geometry_msgs::msg::TransformStamped cached_tf_;
  std::vector<float> previous_ranges_;
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation::ScanSelfFilter>());
  rclcpp::shutdown();
  return 0;
}
