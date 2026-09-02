#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <functional>
#include <memory>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace navigation
{

class GoalPoseNav2Bridge final : public rclcpp::Node
{
public:
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using Navigate = nav2_msgs::action::NavigateToPose;
  using ComputeGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;

  GoalPoseNav2Bridge()
  : Node("goal_pose_nav2_bridge_cpp")
  {
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    compute_action_name_ = declare_parameter<std::string>("compute_path_action_name", "/compute_path_to_pose");
    navigate_action_name_ = declare_parameter<std::string>("navigate_action_name", "/navigate_to_pose");
    planner_id_ = declare_parameter<std::string>("planner_id", "GridBased");
    primary_bt_ = declare_parameter<std::string>("primary_behavior_tree", "");
    plan_source_topic_ = declare_parameter<std::string>("plan_source_topic", "/plan");
    plan_topic_ = declare_parameter<std::string>("plan_topic", "/smac_plan");
    map_topic_ = declare_parameter<std::string>("nav_map_topic", "/map");
    amcl_topic_ = declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    initialpose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose_safe");
    default_frame_ = declare_parameter<std::string>("default_frame", "map");
    reject_unknown_ = declare_parameter<bool>("reject_unknown_goal", true);
    free_threshold_ = static_cast<int>(declare_parameter<int>("goal_free_threshold", 20));
    use_explicit_amcl_start_ = declare_parameter<bool>("use_explicit_amcl_start", true);
    amcl_max_age_sec_ = declare_parameter<double>("amcl_max_age_sec", 3.0);
    initialpose_max_age_sec_ = declare_parameter<double>("initial_pose_max_age_sec", 30.0);
    snap_goal_to_free_ = declare_parameter<bool>("snap_goal_to_free", true);
    snap_radius_m_ = declare_parameter<double>("snap_radius_m", 0.80);
    footprint_half_length_ = declare_parameter<double>("footprint_half_length", 0.65);
    footprint_half_width_ = declare_parameter<double>("footprint_half_width", 0.40);
    max_plan_retries_ = static_cast<int>(declare_parameter<int>("max_plan_retries", 8));
    plan_retry_period_sec_ = declare_parameter<double>("plan_retry_period_sec", 0.75);
    plan_request_timeout_sec_ = declare_parameter<double>("plan_request_timeout_sec", 7.0);
    nav_goal_response_timeout_sec_ = declare_parameter<double>("nav_goal_response_timeout_sec", 5.0);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_ = *msg; });

    // AMCL QoS differs across Nav2 / vendor builds.  Keep two readers so a
    // stationary pose is never missed merely because one side is TRANSIENT_LOCAL
    // while the other is VOLATILE/BEST_EFFORT.  Both callbacks update the same
    // latest sample and are harmless duplicates.
    auto amcl_transient_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    auto amcl_fallback_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    amcl_sub_transient_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_topic_, amcl_transient_qos,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        amcl_pose_ = *msg;
        amcl_received_ = now();
      });
    amcl_sub_fallback_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_topic_, amcl_fallback_qos,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        amcl_pose_ = *msg;
        amcl_received_ = now();
      });

    // The operator's explicit Initial Pose is a safe temporary planning start
    // while AMCL is processing its first scan.  It is never used for motion
    // authorization; it only prevents the path preview from being needlessly
    // blocked during the short AMCL hand-off.
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        initial_pose_ = *msg;
        initial_pose_received_ = now();
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&GoalPoseNav2Bridge::goalCallback, this, std::placeholders::_1));

    actual_plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      plan_source_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&GoalPoseNav2Bridge::actualPlanCallback, this, std::placeholders::_1));

    plan_pub_ = create_publisher<nav_msgs::msg::Path>(plan_topic_, path_qos);
    goal_event_pub_ = create_publisher<std_msgs::msg::String>("/navigation/goal_event", 10);
    plan_metrics_pub_ = create_publisher<std_msgs::msg::String>("/navigation/plan_metrics", 10);
    planner_status_pub_ = create_publisher<std_msgs::msg::String>("/navigation/planner_status", 10);

    compute_client_ = rclcpp_action::create_client<ComputePath>(this, compute_action_name_);
    navigate_client_ = rclcpp_action::create_client<Navigate>(this, navigate_action_name_);

    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&GoalPoseNav2Bridge::pump, this));

    publishStatus("WAITING", "bridge ready; waiting for map/planner");
    RCLCPP_INFO(
      get_logger(),
      "[GOAL-BRIDGE-CPP] READY %s -> %s planner=%s -> %s -> NavigateToPose",
      goal_topic_.c_str(), compute_action_name_.c_str(), planner_id_.c_str(), plan_topic_.c_str());
  }

private:
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny, cosy);
  }

  static double pathLength(const nav_msgs::msg::Path & path)
  {
    double total = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position;
      const auto & b = path.poses[i].pose.position;
      total += std::hypot(b.x - a.x, b.y - a.y);
    }
    return total;
  }

  static std::string jsonEscape(const std::string & in)
  {
    std::string out;
    out.reserve(in.size() + 8);
    for (char c : in) {
      switch (c) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default: out += c; break;
      }
    }
    return out;
  }

  void publishString(const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub, const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    pub->publish(msg);
  }

  void publishStatus(const std::string & state, const std::string & detail)
  {
    std::ostringstream ss;
    ss << "{\"state\":\"" << jsonEscape(state) << "\",\"detail\":\""
       << jsonEscape(detail) << "\",\"generation\":" << generation_ << "}";
    publishString(planner_status_pub_, ss.str());
  }

  void publishGoalEvent(const std::string & event, const std::string & reason = "")
  {
    std::ostringstream ss;
    ss << "{\"event\":\"" << jsonEscape(event) << "\",\"generation\":" << generation_;
    if (pending_goal_) {
      ss << ",\"x\":" << std::fixed << std::setprecision(3) << pending_goal_->pose.position.x
         << ",\"y\":" << pending_goal_->pose.position.y;
    }
    if (!reason.empty()) {
      ss << ",\"reason\":\"" << jsonEscape(reason) << "\"";
    }
    ss << "}";
    publishString(goal_event_pub_, ss.str());
  }

  void clearPreview()
  {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = default_frame_;
    empty.header.stamp = now();
    plan_pub_->publish(empty);
  }

  bool worldToMap(const double wx, const double wy, int & mx, int & my) const
  {
    if (!map_ || map_->info.resolution <= 0.0 || map_->info.width == 0 || map_->info.height == 0) {
      return false;
    }
    const auto & origin = map_->info.origin;
    const double yaw = yawFromQuaternion(origin.orientation);
    const double dx = wx - origin.position.x;
    const double dy = wy - origin.position.y;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double lx = c * dx + s * dy;
    const double ly = -s * dx + c * dy;
    mx = static_cast<int>(std::floor(lx / map_->info.resolution));
    my = static_cast<int>(std::floor(ly / map_->info.resolution));
    return mx >= 0 && my >= 0 && mx < static_cast<int>(map_->info.width) && my < static_cast<int>(map_->info.height);
  }

  bool cellFree(const int mx, const int my) const
  {
    if (!map_ || mx < 0 || my < 0 || mx >= static_cast<int>(map_->info.width) || my >= static_cast<int>(map_->info.height)) {
      return false;
    }
    const int8_t value = map_->data[static_cast<size_t>(my) * map_->info.width + static_cast<size_t>(mx)];
    if (value < 0) {
      return !reject_unknown_;
    }
    return value <= free_threshold_;
  }

  bool footprintFree(const geometry_msgs::msg::PoseStamped & pose) const
  {
    if (!map_) {
      return false;
    }
    const double res = map_->info.resolution;
    const double yaw = yawFromQuaternion(pose.pose.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double step = std::max(0.025, std::min(res, 0.05));

    for (double x = -footprint_half_length_; x <= footprint_half_length_ + 1e-6; x += step) {
      for (double y = -footprint_half_width_; y <= footprint_half_width_ + 1e-6; y += step) {
        const double wx = pose.pose.position.x + c * x - s * y;
        const double wy = pose.pose.position.y + s * x + c * y;
        int mx = 0;
        int my = 0;
        if (!worldToMap(wx, wy, mx, my) || !cellFree(mx, my)) {
          return false;
        }
      }
    }
    return true;
  }

  bool validateAndMaybeSnapGoal(geometry_msgs::msg::PoseStamped & goal, std::string & reason)
  {
    if (!map_) {
      reason = "planning map has not arrived";
      return false;
    }
    const std::string map_frame = map_->header.frame_id.empty() ? default_frame_ : map_->header.frame_id;
    if (goal.header.frame_id.empty()) {
      goal.header.frame_id = map_frame;
    }
    if (goal.header.frame_id != map_frame) {
      reason = "goal frame does not match planning map frame";
      return false;
    }
    if (footprintFree(goal)) {
      reason = "goal footprint is free";
      return true;
    }
    if (!snap_goal_to_free_) {
      reason = "goal footprint intersects occupied/unknown/outside-map cells";
      return false;
    }

    const double res = map_->info.resolution;
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(snap_radius_m_ / res)));
    geometry_msgs::msg::PoseStamped best = goal;
    double best_d2 = std::numeric_limits<double>::infinity();
    bool found = false;

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const double ox = static_cast<double>(dx) * res;
        const double oy = static_cast<double>(dy) * res;
        const double d2 = ox * ox + oy * oy;
        if (d2 > snap_radius_m_ * snap_radius_m_ || d2 >= best_d2) {
          continue;
        }
        auto candidate = goal;
        candidate.pose.position.x += ox;
        candidate.pose.position.y += oy;
        if (footprintFree(candidate)) {
          best = candidate;
          best_d2 = d2;
          found = true;
        }
      }
    }
    if (!found) {
      reason = "no footprint-safe goal found within snap radius";
      return false;
    }
    const double moved = std::sqrt(best_d2);
    goal = best;
    std::ostringstream ss;
    ss << "goal snapped " << std::fixed << std::setprecision(2) << moved << " m to footprint-safe cell";
    reason = ss.str();
    return true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    ++generation_;
    plan_retries_ = 0;
    planning_in_flight_ = false;
    nav_send_in_flight_ = false;
    preview_ready_ = false;
    last_retry_ = std::chrono::steady_clock::time_point{};

    // Cancel any previous planner/navigation request. Generation guards below
    // also prevent stale callbacks from overwriting the new Goal Pose.
    if (active_compute_goal_ || planning_in_flight_) {
      compute_client_->async_cancel_all_goals();
      active_compute_goal_.reset();
    }
    if (active_nav_goal_ || nav_send_in_flight_) {
      navigate_client_->async_cancel_all_goals();
      active_nav_goal_.reset();
    }

    geometry_msgs::msg::PoseStamped goal = *msg;
    if (goal.header.frame_id.empty()) {
      goal.header.frame_id = default_frame_;
    }
    goal.header.stamp = now();
    pending_goal_ = goal;
    clearPreview();
    publishGoalEvent("REQUESTED");
    publishStatus("GOAL_RECEIVED", "goal queued for direct Smac Hybrid-A* preview");

    RCLCPP_INFO(
      get_logger(), "[GOAL-BRIDGE-CPP] goal #%lu x=%.3f y=%.3f queued",
      static_cast<unsigned long>(generation_), goal.pose.position.x, goal.pose.position.y);
  }

  void actualPlanCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.size() < 2) {
      return;
    }
    auto out = *msg;
    if (out.header.frame_id.empty()) {
      out.header.frame_id = default_frame_;
    }
    plan_pub_->publish(out);
    publishPlanMetrics(out, "NAV2_ACTUAL");
  }

  void publishPlanMetrics(const nav_msgs::msg::Path & path, const std::string & source)
  {
    std::ostringstream ss;
    ss << "{\"generation\":" << generation_
       << ",\"source\":\"" << source << "\""
       << ",\"pose_count\":" << path.poses.size()
       << ",\"path_length_m\":" << std::fixed << std::setprecision(3) << pathLength(path) << "}";
    publishString(plan_metrics_pub_, ss.str());
  }

  bool amclStartUsable() const
  {
    if (!use_explicit_amcl_start_ || !amcl_pose_) {
      return false;
    }
    const auto age = (now() - amcl_received_).seconds();
    if (age < 0.0 || age > amcl_max_age_sec_) {
      return false;
    }
    const std::string frame = amcl_pose_->header.frame_id.empty() ? default_frame_ : amcl_pose_->header.frame_id;
    return frame == default_frame_ || (map_ && frame == map_->header.frame_id);
  }

  bool initialPoseStartUsable() const
  {
    if (!use_explicit_amcl_start_ || !initial_pose_) {
      return false;
    }
    const auto age = (now() - initial_pose_received_).seconds();
    if (age < 0.0 || age > initialpose_max_age_sec_) {
      return false;
    }
    const std::string frame = initial_pose_->header.frame_id.empty() ? default_frame_ : initial_pose_->header.frame_id;
    return frame == default_frame_ || (map_ && frame == map_->header.frame_id);
  }

  void startPlanning()
  {
    if (!pending_goal_ || planning_in_flight_) {
      return;
    }

    auto goal_pose = *pending_goal_;
    std::string validation;
    if (!validateAndMaybeSnapGoal(goal_pose, validation)) {
      // Map not arrived is a wait state. Everything else is a deterministic rejection.
      if (!map_) {
        publishStatus("WAIT_MAP", validation);
        return;
      }
      publishGoalEvent("REJECTED", validation);
      publishStatus("GOAL_REJECTED", validation);
      RCLCPP_ERROR(get_logger(), "[GOAL-BRIDGE-CPP] goal rejected: %s", validation.c_str());
      pending_goal_.reset();
      clearPreview();
      return;
    }
    if (goal_pose.pose.position.x != pending_goal_->pose.position.x ||
        goal_pose.pose.position.y != pending_goal_->pose.position.y)
    {
      pending_goal_ = goal_pose;
      publishGoalEvent("SNAPPED", validation);
      RCLCPP_INFO(get_logger(), "[GOAL-BRIDGE-CPP] %s", validation.c_str());
    }

    if (!compute_client_->action_server_is_ready()) {
      publishStatus("WAIT_PLANNER", "waiting for /compute_path_to_pose; PlannerServer lifecycle not ACTIVE yet");
      return;
    }

    // Never burn Smac retries on a missing start pose.  With an explicit start
    // requested, wait until either AMCL or the latest operator Initial Pose is
    // available in the map frame.  This is the common startup condition on a
    // stationary AGV and should be a WAIT state, not a planning failure.
    if (use_explicit_amcl_start_ && !amclStartUsable() && !initialPoseStartUsable()) {
      publishStatus("WAIT_LOCALIZATION", "planner ACTIVE; waiting for fresh /amcl_pose or /initialpose_safe start pose");
      return;
    }

    if (max_plan_retries_ > 0 && plan_retries_ >= max_plan_retries_) {
      // A planner/costmap lifecycle transition can be temporarily unavailable
      // during autonomous startup.  Do not discard the operator Goal after a
      // fixed retry count; keep it queued and begin a fresh bounded retry cycle.
      // Impossible goals remain visible as WAIT_REPLAN and can be replaced by
      // the next Goal Pose, while transient startup races recover automatically.
      plan_retries_ = 0;
      last_retry_ = std::chrono::steady_clock::now();
      const std::string why =
        "Smac retry cycle exhausted; Goal retained and will retry when planner/costmap are ready";
      publishGoalEvent("WAIT_REPLAN", why);
      publishStatus("WAIT_REPLAN", why);
      RCLCPP_WARN(get_logger(), "[GOAL-BRIDGE-CPP] %s", why.c_str());
      return;
    }

    ++plan_retries_;
    planning_in_flight_ = true;
    planning_started_ = std::chrono::steady_clock::now();
    const uint64_t request_generation = generation_;

    ComputePath::Goal plan_goal;
    plan_goal.goal = *pending_goal_;
    plan_goal.goal.header.stamp = now();
    plan_goal.planner_id = planner_id_;
    if (amclStartUsable()) {
      plan_goal.start.header = amcl_pose_->header;
      if (plan_goal.start.header.frame_id.empty()) {
        plan_goal.start.header.frame_id = default_frame_;
      }
      plan_goal.start.header.stamp = now();
      plan_goal.start.pose = amcl_pose_->pose.pose;
      plan_goal.use_start = true;
    } else if (initialPoseStartUsable()) {
      plan_goal.start.header = initial_pose_->header;
      if (plan_goal.start.header.frame_id.empty()) {
        plan_goal.start.header.frame_id = default_frame_;
      }
      plan_goal.start.header.stamp = now();
      plan_goal.start.pose = initial_pose_->pose.pose;
      plan_goal.use_start = true;
      publishStatus("PLANNING", "using latest explicit Initial Pose while AMCL settles");
    } else {
      plan_goal.use_start = false;
    }

    publishStatus("PLANNING", "ComputePathToPose request sent to Smac Hybrid-A*");
    publishGoalEvent("PLANNING");

    auto options = rclcpp_action::Client<ComputePath>::SendGoalOptions();
    options.goal_response_callback =
      [this, request_generation](ComputeGoalHandle::SharedPtr handle) {
        if (request_generation != generation_) {
          return;
        }
        if (!handle) {
          planning_in_flight_ = false;
          last_retry_ = std::chrono::steady_clock::now();
          publishStatus("PLAN_RETRY", "PlannerServer rejected ComputePathToPose request");
          return;
        }
        active_compute_goal_ = handle;
      };
    options.result_callback =
      [this, request_generation](const ComputeGoalHandle::WrappedResult & wrapped) {
        if (request_generation != generation_) {
          return;
        }
        planning_in_flight_ = false;
        active_compute_goal_.reset();

        if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
            wrapped.result->path.poses.size() < 2)
        {
          last_retry_ = std::chrono::steady_clock::now();
          std::ostringstream why;
          why << "Smac path unavailable (attempt " << plan_retries_ << "/" << max_plan_retries_ << ")";
          publishStatus("PLAN_RETRY", why.str());
          publishGoalEvent("PLAN_RETRY", why.str());
          RCLCPP_WARN(get_logger(), "[GOAL-BRIDGE-CPP] %s", why.str().c_str());
          return;
        }

        auto path = wrapped.result->path;
        if (path.header.frame_id.empty()) {
          path.header.frame_id = default_frame_;
        }
        plan_pub_->publish(path);
        publishPlanMetrics(path, "SMAC_PREVIEW");
        preview_ready_ = true;
        publishGoalEvent("PATH_READY");

        std::ostringstream detail;
        detail << "Smac path READY: " << path.poses.size() << " poses, "
               << std::fixed << std::setprecision(2) << pathLength(path) << " m";
        publishStatus("PATH_READY", detail.str());
        RCLCPP_INFO(get_logger(), "[SMAC-HYBRID-ASTAR] %s", detail.str().c_str());
      };

    compute_client_->async_send_goal(plan_goal, options);
  }

  void startNavigation()
  {
    if (!pending_goal_ || !preview_ready_ || active_nav_goal_ || nav_send_in_flight_) {
      return;
    }
    if (!navigate_client_->action_server_is_ready()) {
      publishStatus("PATH_READY_WAIT_NAV", "path visible; waiting for BT Navigator lifecycle ACTIVE");
      return;
    }

    nav_send_in_flight_ = true;
    nav_send_started_ = std::chrono::steady_clock::now();
    const uint64_t request_generation = generation_;
    Navigate::Goal nav_goal;
    nav_goal.pose = *pending_goal_;
    nav_goal.pose.header.stamp = now();
    if (!primary_bt_.empty()) {
      nav_goal.behavior_tree = primary_bt_;
    }

    auto options = rclcpp_action::Client<Navigate>::SendGoalOptions();
    options.goal_response_callback =
      [this, request_generation](NavigateGoalHandle::SharedPtr handle) {
        if (request_generation != generation_) {
          return;
        }
        nav_send_in_flight_ = false;
        if (!handle) {
          publishStatus("NAV_REJECTED", "BT Navigator rejected NavigateToPose");
          publishGoalEvent("NAV_REJECTED", "BT Navigator rejected NavigateToPose");
          return;
        }
        active_nav_goal_ = handle;
        publishGoalEvent("ACCEPTED");
        publishStatus("NAVIGATING", "preview path accepted; MPPI navigation started");
        RCLCPP_INFO(get_logger(), "[GOAL-BRIDGE-CPP] NavigateToPose accepted; MPPI control may start");
      };
    options.result_callback =
      [this, request_generation](const NavigateGoalHandle::WrappedResult & wrapped) {
        if (request_generation != generation_) {
          return;
        }
        std::string state;
        switch (wrapped.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: state = "SUCCEEDED"; break;
          case rclcpp_action::ResultCode::ABORTED: state = "ABORTED"; break;
          case rclcpp_action::ResultCode::CANCELED: state = "CANCELED"; break;
          default: state = "UNKNOWN"; break;
        }
        publishGoalEvent("RESULT", state);
        publishStatus("NAV_" + state, "NavigateToPose result=" + state);
        active_nav_goal_.reset();
        pending_goal_.reset();
        preview_ready_ = false;
      };

    navigate_client_->async_send_goal(nav_goal, options);
  }

  void pump()
  {
    if (!pending_goal_) {
      return;
    }

    if (!preview_ready_) {
      if (planning_in_flight_) {
        const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - planning_started_).count();
        if (elapsed <= plan_request_timeout_sec_) {
          return;
        }
        compute_client_->async_cancel_all_goals();
        active_compute_goal_.reset();
        planning_in_flight_ = false;
        last_retry_ = std::chrono::steady_clock::now();
        publishStatus("PLAN_RETRY", "ComputePathToPose watchdog timeout; retrying PlannerServer");
        RCLCPP_WARN(get_logger(), "[GOAL-BRIDGE-CPP] ComputePathToPose timeout after %.1f s", elapsed);
        return;
      }
      if (last_retry_.time_since_epoch().count() != 0) {
        const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_retry_).count();
        if (elapsed < plan_retry_period_sec_) {
          return;
        }
      }
      startPlanning();
      return;
    }
    if (nav_send_in_flight_) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - nav_send_started_).count();
      if (elapsed > nav_goal_response_timeout_sec_) {
        navigate_client_->async_cancel_all_goals();
        nav_send_in_flight_ = false;
        publishStatus("PATH_READY_WAIT_NAV", "NavigateToPose response timeout; retrying BT Navigator");
      }
      return;
    }
    startNavigation();
  }

  std::string goal_topic_;
  std::string compute_action_name_;
  std::string navigate_action_name_;
  std::string planner_id_;
  std::string primary_bt_;
  std::string plan_source_topic_;
  std::string plan_topic_;
  std::string map_topic_;
  std::string amcl_topic_;
  std::string initialpose_topic_;
  std::string default_frame_;
  bool reject_unknown_{true};
  int free_threshold_{20};
  bool use_explicit_amcl_start_{true};
  double amcl_max_age_sec_{3.0};
  double initialpose_max_age_sec_{30.0};
  bool snap_goal_to_free_{true};
  double snap_radius_m_{0.8};
  double footprint_half_length_{0.65};
  double footprint_half_width_{0.40};
  int max_plan_retries_{8};
  double plan_retry_period_sec_{0.75};
  double plan_request_timeout_sec_{7.0};
  double nav_goal_response_timeout_sec_{5.0};

  std::optional<nav_msgs::msg::OccupancyGrid> map_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> amcl_pose_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> initial_pose_;
  rclcpp::Time amcl_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time initial_pose_received_{0, 0, RCL_ROS_TIME};
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_;
  uint64_t generation_{0};
  int plan_retries_{0};
  bool planning_in_flight_{false};
  bool preview_ready_{false};
  bool nav_send_in_flight_{false};
  std::chrono::steady_clock::time_point last_retry_{};
  std::chrono::steady_clock::time_point planning_started_{};
  std::chrono::steady_clock::time_point nav_send_started_{};

  ComputeGoalHandle::SharedPtr active_compute_goal_;
  NavigateGoalHandle::SharedPtr active_nav_goal_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_transient_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_fallback_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr actual_plan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr plan_metrics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_status_pub_;
  rclcpp_action::Client<ComputePath>::SharedPtr compute_client_;
  rclcpp_action::Client<Navigate>::SharedPtr navigate_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation::GoalPoseNav2Bridge>());
  rclcpp::shutdown();
  return 0;
}
