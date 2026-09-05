#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace perception::safety {

inline constexpr const char *NORMAL = "NORMAL";
inline constexpr const char *RECENTER_LEFT = "RECENTER_LEFT";
inline constexpr const char *RECENTER_RIGHT = "RECENTER_RIGHT";
inline constexpr const char *BLOCKED_STOP = "BLOCKED_STOP";
inline constexpr const char *LANE_LOST = "LANE_LOST";

// Menyimpan ambang jarak aman terhadap tepi jalan. Semua jarak diukur dari
// badan kendaraan, bukan dari titik tengah base_link.
struct LaneThresholds {
  double warning_clearance_m{1.0};
  double critical_clearance_m{0.40};
  double release_clearance_m{1.20};
  double center_deadband_m{0.25};

  // Memastikan hysteresis lane mempunyai urutan ambang yang masuk akal.
  void validate() const {
    const double values[] = {
      warning_clearance_m, critical_clearance_m,
      release_clearance_m, center_deadband_m};
    for (const double value : values) {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument("Lane thresholds harus finite dan non-negative");
      }
    }
    if (critical_clearance_m >= warning_clearance_m) {
      throw std::invalid_argument("critical_clearance_m harus lebih kecil dari warning_clearance_m");
    }
    if (release_clearance_m <= warning_clearance_m) {
      throw std::invalid_argument("release_clearance_m harus lebih besar dari warning_clearance_m");
    }
  }
};

struct LaneDecision {
  std::string state{LANE_LOST};
  bool critical{false};
  std::string reason{"lane_geometry_invalid"};
};

// Mengklasifikasikan posisi kendaraan terhadap batas lane dengan hysteresis.
// RECENTER_LEFT berarti kendaraan terlalu dekat ke sisi kanan dan harus bergeser kiri.
inline LaneDecision classifyLaneState(
  bool valid,
  double left_clearance_m,
  double right_clearance_m,
  double center_error_m,
  const std::string &previous_state,
  const LaneThresholds &thresholds)
{
  thresholds.validate();
  if (!valid) {
    return {LANE_LOST, false, "lane_geometry_invalid"};
  }
  if (!std::isfinite(left_clearance_m) || !std::isfinite(right_clearance_m) ||
      !std::isfinite(center_error_m)) {
    return {LANE_LOST, false, "lane_geometry_non_finite"};
  }

  const bool critical =
    std::min(left_clearance_m, right_clearance_m) <= thresholds.critical_clearance_m;

  if (previous_state == RECENTER_LEFT &&
      (right_clearance_m < thresholds.release_clearance_m ||
       center_error_m > thresholds.center_deadband_m)) {
    return {RECENTER_LEFT, critical, "right_edge_hysteresis"};
  }
  if (previous_state == RECENTER_RIGHT &&
      (left_clearance_m < thresholds.release_clearance_m ||
       center_error_m < -thresholds.center_deadband_m)) {
    return {RECENTER_RIGHT, critical, "left_edge_hysteresis"};
  }

  const bool left_warning = left_clearance_m < thresholds.warning_clearance_m;
  const bool right_warning = right_clearance_m < thresholds.warning_clearance_m;
  if (left_warning && right_warning) {
    if (center_error_m > thresholds.center_deadband_m) {
      return {RECENTER_LEFT, critical, "both_edges_center_is_left"};
    }
    if (center_error_m < -thresholds.center_deadband_m) {
      return {RECENTER_RIGHT, critical, "both_edges_center_is_right"};
    }
    if (right_clearance_m < left_clearance_m) {
      return {RECENTER_LEFT, critical, "both_edges_right_is_closer"};
    }
    return {RECENTER_RIGHT, critical, "both_edges_left_is_closer"};
  }
  if (right_warning) return {RECENTER_LEFT, critical, "right_edge_warning"};
  if (left_warning) return {RECENTER_RIGHT, critical, "left_edge_warning"};
  return {NORMAL, critical, "inside_safety_area"};
}

// Filter konfirmasi frame. State baru harus konsisten beberapa frame agar noise
// satu frame tidak membuat steering berosilasi atau rem berhenti palsu.
class ConfirmedState {
public:
  ConfirmedState(
    std::string initial_state = LANE_LOST,
    int state_confirm_frames = 3,
    int release_confirm_frames = 5,
    int lost_confirm_frames = 2)
  : state_(std::move(initial_state)), candidate_(state_),
    state_confirm_frames_(std::max(1, state_confirm_frames)),
    release_confirm_frames_(std::max(1, release_confirm_frames)),
    lost_confirm_frames_(std::max(1, lost_confirm_frames)) {}

  // Memasukkan kandidat state terbaru dan mengembalikan state terkonfirmasi.
  const std::string &update(const std::string &candidate) {
    if (candidate == state_) {
      candidate_ = candidate;
      count_ = 0;
      return state_;
    }
    if (candidate != candidate_) {
      candidate_ = candidate;
      count_ = 1;
    } else {
      ++count_;
    }
    int required = state_confirm_frames_;
    if (candidate == NORMAL) required = release_confirm_frames_;
    else if (candidate == LANE_LOST) required = lost_confirm_frames_;
    if (count_ >= required) {
      state_ = candidate;
      candidate_ = candidate;
      count_ = 0;
    }
    return state_;
  }

  // Mengembalikan state lane yang saat ini sudah lolos filter konfirmasi.
  const std::string &state() const { return state_; }

private:
  std::string state_;
  std::string candidate_;
  int count_{0};
  int state_confirm_frames_{3};
  int release_confirm_frames_{5};
  int lost_confirm_frames_{2};
};

struct ObstacleMetric {
  int track_id{-1};
  double forward_m{0.0};
  double left_m{0.0};
  double width_m{0.20};
};

struct ObstacleGateConfig {
  double vehicle_width_m{0.55};
  double lateral_margin_m{0.30};
  double forward_min_m{0.20};
  double forward_max_m{3.0};
  double minimum_obstacle_width_m{0.20};

  // Memvalidasi geometri koridor recenter sebelum dipakai untuk keputusan safety.
  void validate() const {
    if (vehicle_width_m <= 0.0) throw std::invalid_argument("vehicle_width_m harus positif");
    if (lateral_margin_m < 0.0) throw std::invalid_argument("lateral_margin_m tidak boleh negatif");
    if (forward_min_m < 0.0 || forward_max_m <= forward_min_m) {
      throw std::invalid_argument("rentang forward obstacle tidak valid");
    }
    if (minimum_obstacle_width_m < 0.0) {
      throw std::invalid_argument("minimum_obstacle_width_m tidak boleh negatif");
    }
  }
};

struct ObstacleGateResult {
  bool blocked{false};
  std::vector<int> blocking_track_ids;
  double swept_min_m{0.0};
  double swept_max_m{0.0};
};

// Menguji apakah koridor lateral dari posisi kendaraan sekarang menuju center lane
// berpotongan dengan obstacle dalam rentang forward yang diperiksa.
inline ObstacleGateResult obstacleBlocksRecenter(
  double target_center_left_m,
  const std::vector<ObstacleMetric> &obstacles,
  const ObstacleGateConfig &config)
{
  config.validate();
  const double half_vehicle = 0.5 * config.vehicle_width_m;
  ObstacleGateResult result;
  result.swept_min_m = std::min(-half_vehicle, target_center_left_m - half_vehicle) -
    config.lateral_margin_m;
  result.swept_max_m = std::max(half_vehicle, target_center_left_m + half_vehicle) +
    config.lateral_margin_m;

  for (const auto &obstacle : obstacles) {
    if (!std::isfinite(obstacle.forward_m) || !std::isfinite(obstacle.left_m)) continue;
    if (obstacle.forward_m < config.forward_min_m || obstacle.forward_m > config.forward_max_m) continue;
    if (!std::isfinite(obstacle.width_m) || obstacle.width_m < config.minimum_obstacle_width_m) continue;
    const double width = obstacle.width_m;
    const double obstacle_min = obstacle.left_m - 0.5 * width;
    const double obstacle_max = obstacle.left_m + 0.5 * width;
    if (obstacle_max >= result.swept_min_m && obstacle_min <= result.swept_max_m) {
      result.blocking_track_ids.push_back(obstacle.track_id);
    }
  }
  result.blocked = !result.blocking_track_ids.empty();
  return result;
}

struct MixerConfig {
  double wheelbase_m{0.70};
  double maximum_steering_angle_rad{0.34};
  double center_gain{0.55};
  double heading_gain{0.70};
  double lane_blend_gain{1.0};
  double recenter_speed_mps{0.15};
  double critical_recenter_speed_mps{0.10};
  double minimum_speed_for_yaw_limit_mps{0.10};

  // Memastikan batas Ackermann dan gain utama tidak menghasilkan geometri tak valid.
  void validate() const {
    if (wheelbase_m <= 0.0) throw std::invalid_argument("wheelbase_m harus positif");
    if (!(maximum_steering_angle_rad > 0.0 && maximum_steering_angle_rad < 1.57079632679489661923)) {
      throw std::invalid_argument("maximum_steering_angle_rad harus berada di (0, pi/2)");
    }
    if (recenter_speed_mps < 0.0 || critical_recenter_speed_mps < 0.0) {
      throw std::invalid_argument("recenter speed tidak boleh negatif");
    }
    if (center_gain < 0.0 || heading_gain < 0.0 || lane_blend_gain < 0.0) {
      throw std::invalid_argument("gain mixer tidak boleh negatif");
    }
  }
};

// Hasil mixer memakai aggregate bernama, bukan std::pair. Selain lebih jelas di
// call-site, ini menghindari diagnostic psABI GCC 10.x pada return std::pair
// ketika test header dikompilasi sebagai C++17.
struct MixerCommand
{
  double linear_x = 0.0;
  double angular_z = 0.0;
};

// Mencampur perintah Nav2 dengan koreksi lane dan membatasi yaw-rate sesuai
// geometri Ackermann: yaw_rate <= v*tan(delta_max)/wheelbase.
inline MixerCommand mixRecenterCommand(
  double nav_linear_x,
  double nav_angular_z,
  double center_error_m,
  double heading_error_rad,
  bool critical,
  const MixerConfig &config)
{
  config.validate();
  if (!std::isfinite(nav_linear_x)) nav_linear_x = 0.0;
  if (!std::isfinite(nav_angular_z)) nav_angular_z = 0.0;
  if (!std::isfinite(center_error_m)) center_error_m = 0.0;
  if (!std::isfinite(heading_error_rad)) heading_error_rad = 0.0;
  nav_linear_x = std::max(0.0, nav_linear_x);

  const double speed_limit = critical ?
    config.critical_recenter_speed_mps : config.recenter_speed_mps;
  const double output_linear = std::min(nav_linear_x, speed_limit);
  if (output_linear <= 1.0e-4) return {0.0, 0.0};

  const double lane_term = config.center_gain * center_error_m +
    config.heading_gain * heading_error_rad;
  const double requested_angular = nav_angular_z + config.lane_blend_gain * lane_term;
  const double curvature_max = std::tan(config.maximum_steering_angle_rad) / config.wheelbase_m;
  const double yaw_limit_speed = std::max(output_linear, config.minimum_speed_for_yaw_limit_mps);
  const double maximum_yaw_rate = yaw_limit_speed * curvature_max;
  return {output_linear, std::clamp(requested_angular, -maximum_yaw_rate, maximum_yaw_rate)};
}

}  // namespace perception::safety
