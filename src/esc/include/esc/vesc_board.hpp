#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace esc {

enum class VescRole : std::uint8_t { UNKNOWN=255, LEGACY_MIXED=0, DRIVE_DUAL_HALL=1, STEER_LEFT_ENCODER=2 };

struct VescValues {
  bool valid{false};
  double temp_mos_c{0.0}, current_motor_a{0.0}, current_in_a{0.0};
  double id_a{0.0}, iq_a{0.0}, duty{0.0}, erpm{0.0}, vin_v{0.0};
  double position_deg{0.0}, vd_v{0.0}, vq_v{0.0};
  std::uint8_t fault{0}, vesc_id{0};
  std::uint64_t generation{0};
  std::chrono::steady_clock::time_point stamp{};
};

struct SteeringCal {
  bool valid{false}, calibrated{false}, homed{false}, synced{false}, logical_inverted{false};
  bool encoder_configured{false};
  std::int32_t span{0}, safe_span{0}, position{0}, target{0};
  double steering_deg{0.0}, pos360_deg{0.0};
  std::uint8_t sensor_port{0}, sensor_mode{0}, fault{0};
  std::uint32_t encoder_raw{0};
  std::chrono::steady_clock::time_point stamp{};
};

class VescBoard {
 public:
  explicit VescBoard(std::string port="", int baud=921600);
  ~VescBoard();
  bool poll();
  void close_port();
  bool connected() const { return fd_ >= 0; }
  const std::string &port() const { return port_; }
  VescRole role() const { return role_; }
  std::uint8_t role_caps() const { return role_caps_; }
  bool role_known() const { return role_ != VescRole::UNKNOWN; }
  std::uint64_t connection_generation() const { return connection_generation_; }
  const VescValues &values(bool right=false) const { return values_[right?1:0]; }
  const SteeringCal &steering_cal() const { return steering_cal_; }
  bool values_fresh(bool right, double max_age_s) const;
  bool steering_cal_fresh(double max_age_s) const;
  bool request_platform_info();
  bool request_values(bool right=false);
  bool request_steering_cal();
  bool set_rpm(std::int32_t erpm, bool right=false);
  bool set_steering_deg(double deg);
  bool send_alive(bool right=false);
 private:
  bool open_port();
  bool send_payload(const std::vector<std::uint8_t>& payload);
  bool send_forwarded(std::uint8_t can_id, const std::vector<std::uint8_t>& payload);
  void feed(const std::uint8_t *data, std::size_t n);
  void process_payload(const std::vector<std::uint8_t>& payload);
  bool timed_out() const;
  std::string port_; int baud_{921600}; int fd_{-1};
  std::vector<std::uint8_t> rx_;
  std::array<VescValues,2> values_{};
  SteeringCal steering_cal_{};
  VescRole role_{VescRole::UNKNOWN}; std::uint8_t role_caps_{0};
  std::uint64_t connection_generation_{0};
  std::chrono::steady_clock::time_point last_open_try_{}, opened_at_{}, last_rx_{};
};

const char *vesc_role_name(VescRole role);
}  // namespace esc
