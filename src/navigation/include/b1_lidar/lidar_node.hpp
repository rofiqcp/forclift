// B1LiDAR C++ Node - YDLiDAR X2 / T-mini Plus / G-series driver
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <termios.h>
#include <thread>
#include <vector>

namespace b1_lidar
{

struct ScanPoint
{
  double angle_deg{0.0};
  double distance_m{0.0};
  uint16_t intensity{0};
  bool valid{false};
};

struct ScanData
{
  std::vector<ScanPoint> points;
  rclcpp::Time stamp;
  bool ring_start{false};
  bool checksum_ok{false};
};

class SerialPort
{
public:
  SerialPort(std::string port, int baudrate);
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool open();
  void close();
  bool is_open() const;

  // Non-blocking-friendly read with short deadline (ms).
  std::vector<uint8_t> read(size_t size, int timeout_ms = 50);
  ssize_t write(const uint8_t * data, size_t size);
  int available() const;
  void flush();
  bool set_motor_power(bool enabled);
  bool path_is_healthy() const;

  static speed_t baud_to_constant(int baud);
  static bool path_exists(const std::string & path);
  static bool is_port_accessible(const std::string & path);

private:
  std::string port_;
  int baudrate_{230400};
  int fd_{-1};
  mutable std::mutex mutex_;
  std::atomic<bool> io_error_{false};
};

class LiDARConnector
{
public:
  enum class State { Disconnected, Connecting, Connected, Reconnecting, Error };

  using ConnectCallback = std::function<void()>;
  using DisconnectCallback = std::function<void()>;
  using ErrorCallback = std::function<void(const std::string &)>;

  LiDARConnector(std::string port, int baudrate);
  ~LiDARConnector();

  bool connect();
  void disconnect();
  void start_auto_reconnect();
  void stop_auto_reconnect();
  void request_reconnect(const std::string & reason);

  void set_on_connected(ConnectCallback cb) { on_connected_ = std::move(cb); }
  void set_on_disconnected(DisconnectCallback cb) { on_disconnected_ = std::move(cb); }
  void set_on_error(ErrorCallback cb) { on_error_ = std::move(cb); }

  State state() const { return state_.load(); }
  const std::string & port() const { return port_; }
  int baudrate() const { return baudrate_; }

  std::vector<uint8_t> read(size_t size, int timeout_ms = 50);
  ssize_t write(const uint8_t * data, size_t size);
  void flush();
  bool set_motor_power(bool enabled);
  bool is_open() const;

private:
  void reconnect_loop();

  std::string port_;
  int baudrate_{230400};
  // Protect ownership of the SerialPort across the reader, watchdog and
  // reconnect worker. The old code could reset serial_ while another thread
  // was inside read()/write(), which is undefined behaviour and can surface as
  // intermittent "node up / no scan" stalls.
  mutable std::mutex serial_owner_mutex_;
  // Serialize connect/disconnect/reconnect ownership transitions themselves.
  // serial_owner_mutex_ protects dereference lifetime; lifecycle_mutex_ stops
  // two recovery threads from opening/replacing the port concurrently.
  mutable std::mutex lifecycle_mutex_;
  std::unique_ptr<SerialPort> serial_;
  std::atomic<State> state_{State::Disconnected};

  ConnectCallback on_connected_;
  DisconnectCallback on_disconnected_;
  ErrorCallback on_error_;

  std::thread reconnect_thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> reconnect_attempts_{0};
  static constexpr int kMaxReconnectAttempts = 100;
  static constexpr double kReconnectIntervalSec = 2.0;
};

class LiDARMotor
{
public:
  explicit LiDARMotor(LiDARConnector & connector);

  bool start_scan(int wait_ms = 1500);
  bool stop_scan();
  bool get_scan_frequency(double & hz, int timeout_ms = 700);
  bool set_scan_frequency(double target_hz, double & actual_hz);
  void mark_stream_running();
  std::string get_motor_state() const;

private:
  bool probe_for_sync(int wait_ms);
  bool command_scan_frequency(uint8_t command, double & hz, int timeout_ms);

  LiDARConnector & connector_;
  mutable std::mutex mutex_;
  std::string motor_state_{"UNKNOWN"};
};

class ScanReader
{
public:
  using ScanCallback = std::function<void(const ScanData &)>;

  ScanReader(
    LiDARConnector & connector, bool intensity_mode, int intensity_bits, bool strict_checksum);
  ~ScanReader();

  void start();
  void stop();
  void reset_accumulator();
  void set_on_scan_received(ScanCallback cb) { on_scan_received_ = std::move(cb); }

  uint64_t valid_packets() const { return valid_packets_.load(); }
  uint64_t invalid_packets() const { return invalid_packets_.load(); }
  uint64_t complete_scans() const { return complete_scans_.load(); }
  uint64_t checksum_failures() const { return checksum_failures_.load(); }
  uint64_t checksum_drops() const { return checksum_drops_.load(); }
  uint64_t malformed_packets() const { return malformed_packets_.load(); }
  uint64_t incomplete_revolutions() const { return incomplete_revolutions_.load(); }
  uint64_t ring_starts() const { return ring_starts_.load(); }
  bool intensity_mode() const { return intensity_mode_; }
  int intensity_bits() const { return intensity_bits_.load(); }
  size_t sample_bytes() const
  {
    const int bits = intensity_bits_.load();
    if (!intensity_mode_ || bits <= 0) {
      return 2U;
    }
    return (bits >= 16) ? 4U : 3U;
  }
  uint64_t parser_layout_switches() const { return parser_layout_switches_.load(); }
  bool parser_layout_locked() const { return parser_layout_locked_.load(); }
  size_t pending_points() const;

private:
  void read_loop();
  std::optional<ScanData> read_packet();
  bool validate_checksum(
    uint8_t ct, uint8_t lsn, uint16_t fsa, uint16_t lsa, uint16_t cs,
    const std::vector<uint8_t> & sample_bytes);

  LiDARConnector & connector_;
  const bool intensity_mode_{true};
  // Runtime-selected wire layout. Start from the configured profile and, before
  // locking, cycle 16 -> 8 -> 0 bits when checksums prove the byte width is
  // wrong. This mirrors the YDLIDAR SDK auto-intensity strategy and prevents a
  // wrong 4B/3B/2B assumption from permanently blocking /scan_nav.
  std::atomic<int> intensity_bits_{16};
  const bool strict_checksum_{false};
  ScanCallback on_scan_received_;
  std::thread reader_thread_;
  std::atomic<bool> running_{false};

  std::vector<ScanPoint> current_scan_;
  mutable std::mutex scan_mutex_;

  std::atomic<uint64_t> valid_packets_{0};
  std::atomic<uint64_t> invalid_packets_{0};
  std::atomic<uint64_t> complete_scans_{0};
  std::atomic<uint64_t> checksum_failures_{0};
  std::atomic<uint64_t> checksum_drops_{0};
  std::atomic<uint64_t> malformed_packets_{0};
  std::atomic<uint64_t> incomplete_revolutions_{0};
  std::atomic<uint64_t> ring_starts_{0};
  std::atomic<int> consecutive_checksum_failures_{0};
  std::atomic<int> consecutive_valid_layout_packets_{0};
  std::atomic<bool> parser_layout_locked_{false};
  std::atomic<uint64_t> parser_layout_switches_{0};

  static constexpr uint8_t kStartByte1 = 0xAA;
  static constexpr uint8_t kStartByte2 = 0x55;
};

class LiDARNode : public rclcpp::Node
{
public:
  explicit LiDARNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LiDARNode() override;

private:
  void initialize_lidar();
  bool detect_port();
  void publish_scan(const std::vector<float> & scan, const rclcpp::Time & stamp,
    double measured_scan_period);
  void publish_safety_scan(const std::vector<float> & scan, const rclcpp::Time & stamp,
    double measured_scan_period);
  void monitor_status_callback();
  void on_scan_received(const ScanData & scan);
  void on_lidar_connected();
  void on_lidar_disconnected();
  void on_lidar_error(const std::string & error);
  void log_scan_reader_stats();
  void ensure_motor_and_reader();
  void soft_recover_lidar_stream(const std::string & reason);
  void restart_lidar_stack(const std::string & reason);
  void shutdown_lidar_hardware(const char * reason);

  // Parameters
  std::string port_;
  std::string exclude_ports_;
  int baudrate_{230400};
  std::string frame_id_{"lidar_link"};
  double scan_frequency_{10.0};
  double scan_publish_rate_{10.0};
  double hardware_scan_frequency_{0.0};
  double scan_timeout_{0.3};
  double stream_recovery_timeout_{2.5};
  double stream_recovery_cooldown_{1.0};
  double recovery_grace_sec_{4.0};
  double hard_restart_delay_sec_{6.0};
  int soft_recovery_attempts_before_hard_{2};
  double target_scan_period_{0.1};
  double range_min_{0.12};
  double range_max_{12.0};
  double angle_min_deg_{0.0};
  double angle_max_deg_{359.0};
  double angle_offset_deg_{0.0};
  bool auto_detect_port_{true};
  bool invert_angle_{true};
  bool intensity_mode_{true};
  int intensity_bits_{16};
  bool strict_checksum_{false};
  bool spatial_filter_enabled_{true};
  int spatial_filter_radius_bins_{7};
  int spatial_filter_min_support_{2};
  double spatial_filter_abs_tolerance_m_{0.18};
  double spatial_filter_rel_tolerance_{0.03};
  bool temporal_filter_enabled_{true};
  int temporal_filter_radius_bins_{2};
  double temporal_filter_range_diff_m_{0.25};
  // V7 endpoint-continuity gate: real walls/objects form compact Cartesian
  // neighbours even when range changes quickly on an oblique surface. Radial
  // spikes generally do not.
  double spatial_endpoint_max_gap_m_{0.32};
  int far_shadow_max_run_bins_{6};
  int min_valid_bins_per_scan_{40};

  // Components
  std::unique_ptr<LiDARConnector> connector_;
  std::unique_ptr<LiDARMotor> motor_;
  std::unique_ptr<ScanReader> reader_;

  // Publishers
  // Two physically synchronized scan products:
  //   scan_safety: minimally processed physical returns (no anti-starburst)
  //   scan:        anti-starburst filtered navigation returns
  // Robot-self masking is applied downstream to BOTH topics.
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr safety_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_motor_service_;

  // Timers
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // Scan accumulation (thread-safe)
  mutable std::mutex scan_buffer_mutex_;
  std::atomic<uint64_t> published_scans_{0};
  std::atomic<uint64_t> safety_published_scans_{0};
  std::atomic<uint64_t> dropped_sparse_scans_{0};
  std::atomic<uint64_t> sparse_published_scans_{0};
  std::atomic<uint64_t> filtered_isolated_bins_{0};
  std::atomic<uint64_t> scan_timeouts_{0};
  std::atomic<bool> scan_stale_active_{false};
  std::atomic<uint64_t> stream_recoveries_{0};
  std::atomic<uint64_t> soft_stream_recoveries_{0};
  std::atomic<uint64_t> hard_stream_recoveries_{0};
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_scan_wall_time_{};
  std::chrono::steady_clock::time_point last_published_scan_wall_time_{};
  std::chrono::steady_clock::time_point last_stream_recovery_{};
  std::chrono::steady_clock::time_point recovery_grace_until_{};
  std::chrono::steady_clock::time_point connector_unhealthy_since_{};
  std::atomic<bool> recovery_in_progress_{false};
  std::atomic<int> consecutive_stream_failures_{0};
  std::atomic<int> soft_recovery_failures_{0};
  std::vector<float> previous_raw_scan_;
  std::vector<float> previous_raw_scan_2_;
  std::atomic<bool> manual_stop_requested_{false};
  std::mutex shutdown_mutex_;
};

}  // namespace b1_lidar
