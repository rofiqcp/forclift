#include "b1_lidar/lidar_node.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <limits.h>
#include <sstream>
#include <cstdlib>
#include <sys/ioctl.h>
#include <unistd.h>

namespace b1_lidar
{
namespace
{
constexpr int kDefaultBaud = 230400;
constexpr int kProbeBauds[] = {230400, 128000, 115200};
constexpr uint8_t kCmdStartStandard[] = {0xA5, 0x60};
constexpr uint8_t kCmdStartG4[] = {0xA5, 0x82};
constexpr uint8_t kCmdStartTmini[] = {0xA5, 0x90};
constexpr uint8_t kCmdStop[] = {0xA5, 0x65};
constexpr uint8_t kCmdScanFreqAdd01 = 0x09;
constexpr uint8_t kCmdScanFreqSub01 = 0x0A;
constexpr uint8_t kCmdScanFreqAdd1 = 0x0B;
constexpr uint8_t kCmdScanFreqSub1 = 0x0C;
constexpr uint8_t kCmdGetScanFreq = 0x0D;
constexpr size_t kScanBins = 360;

// AGV HARDWARE: USB serial port is the stable alias created by resolve_usb_roles.py
// which resolves by physical hub port (by-path). Both IMU and LiDAR use CP2102
// (10c4:ea60) with duplicate serial "0001" on the same USB hub, so by-id is
// ambiguous. Resolution: /tmp/agv_devices/* aliases point by physical hub port.
// ACTUAL topology (Aug 2026):
//   Hub 1-2.1.1 = IMU  -> /dev/serial/by-path/...2.1.1:1.0-port0 -> ttyUSB0
//   Hub 1-2.1.4 = LiDAR -> /dev/serial/by-path/...2.1.4:1.0-port0 -> ttyUSB1
static constexpr const char * IMU_PHYSICAL_PATH =
  "/tmp/agv_devices/imu";
constexpr const char * LIDAR_PHYSICAL_PATH =
  "/tmp/agv_devices/lidar";

bool is_cp210x_io_fault(int error_code)
{
  return error_code == EIO || error_code == ENODEV || error_code == ENXIO;
}

void request_cp210x_recovery(int error_code, const std::string & port)
{
  if (!is_cp210x_io_fault(error_code)) {
    return;
  }
  static auto last_request = std::chrono::steady_clock::time_point{};
  const auto now = std::chrono::steady_clock::now();
  if (last_request.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - last_request).count() < 8.0)
  {
    return;
  }
  last_request = now;
  std::cerr << "[LIDAR-USB-RECOVERY] kernel tty I/O fault on " << port
            << " (" << std::strerror(error_code)
            << "); requesting serialized CP210x repair" << std::endl;
  // Run the role-specific helper synchronously and bounded.  Do NOT append '&':
  // an untracked background recovery can survive STOP/Ctrl+C and hold the
  // global recovery lock during the next mapping START.
  const int recovery_rc = std::system(
    "timeout 25s sudo -n /usr/local/sbin/agv-sensor-recover lidar "
    ">>/tmp/agv_sensor_recover.log 2>&1");
  if (recovery_rc == -1) {
    std::cerr << "[LIDAR-USB-RECOVERY] failed to start recovery helper: "
              << std::strerror(errno) << std::endl;
  } else if (recovery_rc != 0) {
    std::cerr << "[LIDAR-USB-RECOVERY] recovery helper exited rc="
              << recovery_rc << "; reconnect will retry" << std::endl;
  }
}

}  // namespace

// ===================== SerialPort =====================

SerialPort::SerialPort(std::string port, int baudrate)
: port_(std::move(port)), baudrate_(baudrate)
{
}

SerialPort::~SerialPort()
{
  close();
}

speed_t SerialPort::baud_to_constant(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B128000
    case 128000: return B128000;
#endif
    case 230400: return B230400;
#ifdef B256000
    case 256000: return B256000;
#endif
    case 460800: return B460800;
    default: return B230400;
  }
}

bool SerialPort::path_exists(const std::string & path)
{
  return access(path.c_str(), F_OK) == 0;
}

bool SerialPort::is_port_accessible(const std::string & path)
{
  // Just check file exists + read permission. Do NOT open the port here —
  // that resets the LiDAR state and breaks detection. The actual open()
  // with TIOCEXCL is done later and will fail properly if permissions deny it.
  return access(path.c_str(), F_OK) == 0;
}

bool SerialPort::open()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    return true;
  }

  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    const int saved_errno = errno;
    std::cout << "[LIDAR-TRANSPORT] open pending on " << port_ << ": "
              << std::strerror(saved_errno) << std::endl;
    io_error_ = true;
    request_cp210x_recovery(saved_errno, port_);
    return false;
  }

  // TIOCEXCL: exclusive serial ownership. A second open() by any process
  // (IMU node, modem manager, etc.) fails with EBUSY. Kept until close().
  if (ioctl(fd_, TIOCEXCL) != 0) {
    const int saved_errno = errno;
    std::cout << "[LIDAR-TRANSPORT] exclusive-open pending on " << port_ << ": "
              << std::strerror(saved_errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    io_error_ = true;
    request_cp210x_recovery(saved_errno, port_);
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    const int saved_errno = errno;
    std::cerr << "[LIDAR-TRANSPORT] tcgetattr failed on " << port_ << ": "
              << std::strerror(saved_errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    io_error_ = true;
    request_cp210x_recovery(saved_errno, port_);
    return false;
  }

  cfmakeraw(&tty);
  const speed_t speed = baud_to_constant(baudrate_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  // Do not let last-close HUPCL toggle RTS/DTR.  LiDAR motor power is
  // controlled explicitly by set_motor_power(), so close must be electrically quiet.
  tty.c_cflag &= ~HUPCL;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;  // 100 ms per blocking read chunk

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const int saved_errno = errno;
    std::cerr << "[LIDAR-TRANSPORT] tcsetattr failed on " << port_ << ": "
              << std::strerror(saved_errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    io_error_ = true;
    request_cp210x_recovery(saved_errno, port_);
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  int flags = fcntl(fd_, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  }

  io_error_ = false;
  return true;
}

void SerialPort::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0 && !io_error_.load();
}

std::vector<uint8_t> SerialPort::read(size_t size, int timeout_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> out;
  if (fd_ < 0 || size == 0) {
    return out;
  }

  out.reserve(size);
  uint8_t buf[512];
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max(1, timeout_ms));

  while (out.size() < size) {
    const size_t want = std::min(sizeof(buf), size - out.size());
    const ssize_t n = ::read(fd_, buf, want);
    if (n > 0) {
      out.insert(out.end(), buf, buf + n);
      continue;
    }
    if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    // Hard I/O error (USB unplug, etc.)
    io_error_ = true;
    break;
  }
  return out;
}

ssize_t SerialPort::write(const uint8_t * data, size_t size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0 || data == nullptr || size == 0) {
    return -1;
  }
  const ssize_t n = ::write(fd_, data, size);
  if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    io_error_ = true;
  }
  return n;
}

int SerialPort::available() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    return 0;
  }
  int count = 0;
  return ioctl(fd_, FIONREAD, &count) == 0 ? count : 0;
}

void SerialPort::flush()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    tcflush(fd_, TCIOFLUSH);
  }
}

bool SerialPort::set_motor_power(bool enabled)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    return false;
  }
  // Verified on the installed CP2102 + YDLiDAR T-mini Plus: the
  // continuous scan stream appears only when RTS/DTR are asserted.  The
  // previous active-low mapping left map-mode connected but with 0 scan data.
  int lines = TIOCM_RTS | TIOCM_DTR;
  return ioctl(fd_, enabled ? TIOCMBIS : TIOCMBIC, &lines) == 0;
}

bool SerialPort::path_is_healthy() const
{
  return path_exists(port_) && is_port_accessible(port_) && !io_error_.load();
}

// ===================== LiDARConnector =====================

LiDARConnector::LiDARConnector(std::string port, int baudrate)
: port_(std::move(port)), baudrate_(baudrate)
{
}

LiDARConnector::~LiDARConnector()
{
  stop_auto_reconnect();
  disconnect();
}

bool LiDARConnector::connect()
{
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
  {
    std::lock_guard<std::mutex> owner_lock(serial_owner_mutex_);
    if (state_.load() == State::Connected && serial_ && serial_->is_open()) {
      return true;
    }
  }

  state_ = State::Connecting;

  // Detach the old descriptor under the ownership mutex before opening a new
  // one. This guarantees the reader thread can never dereference serial_ while
  // the reconnect worker destroys it.
  std::unique_ptr<SerialPort> old_serial;
  {
    std::lock_guard<std::mutex> owner_lock(serial_owner_mutex_);
    old_serial = std::move(serial_);
  }
  if (old_serial) {
    old_serial->close();
  }

  auto candidate = std::make_unique<SerialPort>(port_, baudrate_);
  if (!candidate->open()) {
    state_ = State::Error;
    lifecycle_lock.unlock();
    if (on_error_) {
      on_error_("Failed to open serial port: " + port_);
    }
    return false;
  }

  // Power motor lines early so continuous-mode devices can stream.
  candidate->set_motor_power(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  {
    std::lock_guard<std::mutex> owner_lock(serial_owner_mutex_);
    serial_ = std::move(candidate);
  }

  state_ = State::Connected;
  reconnect_attempts_ = 0;
  lifecycle_lock.unlock();
  if (on_connected_) {
    on_connected_();
  }
  return true;
}

void LiDARConnector::disconnect()
{
  State prev = State::Disconnected;
  {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    std::unique_ptr<SerialPort> owned;
    {
      std::lock_guard<std::mutex> owner_lock(serial_owner_mutex_);
      owned = std::move(serial_);
    }
    if (owned) {
      owned->set_motor_power(false);
      owned->close();
    }
    prev = state_.exchange(State::Disconnected);
  }
  if (prev != State::Disconnected && on_disconnected_) {
    on_disconnected_();
  }
}

void LiDARConnector::start_auto_reconnect()
{
  if (reconnect_thread_.joinable()) {
    return;
  }
  running_ = true;
  reconnect_thread_ = std::thread(&LiDARConnector::reconnect_loop, this);
}

void LiDARConnector::stop_auto_reconnect()
{
  running_ = false;
  if (reconnect_thread_.joinable()) {
    reconnect_thread_.join();
  }
}

void LiDARConnector::request_reconnect(const std::string & reason)
{
  {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    state_ = State::Reconnecting;
    std::unique_ptr<SerialPort> owned;
    {
      std::lock_guard<std::mutex> owner_lock(serial_owner_mutex_);
      owned = std::move(serial_);
    }
    if (owned) {
      owned->set_motor_power(false);
      owned->close();
    }
  }
  // Report after releasing lifecycle ownership so callbacks never execute while
  // the connector's transition mutex is held.
  if (on_error_) {
    on_error_(reason);
  }
}

void LiDARConnector::reconnect_loop()
{
  while (running_) {
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(kReconnectIntervalSec * 1000)));

    if (!running_) {
      break;
    }

    const State st = state_.load();
    const bool open_ok = is_open();

    if (st == State::Connected && open_ok) {
      continue;
    }

    reconnect_attempts_++;
    state_ = State::Reconnecting;

    // CRITICAL: re-resolve the port on every reconnect attempt.
    // The ttyUSB number may have changed after USB re-enumeration.
    char resolved[PATH_MAX];
    if (SerialPort::path_exists(port_)) {
      if (realpath(port_.c_str(), resolved)) {
        fprintf(stderr, "[LiDAR-RECONNECT] port %s -> %s\n", port_.c_str(), resolved);
      } else {
        fprintf(stderr, "[LiDAR-RECONNECT] port %s [realpath failed: %s]\n",
          port_.c_str(), std::strerror(errno));
      }
    } else {
      fprintf(stderr, "[LiDAR-RECONNECT] port %s still absent — waiting\n", port_.c_str());
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }

    connect();
  }
}

std::vector<uint8_t> LiDARConnector::read(size_t size, int timeout_ms)
{
  bool open_after = false;
  std::vector<uint8_t> data;
  {
    // Hold ownership while SerialPort::read executes. Reconnect/disconnect will
    // wait for the short read deadline instead of deleting the object under it.
    std::lock_guard<std::mutex> owner_lock(serial_owner_mutex_);
    if (!serial_) {
      return {};
    }
    data = serial_->read(size, timeout_ms);
    open_after = serial_->is_open();
  }
  if (!open_after) {
    request_reconnect("Serial I/O error during read");
  }
  return data;
}

ssize_t LiDARConnector::write(const uint8_t * data, size_t size)
{
  ssize_t n = -1;
  bool open_after = false;
  {
    std::lock_guard<std::mutex> owner_lock(serial_owner_mutex_);
    if (!serial_) {
      return -1;
    }
    n = serial_->write(data, size);
    open_after = serial_->is_open();
  }
  if (!open_after) {
    request_reconnect("Serial I/O error during write");
  }
  return n;
}

void LiDARConnector::flush()
{
  std::lock_guard<std::mutex> owner_lock(serial_owner_mutex_);
  if (serial_) {
    serial_->flush();
  }
}

bool LiDARConnector::set_motor_power(bool enabled)
{
  std::lock_guard<std::mutex> owner_lock(serial_owner_mutex_);
  return serial_ && serial_->set_motor_power(enabled);
}

bool LiDARConnector::is_open() const
{
  std::lock_guard<std::mutex> owner_lock(serial_owner_mutex_);
  return serial_ && serial_->is_open();
}

// ===================== LiDARMotor =====================

LiDARMotor::LiDARMotor(LiDARConnector & connector)
: connector_(connector)
{
}

bool LiDARMotor::probe_for_sync(int wait_ms)
{
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);

  // Preserve the previous byte across serial read chunks. AA55 can straddle
  // the chunk boundary; missing it made a healthy stream look offline and
  // caused unnecessary fallback start commands.
  bool have_prev = false;
  uint8_t prev = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    auto chunk = connector_.read(200, 100);
    for (const uint8_t byte : chunk) {
      if (have_prev && prev == 0xAA && byte == 0x55) {
        return true;
      }
      prev = byte;
      have_prev = true;
    }
  }
  return false;
}

bool LiDARMotor::start_scan(int wait_ms)
{
  if (!connector_.is_open()) {
    std::lock_guard<std::mutex> lock(mutex_);
    motor_state_ = "ERROR";
    return false;
  }

  connector_.set_motor_power(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Fast path: device may already be streaming (continuous mode / previous start).
  if (probe_for_sync(400)) {
    std::lock_guard<std::mutex> lock(mutex_);
    motor_state_ = "RUNNING";
    std::cout << "[INFO] MOTOR RUNNING - already streaming AA55" << std::endl;
    return true;
  }

  // On this unit A5 60 is the continuous-scan command and A5 82 is
  // kept as the only fallback.  A5 90 returns a configuration/info frame and
  // can interrupt a stream that has already started.
  static const std::vector<std::vector<uint8_t>> start_cmds = {
    {kCmdStartStandard[0], kCmdStartStandard[1]},
    {kCmdStartG4[0], kCmdStartG4[1]},
  };

  for (const auto & cmd : start_cmds) {
    connector_.flush();
    connector_.set_motor_power(true);
    if (connector_.write(cmd.data(), cmd.size()) < 0) {
      continue;
    }
    if (probe_for_sync(wait_ms)) {
      std::lock_guard<std::mutex> lock(mutex_);
      motor_state_ = "RUNNING";
      std::cout << "[INFO] MOTOR RUNNING - start cmd "
                << std::hex << static_cast<int>(cmd[0]) << " "
                << static_cast<int>(cmd[1]) << std::dec << std::endl;
      return true;
    }
    std::cerr << "Command " << std::hex << static_cast<int>(cmd[0]) << " "
              << static_cast<int>(cmd[1]) << std::dec
              << " sent but no AA55 sync detected" << std::endl;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  motor_state_ = "ERROR";
  return false;
}

bool LiDARMotor::stop_scan()
{
  if (!connector_.is_open()) {
    return false;
  }
  connector_.write(kCmdStop, sizeof(kCmdStop));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  connector_.set_motor_power(false);
  std::lock_guard<std::mutex> lock(mutex_);
  motor_state_ = "STOPPED";
  return true;
}

bool LiDARMotor::command_scan_frequency(uint8_t command, double & hz, int timeout_ms)
{
  if (!connector_.is_open()) {
    return false;
  }

  connector_.flush();
  const uint8_t cmd[2] = {0xA5, command};
  if (connector_.write(cmd, sizeof(cmd)) != static_cast<ssize_t>(sizeof(cmd))) {
    return false;
  }

  std::vector<uint8_t> buffer;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(std::max(100, timeout_ms));
  while (std::chrono::steady_clock::now() < deadline) {
    auto chunk = connector_.read(32, 60);
    buffer.insert(buffer.end(), chunk.begin(), chunk.end());
    for (size_t i = 0; i + 7U <= buffer.size(); ++i) {
      if (buffer[i] != 0xA5 || buffer[i + 1U] != 0x5A) {
        continue;
      }
      const uint32_t descriptor =
        static_cast<uint32_t>(buffer[i + 2U]) |
        (static_cast<uint32_t>(buffer[i + 3U]) << 8U) |
        (static_cast<uint32_t>(buffer[i + 4U]) << 16U) |
        (static_cast<uint32_t>(buffer[i + 5U]) << 24U);
      const uint32_t payload_len = descriptor & 0x3FFFFFFFU;
      const uint8_t response_type = buffer[i + 6U];
      if (payload_len != 4U || response_type != 0x04U) {
        continue;
      }
      if (i + 7U + payload_len > buffer.size()) {
        break;
      }
      const size_t p = i + 7U;
      const uint32_t raw =
        static_cast<uint32_t>(buffer[p]) |
        (static_cast<uint32_t>(buffer[p + 1U]) << 8U) |
        (static_cast<uint32_t>(buffer[p + 2U]) << 16U) |
        (static_cast<uint32_t>(buffer[p + 3U]) << 24U);
      const double parsed_hz = static_cast<double>(raw) / 100.0;
      if (std::isfinite(parsed_hz) && parsed_hz >= 3.0 && parsed_hz <= 20.0) {
        hz = parsed_hz;
        return true;
      }
    }
    if (buffer.size() > 128U) {
      buffer.erase(buffer.begin(), buffer.end() - 64);
    }
  }
  return false;
}

bool LiDARMotor::get_scan_frequency(double & hz, int timeout_ms)
{
  return command_scan_frequency(kCmdGetScanFreq, hz, timeout_ms);
}

bool LiDARMotor::set_scan_frequency(double target_hz, double & actual_hz)
{
  // T-mini family supports software scan-frequency regulation. Keep the target
  // inside the documented SDK range and quantize to the protocol's 0.1 Hz step.
  target_hz = std::clamp(target_hz, 5.0, 12.0);
  target_hz = std::round(target_hz * 10.0) / 10.0;

  // Frequency commands are defined for the non-scan state. Stop streaming but
  // keep serial/motor control power asserted so the command channel stays alive.
  connector_.write(kCmdStop, sizeof(kCmdStop));
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  connector_.flush();

  double current_hz = 0.0;
  if (!get_scan_frequency(current_hz, 800)) {
    return false;
  }

  int tenths = static_cast<int>(std::llround((target_hz - current_hz) * 10.0));
  int guard = 0;
  while (std::abs(tenths) >= 10 && guard++ < 12) {
    const uint8_t command = tenths > 0 ? kCmdScanFreqAdd1 : kCmdScanFreqSub1;
    if (!command_scan_frequency(command, current_hz, 800)) {
      return false;
    }
    tenths = static_cast<int>(std::llround((target_hz - current_hz) * 10.0));
  }
  guard = 0;
  while (tenths != 0 && guard++ < 20) {
    const uint8_t command = tenths > 0 ? kCmdScanFreqAdd01 : kCmdScanFreqSub01;
    if (!command_scan_frequency(command, current_hz, 800)) {
      return false;
    }
    tenths = static_cast<int>(std::llround((target_hz - current_hz) * 10.0));
  }

  double verified_hz = 0.0;
  if (!get_scan_frequency(verified_hz, 800)) {
    return false;
  }
  actual_hz = verified_hz;
  return std::abs(verified_hz - target_hz) <= 0.15;
}

void LiDARMotor::mark_stream_running()
{
  // The strongest proof that the motor/transport is healthy is a completed
  // physical revolution. A previous start command can time out while the
  // device is still spinning; do not leave the diagnostic state stuck at
  // ERROR after real scan data has resumed.
  std::lock_guard<std::mutex> lock(mutex_);
  motor_state_ = "RUNNING";
}

std::string LiDARMotor::get_motor_state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return motor_state_;
}

// ===================== ScanReader =====================

ScanReader::ScanReader(
  LiDARConnector & connector, bool intensity_mode, int intensity_bits, bool strict_checksum)
: connector_(connector),
  intensity_mode_(intensity_mode),
  intensity_bits_(intensity_mode ? intensity_bits : 0),
  strict_checksum_(strict_checksum)
{
}

ScanReader::~ScanReader()
{
  stop();
}

size_t ScanReader::pending_points() const
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  return current_scan_.size();
}

void ScanReader::start()
{
  if (running_.load() && reader_thread_.joinable()) {
    return;
  }
  reset_accumulator();
  consecutive_checksum_failures_ = 0;
  consecutive_valid_layout_packets_ = 0;
  parser_layout_locked_ = false;
  connector_.flush();
  running_ = true;
  reader_thread_ = std::thread(&ScanReader::read_loop, this);
}

void ScanReader::stop()
{
  running_ = false;
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
}

void ScanReader::reset_accumulator()
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  current_scan_.clear();
}

bool ScanReader::validate_checksum(
  uint8_t ct, uint8_t lsn, uint16_t fsa, uint16_t lsa, uint16_t cs,
  const std::vector<uint8_t> & sample_bytes)
{
  // YDLiDAR protocol checksum.  The intensity layout is NOT a generic XOR of
  // consecutive 16-bit payload words: each 3-byte sample contributes the
  // intensity byte as one XOR term and the two distance bytes as another.
  uint16_t chk = 0x55AA;
  chk ^= fsa;
  const int active_bits = intensity_bits_.load();

  if (intensity_mode_ && active_bits >= 16) {
    // T-mini Plus SH (25 m) uses 16-bit intensity + 16-bit distance:
    //   Si = I_low, I_high, D_low, D_high  -> 4 bytes/sample.
    // The official YDLidar SDK XORs both 16-bit words independently.
    const size_t expected = static_cast<size_t>(lsn) * 4U;
    if (sample_bytes.size() != expected) {
      return false;
    }
    for (size_t i = 0; i < expected; i += 4U) {
      const uint16_t intensity_word = static_cast<uint16_t>(sample_bytes[i]) |
        (static_cast<uint16_t>(sample_bytes[i + 1U]) << 8);
      const uint16_t distance_word = static_cast<uint16_t>(sample_bytes[i + 2U]) |
        (static_cast<uint16_t>(sample_bytes[i + 3U]) << 8);
      chk ^= intensity_word;
      chk ^= distance_word;
    }
  } else if (intensity_mode_ && active_bits > 0) {
    // 8/10-bit intensity YDLidar variants use one intensity byte followed by
    // the 16-bit Q2 distance word (3 bytes/sample).
    const size_t expected = static_cast<size_t>(lsn) * 3U;
    if (sample_bytes.size() != expected) {
      return false;
    }
    for (size_t i = 0; i < expected; i += 3U) {
      chk ^= static_cast<uint16_t>(sample_bytes[i]);
      chk ^= static_cast<uint16_t>(
        (static_cast<uint16_t>(sample_bytes[i + 2U]) << 8) |
        static_cast<uint16_t>(sample_bytes[i + 1U]));
    }
  } else {
    const size_t expected = static_cast<size_t>(lsn) * 2U;
    if (sample_bytes.size() != expected) {
      return false;
    }
    for (size_t i = 0; i < expected; i += 2U) {
      chk ^= static_cast<uint16_t>(
        (static_cast<uint16_t>(sample_bytes[i + 1U]) << 8) |
        static_cast<uint16_t>(sample_bytes[i]));
    }
  }

  // The current YDLIDAR SDK protocol specifies the complete [CT | LSN] word.
  // Some older triangle-LiDAR examples mask CT to its packet-type bit before
  // calculating CS.  Accept both documented wire variants, but never accept a
  // packet that matches neither.  The previous implementation accepted only
  // the masked variant; field logs consequently reported checksum_fail for
  // virtually every T-mini Plus packet and had to run permanently unverified.
  const uint16_t checksum_without_ct = static_cast<uint16_t>(chk ^ lsa);
  const uint16_t full_ct_checksum = static_cast<uint16_t>(
    checksum_without_ct ^
    ((static_cast<uint16_t>(lsn) << 8) | static_cast<uint16_t>(ct)));
  const uint8_t packet_type = static_cast<uint8_t>(ct & 0x01U);
  const uint16_t type_only_checksum = static_cast<uint16_t>(
    checksum_without_ct ^
    ((static_cast<uint16_t>(lsn) << 8) | static_cast<uint16_t>(packet_type)));
  return full_ct_checksum == cs || type_only_checksum == cs;
}

void ScanReader::read_loop()
{
  // V6: delimit scans by the LiDAR's own CT ring-start marker.  The latest
  // runtime from this exact unit shows ring_start tracking completed scans
  // almost 1:1 (e.g. complete=627, ring=629), so a 350-degree software cut is
  // no longer allowed to split/mix neighbouring physical revolutions while the
  // chassis is moving.  Points before the first ring marker are deliberately
  // discarded as an unknown partial revolution.
  double accumulated_span_deg = 0.0;
  double last_angle_deg = -1.0;
  bool have_ring_start = false;
  rclcpp::Time revolution_start_stamp(0, 0, RCL_ROS_TIME);

  while (running_.load()) {
    if (!connector_.is_open()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    auto scan = read_packet();
    if (!scan) {
      invalid_packets_++;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    valid_packets_++;
    if (scan->ring_start) {
      ring_starts_++;
    }

    ScanData complete;
    bool emit_complete = false;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);

      if (scan->ring_start) {
        // A new ring marker closes the PREVIOUS physical revolution.  Stamp it
        // with the acquisition time of its first packet (LaserScan semantics),
        // not with the following revolution's packet time.  This also prevents
        // RViz/SLAM from asking TF for a scan timestamp newer than available TF.
        if (have_ring_start) {
          const bool enough_points = current_scan_.size() >= 80U;
          const bool enough_coverage = accumulated_span_deg >= 300.0;
          if (enough_points && enough_coverage) {
            complete.stamp = revolution_start_stamp;
            complete.points.swap(current_scan_);
            complete.ring_start = true;
            complete.checksum_ok = scan->checksum_ok;
            complete_scans_++;
            emit_complete = true;
          } else if (!current_scan_.empty()) {
            incomplete_revolutions_++;
            current_scan_.clear();
          }
        } else {
          // Startup bytes may begin midway through a revolution.  Never feed
          // that partial geometry into mapping.
          current_scan_.clear();
        }

        have_ring_start = true;
        revolution_start_stamp = scan->stamp;
        accumulated_span_deg = 0.0;
        last_angle_deg = -1.0;
      }

      // Until a real ring marker has been seen, ignore packet geometry.  The
      // stream watchdog will recover automatically if ring markers disappear.
      if (have_ring_start) {
        for (const auto & pt : scan->points) {
          if (last_angle_deg >= 0.0) {
            double delta = pt.angle_deg - last_angle_deg;
            if (delta < -180.0) {
              delta += 360.0;
            } else if (delta > 180.0) {
              delta -= 360.0;
            }

            // Only forward, physically plausible beam progression contributes
            // to coverage diagnostics.  Corrupt angle packets cannot fabricate
            // a valid full revolution.
            if (delta > 0.0 && delta < 45.0) {
              accumulated_span_deg += delta;
            }
          }
          last_angle_deg = pt.angle_deg;
          current_scan_.push_back(pt);
        }
      }

      // Hard safety only.  Do NOT publish an arbitrary 350-degree software
      // sweep as a fallback: mixing two revolutions is worse than dropping one
      // scan and letting the existing stream-recovery watchdog restart cleanly.
      if (current_scan_.size() > 5000U || accumulated_span_deg > 500.0) {
        current_scan_.clear();
        accumulated_span_deg = 0.0;
        last_angle_deg = -1.0;
        have_ring_start = false;
        incomplete_revolutions_++;
      }
    }

    if (emit_complete && on_scan_received_) {
      on_scan_received_(complete);
    }
  }
}

std::optional<ScanData> ScanReader::read_packet()
{
  // Search for AA 55 sync with short per-byte timeouts (no long blocking).
  bool sync_found = false;
  for (int i = 0; i < 200 && running_.load(); ++i) {
    auto b1 = connector_.read(1, 20);
    if (b1.empty()) {
      if (i > 50) {
        return std::nullopt;
      }
      continue;
    }
    if (b1[0] != kStartByte1) {
      continue;
    }
    auto b2 = connector_.read(1, 20);
    if (b2.empty()) {
      return std::nullopt;
    }
    if (b2[0] == kStartByte2) {
      sync_found = true;
      break;
    }
  }
  if (!sync_found) {
    return std::nullopt;
  }

  auto header = connector_.read(2, 50);
  if (header.size() < 2) {
    return std::nullopt;
  }
  const uint8_t ct = header[0];
  const uint8_t lsn = header[1];
  if (lsn < 1 || lsn > 80) {
    return std::nullopt;
  }

  auto angle_cs = connector_.read(6, 50);
  if (angle_cs.size() < 6) {
    return std::nullopt;
  }

  const uint16_t fsa_raw =
    static_cast<uint16_t>(angle_cs[0]) | (static_cast<uint16_t>(angle_cs[1]) << 8);
  const uint16_t lsa_raw =
    static_cast<uint16_t>(angle_cs[2]) | (static_cast<uint16_t>(angle_cs[3]) << 8);
  const uint16_t cs_raw =
    static_cast<uint16_t>(angle_cs[4]) | (static_cast<uint16_t>(angle_cs[5]) << 8);

  // FSA/LSA use bit0 as the protocol check bit. A false AA55 found inside
  // sample payload often fails this immediately; reject it before it can turn
  // into a radial ray in the map.
  if ((fsa_raw & 0x1U) == 0U || (lsa_raw & 0x1U) == 0U) {
    malformed_packets_++;
    return std::nullopt;
  }

  double fsa = (fsa_raw >> 1) / 64.0;
  double lsa = (lsa_raw >> 1) / 64.0;
  if (lsa < fsa) {
    lsa += 360.0;
  }
  const double packet_span = lsa - fsa;
  if (lsn > 1U && (!std::isfinite(packet_span) || packet_span <= 0.0 || packet_span > 90.0)) {
    malformed_packets_++;
    return std::nullopt;
  }

  // Start from the configured T-mini Plus SH 16-bit layout, but do not assume
  // the USB unit always exposes that exact wire format. Some YDLIDAR firmware
  // variants report 8-bit intensity (3 B/sample) or no intensity (2 B/sample).
  // The checksum below is the authority; before the parser locks, repeated
  // checksum failures rotate 4B -> 3B -> 2B until real packets validate.
  const int active_bits = intensity_bits_.load();
  const size_t bytes_per_sample =
    (!intensity_mode_ || active_bits <= 0) ? 2U : (active_bits >= 16 ? 4U : 3U);
  const size_t data_size = static_cast<size_t>(lsn) * bytes_per_sample;
  auto data = connector_.read(data_size, 100);
  if (data.size() < data_size) {
    malformed_packets_++;
    return std::nullopt;
  }

  const bool checksum_ok = validate_checksum(ct, lsn, fsa_raw, lsa_raw, cs_raw, data);
  if (!checksum_ok) {
    checksum_failures_++;
    consecutive_valid_layout_packets_ = 0;
    const int failures = consecutive_checksum_failures_.fetch_add(1) + 1;

    // Only auto-detect before a layout has been proven. Reading with a wrong
    // width can consume bytes from the next packet, so after switching we
    // flush only the software revolution accumulator and let the AA55 search
    // resynchronise naturally on the wire. Three failures avoids switching on
    // one damaged packet while still converging quickly at startup.
    if (!parser_layout_locked_.load() && failures >= 3) {
      const int old_bits = intensity_bits_.load();
      const int new_bits = old_bits >= 16 ? 8 : (old_bits > 0 ? 0 : 16);
      intensity_bits_ = new_bits;
      consecutive_checksum_failures_ = 0;
      parser_layout_switches_++;
      reset_accumulator();
      const size_t new_bytes = new_bits <= 0 ? 2U : (new_bits >= 16 ? 4U : 3U);
      std::cerr << "[LIDAR-AUTO-PARSER] checksum mismatch on " << bytes_per_sample
                << "B layout; switching to " << new_bytes
                << "B/sample (intensity_bits=" << new_bits << ")" << std::endl;
    }

    if (strict_checksum_) {
      checksum_drops_++;
      return std::nullopt;
    }
  } else {
    consecutive_checksum_failures_ = 0;
    const int valid_streak = consecutive_valid_layout_packets_.fetch_add(1) + 1;
    if (!parser_layout_locked_.load() && valid_streak >= 3) {
      parser_layout_locked_ = true;
      std::cerr << "[LIDAR-AUTO-PARSER] locked verified layout: "
                << bytes_per_sample << "B/sample intensity_bits=" << active_bits
                << std::endl;
    }
  }

  ScanData scan;
  scan.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  scan.ring_start = (ct & 0x01U) != 0U;
  scan.checksum_ok = checksum_ok;
  scan.points.reserve(lsn);

  for (uint8_t i = 0; i < lsn; ++i) {
    uint16_t distance_q2 = 0U;
    uint16_t intensity = 0U;
    if (intensity_mode_ && active_bits >= 16) {
      const size_t base = static_cast<size_t>(i) * 4U;
      intensity = static_cast<uint16_t>(data[base]) |
        (static_cast<uint16_t>(data[base + 1U]) << 8);
      distance_q2 = static_cast<uint16_t>(data[base + 2U]) |
        (static_cast<uint16_t>(data[base + 3U]) << 8);
    } else if (intensity_mode_ && active_bits > 0) {
      const size_t base = static_cast<size_t>(i) * 3U;
      const uint8_t ibyte = data[base];
      distance_q2 = static_cast<uint16_t>(data[base + 1U]) |
        (static_cast<uint16_t>(data[base + 2U]) << 8);
      if (active_bits == 10) {
        intensity = static_cast<uint16_t>(ibyte) |
          static_cast<uint16_t>((distance_q2 & 0x0003U) << 8);
      } else {
        intensity = static_cast<uint16_t>(ibyte);
      }
      // The low two distance bits carry intensity/flag information on the
      // 8/10-bit layouts, not distance.
      distance_q2 = static_cast<uint16_t>(distance_q2 & 0xFFFCU);
    } else {
      const size_t base = static_cast<size_t>(i) * 2U;
      distance_q2 = static_cast<uint16_t>(data[base]) |
        (static_cast<uint16_t>(data[base + 1U]) << 8);
    }

    // The vendor T-mini Plus SH ROS profile is TYPE_TOF with 16-bit intensity.
    // Its distance word is millimetres, so convert raw / 1000 to metres.  The
    // legacy 2/3-byte triangle layouts remain Q2 millimetres (raw / 4000).
    double dist_m = 0.0;
    if (distance_q2 > 0U) {
      const double distance_scale =
        (intensity_mode_ && active_bits >= 16) ? 1000.0 : 4000.0;
      dist_m = static_cast<double>(distance_q2) / distance_scale;
    }

    double angle = (lsn > 1) ?
      fsa + (lsa - fsa) * static_cast<double>(i) / (lsn - 1) : fsa;

    // Current YDLidar SDK intentionally skips the legacy second-level angle
    // correction for the entire T-mini family, including T-mini Plus SH.

    angle = std::fmod(angle, 360.0);
    if (angle < 0.0) {
      angle += 360.0;
    }

    ScanPoint pt;
    pt.angle_deg = angle;
    pt.distance_m = dist_m;
    pt.intensity = intensity;
    pt.valid = (dist_m >= 0.03 && dist_m <= 25.0);
    if (!pt.valid) {
      pt.distance_m = 0.0;
    }
    scan.points.push_back(pt);
  }

  return scan;
}

// ===================== LiDARNode =====================

LiDARNode::LiDARNode(const rclcpp::NodeOptions & options)
: Node("lidar_node", options)
{
  declare_parameter("port", std::string(LIDAR_PHYSICAL_PATH));
  declare_parameter("baudrate", kDefaultBaud);
  declare_parameter("frame_id", std::string("lidar_link"));
  declare_parameter("scan_frequency", 10.0);
  declare_parameter("scan_publish_rate", 10.0);
  declare_parameter("scan_timeout", 0.3);
  declare_parameter("stream_recovery_timeout", 2.5);
  declare_parameter("stream_recovery_cooldown", 1.0);
  declare_parameter("recovery_grace_sec", 4.0);
  declare_parameter("hard_restart_delay_sec", 6.0);
  declare_parameter("soft_recovery_attempts_before_hard", 2);
  declare_parameter("target_scan_period", 0.1);
  declare_parameter("range_min", 0.12);
  declare_parameter("range_max", 6.0);
  declare_parameter("angle_min", 0.0);
  declare_parameter("angle_max", 359.0);
  declare_parameter("auto_detect_port", false);
  declare_parameter("exclude_ports", std::string(""));
  declare_parameter("angle_offset", 0.0);
  declare_parameter("invert_angle", true);
  declare_parameter("intensity_mode", true);
  declare_parameter("intensity_bits", 16);
  declare_parameter("strict_checksum", true);
  declare_parameter("spatial_filter_enabled", true);
  declare_parameter("spatial_filter_radius_bins", 7);
  declare_parameter("spatial_filter_min_support", 2);
  declare_parameter("spatial_filter_abs_tolerance_m", 0.18);
  declare_parameter("spatial_filter_rel_tolerance", 0.03);
  declare_parameter("temporal_filter_enabled", true);
  declare_parameter("temporal_filter_radius_bins", 2);
  declare_parameter("temporal_filter_range_diff_m", 0.25);
  declare_parameter("spatial_endpoint_max_gap_m", 0.32);
  declare_parameter("far_shadow_max_run_bins", 6);
  declare_parameter("min_valid_bins_per_scan", 40);

  port_ = get_parameter("port").as_string();
  baudrate_ = get_parameter("baudrate").as_int();
  frame_id_ = get_parameter("frame_id").as_string();
  scan_frequency_ = get_parameter("scan_frequency").as_double();
  scan_publish_rate_ = get_parameter("scan_publish_rate").as_double();
  scan_timeout_ = get_parameter("scan_timeout").as_double();
  stream_recovery_timeout_ = std::max(0.8, get_parameter("stream_recovery_timeout").as_double());
  stream_recovery_cooldown_ = std::max(0.2, get_parameter("stream_recovery_cooldown").as_double());
  recovery_grace_sec_ = std::max(1.0, get_parameter("recovery_grace_sec").as_double());
  hard_restart_delay_sec_ = std::max(recovery_grace_sec_, get_parameter("hard_restart_delay_sec").as_double());
  soft_recovery_attempts_before_hard_ = std::max(1, static_cast<int>(get_parameter("soft_recovery_attempts_before_hard").as_int()));
  target_scan_period_ = get_parameter("target_scan_period").as_double();
  range_min_ = get_parameter("range_min").as_double();
  range_max_ = get_parameter("range_max").as_double();
  angle_min_deg_ = get_parameter("angle_min").as_double();
  angle_max_deg_ = get_parameter("angle_max").as_double();
  angle_offset_deg_ = get_parameter("angle_offset").as_double();
  auto_detect_port_ = get_parameter("auto_detect_port").as_bool();
  exclude_ports_ = get_parameter("exclude_ports").as_string();
  invert_angle_ = get_parameter("invert_angle").as_bool();
  intensity_mode_ = get_parameter("intensity_mode").as_bool();
  intensity_bits_ = static_cast<int>(get_parameter("intensity_bits").as_int());
  if (!intensity_mode_) {
    intensity_bits_ = 0;
  } else if (intensity_bits_ != 8 && intensity_bits_ != 10 && intensity_bits_ != 16) {
    // The 25 m T-mini Plus SH profile is 16-bit intensity. Keep startup
    // deterministic rather than silently choosing an unsupported byte width.
    intensity_bits_ = 16;
  }
  strict_checksum_ = get_parameter("strict_checksum").as_bool();
  spatial_filter_enabled_ = get_parameter("spatial_filter_enabled").as_bool();
  spatial_filter_radius_bins_ = std::max(1, static_cast<int>(get_parameter("spatial_filter_radius_bins").as_int()));
  spatial_filter_min_support_ = std::max(1, static_cast<int>(get_parameter("spatial_filter_min_support").as_int()));
  spatial_filter_abs_tolerance_m_ = std::max(0.01, get_parameter("spatial_filter_abs_tolerance_m").as_double());
  spatial_filter_rel_tolerance_ = std::max(0.0, get_parameter("spatial_filter_rel_tolerance").as_double());
  temporal_filter_enabled_ = get_parameter("temporal_filter_enabled").as_bool();
  temporal_filter_radius_bins_ = std::max(0, static_cast<int>(get_parameter("temporal_filter_radius_bins").as_int()));
  temporal_filter_range_diff_m_ = std::max(0.01, get_parameter("temporal_filter_range_diff_m").as_double());
  spatial_endpoint_max_gap_m_ = std::max(0.05, get_parameter("spatial_endpoint_max_gap_m").as_double());
  far_shadow_max_run_bins_ = std::max(1, static_cast<int>(get_parameter("far_shadow_max_run_bins").as_int()));
  min_valid_bins_per_scan_ = std::max(20, static_cast<int>(get_parameter("min_valid_bins_per_scan").as_int()));

  if (scan_frequency_ < 1.0) {
    scan_frequency_ = 10.0;
  }

  RCLCPP_INFO(get_logger(), "LiDAR Node initialized");
  {
    char resolved[PATH_MAX];
    const char * resolved_path = realpath(port_.c_str(), resolved) ? resolved : port_.c_str();
    RCLCPP_INFO(
      get_logger(),
      "Port: %s  [resolved: %s]  (auto-detect: %s, EXCLUSIVE by-path)",
      port_.c_str(), resolved_path,
      auto_detect_port_ ? "true" : "false");
  }
  RCLCPP_INFO(get_logger(), "Baudrate: %d", baudrate_);
  RCLCPP_INFO(get_logger(), "Frame ID: %s", frame_id_.c_str());
  if (!exclude_ports_.empty()) {
    RCLCPP_INFO(get_logger(), "Excluded serial ports from LiDAR probing: %s", exclude_ports_.c_str());
  }
  RCLCPP_INFO(get_logger(), "Requested hardware scan frequency: %.1f Hz", scan_frequency_);
  if (std::abs(scan_publish_rate_ - scan_frequency_) > 0.01) {
    RCLCPP_WARN(
      get_logger(),
      "scan_publish_rate=%.1f differs from scan_frequency=%.1f; driver publishes one fresh scan per physical revolution (no replay timer)",
      scan_publish_rate_, scan_frequency_);
  }
  const int parser_sample_bytes =
    (!intensity_mode_ || intensity_bits_ <= 0) ? 2 : (intensity_bits_ >= 16 ? 4 : 3);
  const char * parser_mode =
    (!intensity_mode_ || intensity_bits_ <= 0) ? "LEGACY_2BYTE" :
    (intensity_bits_ >= 16 ? "TMINI_PLUS_SH_INTENSITY_16BIT_4BYTE" :
     (intensity_bits_ == 10 ? "INTENSITY_10BIT_3BYTE" : "INTENSITY_8BIT_3BYTE"));
  RCLCPP_INFO(
    get_logger(),
    "[LIDAR-PARSER] mode=%s intensity_bits=%d bytes/sample=%d strict_checksum=%s spatial_filter=%s min_bins=%d",
    parser_mode, intensity_bits_, parser_sample_bytes, strict_checksum_ ? "true" : "false",
    spatial_filter_enabled_ ? "true" : "false", min_valid_bins_per_scan_);
  RCLCPP_INFO(
    get_logger(),
    "[ANTI-STARBURST-V7] side_radius=%d min_side=%d endpoint_gap=%.2fm far_run<=%d bins temporal=%s two_frame=true +/- %d bins / %.2fm",
    spatial_filter_radius_bins_, spatial_filter_min_support_, spatial_endpoint_max_gap_m_,
    far_shadow_max_run_bins_, temporal_filter_enabled_ ? "on" : "off",
    temporal_filter_radius_bins_, temporal_filter_range_diff_m_);

  // SensorData QoS: BEST_EFFORT + VOLATILE + KEEP_LAST — compatible with slam_toolbox.
  const auto sensor_qos = rclcpp::SensorDataQoS();
  // PART-4 Stage-2: keep the safety path independent of the anti-starburst
  // navigation filter.  Both messages come from the same completed physical
  // revolution and carry the same acquisition timestamp.  The safety stream
  // contains all physically valid returns; the navigation stream is the
  // quality-filtered product used by SLAM/AMCL/costmaps.
  safety_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan_safety", sensor_qos);
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", sensor_qos);
  status_pub_ = create_publisher<std_msgs::msg::String>("lidar/status", 10);
  stop_motor_service_ = create_service<std_srvs::srv::Trigger>(
    "/lidar/stop_motor",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      manual_stop_requested_.store(true);
      shutdown_lidar_hardware("GUI/service stop request");
      response->success = true;
      response->message = "LiDAR stop command sent, motor power OFF, serial closed";
    });

  monitor_timer_ = create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&LiDARNode::monitor_status_callback, this));
  stats_timer_ = create_wall_timer(
    std::chrono::seconds(3),
    std::bind(&LiDARNode::log_scan_reader_stats, this));

  initialize_lidar();
}

LiDARNode::~LiDARNode()
{
  manual_stop_requested_.store(true);
  shutdown_lidar_hardware("node destructor");
}

void LiDARNode::shutdown_lidar_hardware(const char * reason)
{
  std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
  RCLCPP_INFO(get_logger(), "[LIDAR-SHUTDOWN] %s", reason ? reason : "requested");

  // Prevent watchdog/recovery callbacks from restarting the motor while STOP is in progress.
  if (monitor_timer_) {
    monitor_timer_->cancel();
  }
  if (stats_timer_) {
    stats_timer_->cancel();
  }

  if (reader_) {
    reader_->stop();
  }
  if (connector_) {
    connector_->stop_auto_reconnect();
  }

  // Send protocol STOP while the serial port is still owned, then force the
  // adapter's RTS/DTR motor-control lines to the OFF state.
  if (motor_ && connector_ && connector_->is_open()) {
    const bool stopped = motor_->stop_scan();
    RCLCPP_INFO(get_logger(), "[LIDAR-SHUTDOWN] A5 65 + motor power OFF: %s", stopped ? "OK" : "FAILED");
  } else if (connector_ && connector_->is_open()) {
    connector_->set_motor_power(false);
  }

  if (connector_) {
    connector_->disconnect();
  }
}

bool LiDARNode::detect_port()
{
  // HARDWIRED MODE: port is configured via LIDAR_PHYSICAL_PATH (default parameter).
  // Verify it exists and return immediately. No probing.
  if (!port_.empty()) {
    if (SerialPort::path_exists(port_)) {
      char resolved[PATH_MAX];
      RCLCPP_INFO(
        get_logger(),
        "[detect_port] using configured port: %s  [resolved: %s]",
        port_.c_str(),
        realpath(port_.c_str(), resolved) ? resolved : "(unresolved)");
      return true;
    }
    RCLCPP_ERROR(
      get_logger(),
      "[detect_port] configured port %s does not exist",
      port_.c_str());
    return false;
  }

  RCLCPP_ERROR(get_logger(), "[detect_port] No port configured and auto-detect is disabled");
  return false;
}

void LiDARNode::initialize_lidar()
{
  // === AGV HARDWARE COLLISION CHECK ===
  // IMU and LiDAR share the same CP2102 VID/PID with duplicate serial numbers
  // on adjacent hub ports.  This guard detects a pathological case where both
  // symlinks resolve to the same ttyUSB device (e.g. USB topology change after
  // reboot or hub reset) and aborts before either driver claims exclusive access.
  {
    char imu_resolved[PATH_MAX];
    char lidar_resolved[PATH_MAX];
    if (realpath(IMU_PHYSICAL_PATH, imu_resolved) &&
      realpath(LIDAR_PHYSICAL_PATH, lidar_resolved))
    {
      if (std::string(imu_resolved) == std::string(lidar_resolved)) {
        RCLCPP_FATAL(
          get_logger(),
          "FATAL SERIAL PORT COLLISION: IMU and LiDAR both resolve to %s — "
          "IMU path: %s  LiDAR path: %s  refusing to start",
          lidar_resolved, IMU_PHYSICAL_PATH, LIDAR_PHYSICAL_PATH);
        throw std::runtime_error("Serial port collision between IMU and LiDAR");
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "[AGV DEVICE MAP]  IMU: %s → %s  |  LiDAR: %s → %s",
      IMU_PHYSICAL_PATH,
      realpath(IMU_PHYSICAL_PATH, imu_resolved) ? imu_resolved : "(absent)",
      LIDAR_PHYSICAL_PATH,
      realpath(LIDAR_PHYSICAL_PATH, lidar_resolved) ? lidar_resolved : "(absent)");
  }

  // Only invoke detect_port() as a true fallback — when port_ was never set.
  if (port_.empty()) {
    RCLCPP_INFO(get_logger(), "[initialize_lidar] no port configured — running detect_port()");
    if (!detect_port()) {
      return;
    }
  } else {
    RCLCPP_INFO(
      get_logger(),
      "[initialize_lidar] port already configured (%s) — skipping detect_port()",
      port_.c_str());
  }

  if (port_.empty()) {
    RCLCPP_ERROR(get_logger(), "No valid port found");
    return;
  }

  {
    char resolved[PATH_MAX];
    RCLCPP_INFO(
      get_logger(),
      "Opening serial port %s @ %d baud  [resolved: %s]",
      port_.c_str(), baudrate_,
      realpath(port_.c_str(), resolved) ? resolved : "(unresolved)");
  }

  connector_ = std::make_unique<LiDARConnector>(port_, baudrate_);
  connector_->set_on_connected([this] { on_lidar_connected(); });
  connector_->set_on_disconnected([this] { on_lidar_disconnected(); });
  connector_->set_on_error([this](const std::string & err) { on_lidar_error(err); });

  if (!connector_->connect()) {
    RCLCPP_INFO(get_logger(), "[LIDAR-WAIT] serial transport not open yet; watchdog/reconnect remains armed");
    return;
  }

  // === AGV DEVICE MAP: log resolved port + exclusive ownership on success ===
  {
    char resolved[PATH_MAX];
    const char * rp = realpath(port_.c_str(), resolved) ? resolved : "(unresolved)";
    RCLCPP_INFO(
      get_logger(),
      "[AGV DEVICE MAP] LiDAR CONNECTED on resolved port %s  baud=%d  EXCLUSIVE=TIOCEXCL",
      rp, baudrate_);
  }

  connector_->start_auto_reconnect();
  motor_ = std::make_unique<LiDARMotor>(*connector_);

  double configured_hz = 0.0;
  if (motor_->set_scan_frequency(scan_frequency_, configured_hz)) {
    hardware_scan_frequency_ = configured_hz;
    RCLCPP_INFO(
      get_logger(),
      "[LIDAR-FREQ] requested=%.1f Hz hardware=%.1f Hz VERIFIED",
      scan_frequency_, hardware_scan_frequency_);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "[LIDAR-FREQ] hardware query/set failed; preserving existing device frequency and continuing with normal scan startup");
  }

  if (!motor_->start_scan(1500)) {
    RCLCPP_INFO(
      get_logger(),
      "[LIDAR-MOTOR] first start command not acknowledged yet; recovery path remains armed");
    // Still start reader; reconnect path may recover.
  }

  reader_ = std::make_unique<ScanReader>(
    *connector_, intensity_mode_, intensity_bits_, strict_checksum_);
  reader_->set_on_scan_received([this](const ScanData & scan) { on_scan_received(scan); });
  reader_->start();
  {
    std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
    // Start the stream watchdog even before the first complete revolution, but
    // give the motor/parser enough time to reacquire a real ring marker.
    const auto now = std::chrono::steady_clock::now();
    last_scan_wall_time_ = now;
    last_published_scan_wall_time_ = now;
    recovery_grace_until_ = now + std::chrono::milliseconds(
      static_cast<int>(recovery_grace_sec_ * 1000.0));
    connector_unhealthy_since_ = {};
  }

  RCLCPP_INFO(
    get_logger(),
    "LiDAR initialization complete [V15 two-stage recovery: soft-first, grace=%.1fs]",
    recovery_grace_sec_);
}

void LiDARNode::ensure_motor_and_reader()
{
  // V15 intentionally does NOT call LiDARMotor::start_scan() while ScanReader
  // is active. start_scan() probes the same serial stream for AA55 bytes; doing
  // that concurrently with ScanReader can steal packet bytes and manufacture a
  // false stream stall. The unified scan-age watchdog below owns all recovery.
  if (manual_stop_requested_.load() || !connector_ || !connector_->is_open()) {
    return;
  }
}

void LiDARNode::soft_recover_lidar_stream(const std::string & reason)
{
  if (manual_stop_requested_.load()) {
    return;
  }

  bool expected = false;
  if (!recovery_in_progress_.compare_exchange_strong(expected, true)) {
    return;
  }

  ++soft_stream_recoveries_;
  const int soft_no = ++soft_recovery_failures_;
  RCLCPP_INFO(
    get_logger(),
    "[LIDAR-SOFT-RECOVERY] #%d reason=%s; preserving USB enumeration and resyncing parser/motor",
    soft_no, reason.c_str());

  try {
    // IMPORTANT: do not power-cycle or destroy the CP210x descriptor on the
    // first stream stall. Movement/vibration can create a short packet gap; a
    // hard restart here was creating a restart storm in the 20260819 log.
    if (!connector_ || !connector_->is_open()) {
      recovery_in_progress_.store(false);
      restart_lidar_stack("soft recovery found serial transport closed");
      return;
    }

    if (reader_) {
      reader_->stop();
      reader_->reset_accumulator();
    }
    connector_->flush();

    if (!motor_) {
      motor_ = std::make_unique<LiDARMotor>(*connector_);
    }
    const bool motor_ok = motor_->start_scan(700);

    if (!reader_) {
      reader_ = std::make_unique<ScanReader>(
    *connector_, intensity_mode_, intensity_bits_, strict_checksum_);
      reader_->set_on_scan_received([this](const ScanData & scan) { on_scan_received(scan); });
    }
    reader_->reset_accumulator();
    reader_->start();

    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
      last_scan_wall_time_ = now;
      // Do not immediately declare the just-restarted stream stale. The sensor
      // must first pass a real ring marker and complete a physical revolution.
      recovery_grace_until_ = now + std::chrono::milliseconds(
        static_cast<int>(recovery_grace_sec_ * 1000.0));
    }

    RCLCPP_INFO(
      get_logger(),
      "[LIDAR-SOFT-RECOVERY] reader restarted, motor=%s; waiting %.1fs for fresh scan",
      motor_ok ? "RUNNING" : "START-PENDING", recovery_grace_sec_);
  } catch (const std::exception & e) {
    RCLCPP_INFO(get_logger(), "[LIDAR-SOFT-RECOVERY] exception: %s", e.what());
  }

  recovery_in_progress_.store(false);
}

void LiDARNode::restart_lidar_stack(const std::string & reason)
{
  if (manual_stop_requested_.load()) {
    return;
  }
  // A CP210x can remain open while the LiDAR itself has stopped streaming.
  // Re-opening only the same file descriptor was not enough in the failing
  // 2026-08-18 log: the port reported CONNECTED, but A5 60/82/90 never saw
  // AA55 again. Rebuild the complete reader/motor/connector stack and give the
  // motor-control lines a real OFF interval before reopening the by-path port.
  bool expected = false;
  if (!recovery_in_progress_.compare_exchange_strong(expected, true)) {
    return;
  }

  const int failure_no = ++consecutive_stream_failures_;
  ++hard_stream_recoveries_;
  // V15: hard power cycling is the SECOND stage only. Keep the OFF pulse long
  // enough to reset the LiDAR, but avoid multi-second restart storms.
  const auto power_off_delay = std::chrono::milliseconds(failure_no >= 3 ? 1000 : 600);

  RCLCPP_INFO(
    get_logger(),
    "[LIDAR-FULL-RESTART] #%d reason=%s; rebuilding serial+motor+reader (power-off %ld ms)",
    failure_no, reason.c_str(), static_cast<long>(power_off_delay.count()));

  try {
    // Stop reader first: it owns the hot read loop and must not race teardown.
    if (reader_) {
      reader_->stop();
      reader_->reset_accumulator();
    }

    // Stop the reconnect worker before touching connector internals.
    if (connector_) {
      connector_->stop_auto_reconnect();
    }

    // Ask the LiDAR to stop, then force RTS/DTR motor power OFF. Even when the
    // command channel is wedged, the control-line toggle provides a clean edge.
    if (motor_ && connector_ && connector_->is_open()) {
      motor_->stop_scan();
    } else if (connector_ && connector_->is_open()) {
      connector_->set_motor_power(false);
    }

    if (connector_) {
      connector_->disconnect();
    }

    reader_.reset();
    motor_.reset();
    connector_.reset();

    {
      std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
      last_scan_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      last_scan_wall_time_ = {};
      last_published_scan_wall_time_ = {};
      scan_stale_active_.store(false);
    }

    std::this_thread::sleep_for(power_off_delay);

    // Wait briefly for a by-path target to return after a cable/hub bounce.
    for (int i = 0; i < 30 && !SerialPort::path_exists(port_); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    initialize_lidar();
    const auto reopened_now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
      connector_unhealthy_since_ = {};
      if (last_scan_wall_time_.time_since_epoch().count() == 0) {
        last_scan_wall_time_ = reopened_now;
      }
      if (last_published_scan_wall_time_.time_since_epoch().count() == 0) {
        last_published_scan_wall_time_ = reopened_now;
      }
      recovery_grace_until_ = reopened_now + std::chrono::milliseconds(
        static_cast<int>(recovery_grace_sec_ * 1000.0));
    }

    if (connector_ && connector_->is_open()) {
      RCLCPP_INFO(
        get_logger(),
        "[LIDAR-FULL-RESTART] serial stack reopened; waiting for fresh physical scans");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "[LIDAR-FULL-RESTART] by-path still unavailable; watchdog will retry");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "[LIDAR-FULL-RESTART] exception: %s", e.what());
  }

  recovery_in_progress_.store(false);
}

void LiDARNode::on_scan_received(const ScanData & scan)
{
  if (scan.points.empty()) {
    return;
  }
  // A complete physical revolution proves the motor/transport is moving, but
  // only a successfully published quality-checked scan clears recovery debt.
  // This prevents a stream of corrupt/near-empty revolutions from masking a
  // stale /scan topic forever.
  if (motor_) {
    motor_->mark_stream_running();
  }

  std::vector<float> new_scan(kScanBins, 0.0f);
  for (const auto & pt : scan.points) {
    if (!pt.valid) {
      continue;
    }
    if (pt.distance_m < range_min_ || pt.distance_m > range_max_) {
      continue;
    }

    double angle = std::fmod(pt.angle_deg + angle_offset_deg_, 360.0);
    if (angle < 0.0) {
      angle += 360.0;
    }
    if (invert_angle_) {
      // Convert CW device angles to CCW ROS convention.
      angle = std::fmod(360.0 - angle, 360.0);
    }

    const int idx = static_cast<int>(std::round(angle)) % static_cast<int>(kScanBins);
    if (idx < 0) {
      continue;
    }
    // Keep nearest return when multiple points map to same bin.
    if (new_scan[static_cast<size_t>(idx)] <= 0.0f ||
      pt.distance_m < static_cast<double>(new_scan[static_cast<size_t>(idx)]))
    {
      new_scan[static_cast<size_t>(idx)] = static_cast<float>(pt.distance_m);
    }
  }

  const rclcpp::Time source_stamp = scan.stamp.nanoseconds() > 0 ? scan.stamp : this->now();

  // A completed hardware revolution counts as live transport even if the scan
  // is later rejected as too sparse. This prevents the stream watchdog from
  // power-cycling a healthy LiDAR merely because the environment has few returns.
  {
    std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
    last_scan_wall_time_ = std::chrono::steady_clock::now();
    recovery_grace_until_ = {};
  }

  // V6 anti-starburst cleanup has two independent gates:
  //   A) local/temporal support rejects isolated or short-lived returns;
  //   B) the two-sided median test rejects FAR radial shadows between surfaces.
  // Real near foreground is explicitly protected. Sparse real geometry can be
  // admitted after it repeats in consecutive physical revolutions.
  std::vector<float> filtered_scan = new_scan;
  size_t raw_valid_bins = 0U;
  size_t kept_valid_bins = 0U;
  uint64_t removed_this_scan = 0U;
  for (const float r : new_scan) {
    if (r > 0.0f && std::isfinite(r)) {
      ++raw_valid_bins;
    }
  }

  if (spatial_filter_enabled_ && raw_valid_bins > 0U) {
    const auto wrapped_index = [](long value) -> size_t {
      constexpr long bin_count = static_cast<long>(kScanBins);
      value %= bin_count;
      if (value < 0) value += bin_count;
      return static_cast<size_t>(value);
    };
    auto median_of = [](std::vector<float> values) -> float {
      if (values.empty()) return 0.0f;
      const auto middle = values.begin() + static_cast<long>(values.size() / 2U);
      std::nth_element(values.begin(), middle, values.end());
      return *middle;
    };

    auto history_has_support = [&](const std::vector<float> & history, size_t index, float range) {
      if (!temporal_filter_enabled_ || history.size() != new_scan.size()) {
        return false;
      }
      for (int d = -temporal_filter_radius_bins_; d <= temporal_filter_radius_bins_; ++d) {
        const float prev = history[wrapped_index(static_cast<long>(index) + d)];
        if (prev > 0.0f && std::isfinite(prev) &&
          std::abs(static_cast<double>(prev) - static_cast<double>(range)) <=
          temporal_filter_range_diff_m_)
        {
          return true;
        }
      }
      return false;
    };

    auto temporal_support = [&](size_t index, float range) {
      if (!temporal_filter_enabled_) return false;
      const bool prev1 = history_has_support(previous_raw_scan_, index, range);
      if (!prev1) return false;
      // Once two previous revolutions exist, require support in BOTH.  This
      // prevents one bad revolution from becoming its own temporal evidence.
      if (previous_raw_scan_2_.size() == new_scan.size()) {
        return history_has_support(previous_raw_scan_2_, index, range);
      }
      return true;
    };

    // Stage A — local support / temporal persistence.  Restrict support radius
    // to +/-3 degrees so unrelated walls across corners cannot validate a ray.
    const int local_radius = std::max(1, std::min(3, spatial_filter_radius_bins_));
    for (size_t i = 0; i < kScanBins; ++i) {
      const float r = new_scan[i];
      if (!(r > 0.0f) || !std::isfinite(r)) continue;

      int compact_endpoint_neighbors = 0;
      int same_range_neighbors = 0;
      float nearest_other = std::numeric_limits<float>::infinity();
      bool have_other = false;
      for (int step = 1; step <= local_radius; ++step) {
        for (const int sign : {-1, 1}) {
          const float nbr = new_scan[
            wrapped_index(static_cast<long>(i) + static_cast<long>(sign * step))];
          if (!(nbr > 0.0f) || !std::isfinite(nbr)) continue;
          have_other = true;
          nearest_other = std::min(nearest_other, nbr);

          // Range-difference alone rejects legitimate oblique walls. Compare
          // Cartesian endpoint spacing for the known angular separation.
          const double dtheta = static_cast<double>(step) * M_PI / 180.0;
          const double endpoint_gap_sq =
            static_cast<double>(r) * static_cast<double>(r) +
            static_cast<double>(nbr) * static_cast<double>(nbr) -
            2.0 * static_cast<double>(r) * static_cast<double>(nbr) * std::cos(dtheta);
          const double endpoint_gap = std::sqrt(std::max(0.0, endpoint_gap_sq));
          if (endpoint_gap <= spatial_endpoint_max_gap_m_) {
            ++compact_endpoint_neighbors;
          }

          const double tol = spatial_filter_abs_tolerance_m_ +
            spatial_filter_rel_tolerance_ * std::min<double>(r, nbr);
          if (std::abs(static_cast<double>(nbr) - static_cast<double>(r)) <= tol) {
            ++same_range_neighbors;
          }
        }
      }

      const bool temporally_supported = temporal_support(i, r);
      // First sighting requires strong local geometry. Once a return repeats
      // in two physical revolutions, one compact neighbour is enough. This
      // preserves small/oblique real obstacles without letting a persistent
      // isolated radial pixel validate itself from RAW temporal history.
      const bool locally_supported =
        compact_endpoint_neighbors >= spatial_filter_min_support_ ||
        (compact_endpoint_neighbors >= 1 && same_range_neighbors >= 1) ||
        (compact_endpoint_neighbors >= 1 && temporally_supported);
      bool near_foreground = false;
      if (have_other && std::isfinite(nearest_other)) {
        const double fg_tol = spatial_filter_abs_tolerance_m_ +
          spatial_filter_rel_tolerance_ * std::min<double>(r, nearest_other);
        near_foreground = static_cast<double>(r) + fg_tol < static_cast<double>(nearest_other);
      }

      if (!locally_supported && !near_foreground) {
        filtered_scan[i] = 0.0f;
        ++removed_this_scan;
      }
    }

    // Stage B — two-sided FAR-shadow rejection.  Use RAW side medians to
    // detect the physical geometry, but only evaluate points that survived A.
    std::vector<float> left_values;
    std::vector<float> right_values;
    left_values.reserve(static_cast<size_t>(spatial_filter_radius_bins_));
    right_values.reserve(static_cast<size_t>(spatial_filter_radius_bins_));

    for (size_t i = 0; i < kScanBins; ++i) {
      const float r = filtered_scan[i];
      if (!(r > 0.0f) || !std::isfinite(r)) continue;

      left_values.clear();
      right_values.clear();
      for (int step = 1; step <= spatial_filter_radius_bins_; ++step) {
        const float left = new_scan[wrapped_index(static_cast<long>(i) - step)];
        const float right = new_scan[wrapped_index(static_cast<long>(i) + step)];
        if (left > 0.0f && std::isfinite(left)) left_values.push_back(left);
        if (right > 0.0f && std::isfinite(right)) right_values.push_back(right);
      }

      if (left_values.size() < static_cast<size_t>(spatial_filter_min_support_) ||
        right_values.size() < static_cast<size_t>(spatial_filter_min_support_))
      {
        continue;
      }

      const float left_median = median_of(left_values);
      const float right_median = median_of(right_values);
      const double side_tol = spatial_filter_abs_tolerance_m_ +
        spatial_filter_rel_tolerance_ * std::min<double>(left_median, right_median);
      if (std::abs(static_cast<double>(left_median) - static_cast<double>(right_median)) > side_tol) {
        continue;  // real edge/corner: two sides are different surfaces
      }

      const double left_tol = spatial_filter_abs_tolerance_m_ +
        spatial_filter_rel_tolerance_ * std::min<double>(r, left_median);
      const double right_tol = spatial_filter_abs_tolerance_m_ +
        spatial_filter_rel_tolerance_ * std::min<double>(r, right_median);
      const bool far_shadow =
        static_cast<double>(r) > static_cast<double>(left_median) + left_tol &&
        static_cast<double>(r) > static_cast<double>(right_median) + right_tol;
      if (!far_shadow) continue;

      // V7: a short FAR run between two consistent near surfaces is the
      // signature of a radial shadow/spike. Do NOT allow RAW temporal history
      // to rescue it: systematic LiDAR artifacts can repeat at the same angle
      // and were therefore able to survive V6. Wider runs are treated as a
      // possible real opening / far wall and are preserved.
      const double far_gate = std::max<double>(left_median, right_median) +
        std::max(left_tol, right_tol);
      int far_run = 1;
      for (int step = 1; step <= far_shadow_max_run_bins_; ++step) {
        const float v = new_scan[wrapped_index(static_cast<long>(i) - step)];
        if (!(v > far_gate) || !std::isfinite(v)) break;
        ++far_run;
      }
      for (int step = 1; step <= far_shadow_max_run_bins_; ++step) {
        const float v = new_scan[wrapped_index(static_cast<long>(i) + step)];
        if (!(v > far_gate) || !std::isfinite(v)) break;
        ++far_run;
      }
      if (far_run <= far_shadow_max_run_bins_) {
        filtered_scan[i] = 0.0f;
        ++removed_this_scan;
      } else if (!temporal_support(i, r)) {
        // A wide first-seen opening is allowed; a wide but weakly supported
        // tail is still removed after history has warmed.
        const bool history_warm = previous_raw_scan_2_.size() == new_scan.size();
        if (history_warm) {
          filtered_scan[i] = 0.0f;
          ++removed_this_scan;
        }
      }
    }
  }

  for (const float r : filtered_scan) {
    if (r > 0.0f && std::isfinite(r)) ++kept_valid_bins;
  }
  filtered_isolated_bins_ += removed_this_scan;

  // Keep two RAW physical revolutions as independent temporal evidence.  RAW
  // history is intentional: a filtered point cannot recursively reinforce
  // itself, but a real persistent return can re-enter after repeat evidence.
  previous_raw_scan_2_ = previous_raw_scan_;
  previous_raw_scan_ = new_scan;

  // PART-4 Stage-2: a safety scan and a navigation scan have intentionally
  // different acceptance policies.  A physically valid revolution must reach
  // the safety path BEFORE anti-starburst filtering.  Navigation may reject an
  // over-filtered revolution without power-cycling a healthy LiDAR.
  const size_t hard_min_valid = std::max<size_t>(3U,
    static_cast<size_t>(std::max(1, min_valid_bins_per_scan_ / 8)));
  const bool hard_sparse = raw_valid_bins < hard_min_valid;
  if (hard_sparse) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "[LIDAR-INVALID-REV] raw=%zu hard_min=%zu; publish SAFETY for immediate health hold, NAV withheld",
      raw_valid_bins, hard_min_valid);
  }

  const bool sparse_quality =
    hard_sparse ||
    raw_valid_bins < static_cast<size_t>(min_valid_bins_per_scan_) ||
    kept_valid_bins < static_cast<size_t>(min_valid_bins_per_scan_);
  if (sparse_quality) {
    ++dropped_sparse_scans_;  // quality counter: below preferred density
    ++sparse_published_scans_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "[LIDAR-SPARSE] raw=%zu filtered=%zu preferred_min=%d",
      raw_valid_bins, kept_valid_bins, min_valid_bins_per_scan_);
  }

  // Publish once per newly completed physical revolution. Do not replay a
  // previous buffer from a 10 Hz timer when the sensor has not delivered one.
  double measured_period = target_scan_period_;
  {
    std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
    if (last_scan_stamp_.nanoseconds() > 0) {
      measured_period = (source_stamp - last_scan_stamp_).seconds();
      if (measured_period <= 0.0) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping completed scan with non-monotonic acquisition timestamp");
        return;
      }
    }
    last_scan_stamp_ = source_stamp;
    last_scan_wall_time_ = std::chrono::steady_clock::now();
  }

  // SAFETY FIRST: every completed physical revolution is published before
  // anti-starburst filtering, even when it is sparse. Keep the acquisition
  // timestamp on the safety stream so health diagnostics retain the physical
  // timing evidence.
  publish_safety_scan(new_scan, source_stamp, measured_period);

  // Navigation consumers (AMCL/costmaps) require a scan timestamp that is still
  // represented in the live TF cache. Under heavy Jetson load a completed scan
  // can sit in the callback queue for >1 s even though the LiDAR itself is healthy.
  // Preserve the real acquisition stamp while it is fresh, but clamp only the
  // NAVIGATION product when queue latency exceeds 350 ms. This prevents TF
  // MessageFilter drops without changing the safety stream or replaying scans.
  rclcpp::Time navigation_stamp = source_stamp;
  const rclcpp::Time publish_now = this->now();
  const double navigation_stamp_age = (publish_now - source_stamp).seconds();
  if (navigation_stamp_age > 0.35 || navigation_stamp_age < -0.05) {
    navigation_stamp = publish_now;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "[LIDAR-NAV-STAMP] acquisition stamp age %.3fs outside TF-safe window; clamped to publish time",
      navigation_stamp_age);
  }

  if (hard_sparse || kept_valid_bins == 0U) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "[LIDAR-NAV-REJECT] raw=%zu filtered=%zu hard_min=%zu; navigation scan held",
      raw_valid_bins, kept_valid_bins, hard_min_valid);
    return;
  }
  publish_scan(filtered_scan, navigation_stamp, measured_period);
}

void LiDARNode::publish_safety_scan(
  const std::vector<float> & snapshot, const rclcpp::Time & stamp,
  double measured_scan_period)
{
  auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id_;
  msg->angle_min = static_cast<float>(angle_min_deg_ * M_PI / 180.0);
  msg->angle_max = static_cast<float>(angle_max_deg_ * M_PI / 180.0);
  msg->angle_increment = static_cast<float>(M_PI / 180.0);
  msg->scan_time = static_cast<float>(measured_scan_period);
  msg->time_increment = static_cast<float>(measured_scan_period / kScanBins);
  msg->range_min = static_cast<float>(range_min_);
  msg->range_max = static_cast<float>(range_max_);
  msg->ranges.resize(kScanBins);
  for (size_t i = 0; i < kScanBins; ++i) {
    const float r = snapshot[i];
    msg->ranges[i] = (r > 0.0f && std::isfinite(r)) ?
      r : std::numeric_limits<float>::infinity();
  }
  msg->intensities.assign(kScanBins, 0.0f);
  safety_scan_pub_->publish(std::move(msg));
  ++safety_published_scans_;
  consecutive_stream_failures_.store(0);
  soft_recovery_failures_.store(0);
  {
    std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
    // Hardware recovery follows the minimally processed safety stream.  A
    // navigation-filter rejection is not a USB/motor failure.
    last_published_scan_wall_time_ = std::chrono::steady_clock::now();
  }
  scan_stale_active_.store(false);
}

void LiDARNode::publish_scan(
  const std::vector<float> & snapshot, const rclcpp::Time & stamp,
  double measured_scan_period)
{
  auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id_;

  msg->angle_min = static_cast<float>(angle_min_deg_ * M_PI / 180.0);
  msg->angle_max = static_cast<float>(angle_max_deg_ * M_PI / 180.0);
  msg->angle_increment = static_cast<float>(M_PI / 180.0);
  msg->scan_time = static_cast<float>(measured_scan_period);
  msg->time_increment = static_cast<float>(measured_scan_period / kScanBins);
  msg->range_min = static_cast<float>(range_min_);
  msg->range_max = static_cast<float>(range_max_);

  msg->ranges.resize(kScanBins);
  for (size_t i = 0; i < kScanBins; ++i) {
    const float r = snapshot[i];
    if (r > 0.0f && std::isfinite(r)) {
      msg->ranges[i] = r;
    } else {
      // Invalid/no-return sample: LaserScan consumers expect +inf or NaN for
      // no return. Do not use 0.0 because it is below range_min and can look
      // like a false obstacle at the sensor origin in mapping/visualization.
      // Use +inf externally but keep internal logic unaffected.
      msg->ranges[i] = std::numeric_limits<float>::infinity();
    }
  }
  msg->intensities.assign(kScanBins, 0.0f);

  scan_pub_->publish(std::move(msg));
  ++published_scans_;
}

void LiDARNode::monitor_status_callback()
{
  if (manual_stop_requested_.load()) {
    return;
  }

  if (!connector_) {
    if (SerialPort::path_exists(port_)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "[LIDAR-RECOVERY] connector absent; rebuilding on %s", port_.c_str());
      restart_lidar_stack("connector absent after USB/serial fault");
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "[LIDAR-RECOVERY] waiting for physical by-path %s", port_.c_str());
    }
    return;
  }

  try {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto conn_status = connector_->state();
    const std::string motor_state = motor_ ? motor_->get_motor_state() : "N/A";
    auto status_msg = std::make_unique<std_msgs::msg::String>();
    status_msg->data =
      "Connected: " +
      std::string(conn_status == LiDARConnector::State::Connected ? "true" : "false") +
      " | Port: " + port_ +
      " | Baud: " + std::to_string(baudrate_) +
      " | Motor: " + motor_state +
      " | SafetyScans: " + std::to_string(safety_published_scans_.load()) +
      " | NavScans: " + std::to_string(published_scans_.load()) +
      " | SparsePub: " + std::to_string(sparse_published_scans_.load()) +
      " | ScanTimeouts: " + std::to_string(scan_timeouts_.load()) +
      " | SoftRecoveries: " + std::to_string(soft_stream_recoveries_.load()) +
      " | HardRecoveries: " + std::to_string(hard_stream_recoveries_.load()) +
      " | ValidPackets: " + std::to_string(reader_ ? reader_->valid_packets() : 0U) +
      " | InvalidPackets: " + std::to_string(reader_ ? reader_->invalid_packets() : 0U) +
      " | MalformedPackets: " + std::to_string(reader_ ? reader_->malformed_packets() : 0U) +
      " | ChecksumFailures: " + std::to_string(reader_ ? reader_->checksum_failures() : 0U) +
      " | StrictChecksum: " + std::string(strict_checksum_ ? "true" : "false");
    status_pub_->publish(std::move(status_msg));

    if (conn_status != LiDARConnector::State::Connected || !connector_->is_open()) {
      double unhealthy_age = 0.0;
      {
        std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
        if (connector_unhealthy_since_.time_since_epoch().count() == 0) {
          connector_unhealthy_since_ = steady_now;
        }
        unhealthy_age =
          std::chrono::duration<double>(steady_now - connector_unhealthy_since_).count();
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "[LIDAR-USB-RECOVERY] transport reconnecting for %.2f s on %s",
        unhealthy_age, port_.c_str());

      // Let the connector's own by-path reconnect worker do its job first. A
      // full power cycle while USB is re-enumerating was the main cause of the
      // repeated disappear/restart loop in the supplied runtime log.
      if (unhealthy_age >= hard_restart_delay_sec_ && SerialPort::path_exists(port_)) {
        const bool cooldown_elapsed =
          last_stream_recovery_.time_since_epoch().count() == 0 ||
          std::chrono::duration<double>(steady_now - last_stream_recovery_).count() >=
            hard_restart_delay_sec_;
        if (cooldown_elapsed) {
          last_stream_recovery_ = steady_now;
          ++stream_recoveries_;
          restart_lidar_stack("serial transport did not recover within hard-restart delay");
        }
      }
      return;
    }

    {
      std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
      connector_unhealthy_since_ = {};
    }
    ensure_motor_and_reader();

    double physical_scan_age = 0.0;
    double published_scan_age = 0.0;
    bool in_recovery_grace = false;
    {
      std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
      if (last_scan_wall_time_.time_since_epoch().count() != 0) {
        physical_scan_age =
          std::chrono::duration<double>(steady_now - last_scan_wall_time_).count();
      }
      if (last_published_scan_wall_time_.time_since_epoch().count() != 0) {
        published_scan_age =
          std::chrono::duration<double>(steady_now - last_published_scan_wall_time_).count();
      }
      in_recovery_grace = recovery_grace_until_.time_since_epoch().count() != 0 &&
        steady_now < recovery_grace_until_;
    }
    const double scan_age = std::max(physical_scan_age, published_scan_age);

    if (scan_age > scan_timeout_ && !in_recovery_grace) {
      bool expected_stale = false;
      if (scan_stale_active_.compare_exchange_strong(expected_stale, true)) {
        ++scan_timeouts_;
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2500,
        "LiDAR freshness delayed: physical=%.3fs published=%.3fs; recovery watchdog armed",
        physical_scan_age, published_scan_age);
    } else if (scan_age <= scan_timeout_) {
      scan_stale_active_.store(false);
    }

    if (in_recovery_grace) {
      return;
    }

    const bool recovery_cooldown_elapsed =
      last_stream_recovery_.time_since_epoch().count() == 0 ||
      std::chrono::duration<double>(steady_now - last_stream_recovery_).count() >=
        stream_recovery_cooldown_;

    if (scan_age > stream_recovery_timeout_ && recovery_cooldown_elapsed) {
      last_stream_recovery_ = steady_now;
      ++stream_recoveries_;

      if (soft_recovery_failures_.load() < soft_recovery_attempts_before_hard_) {
        soft_recover_lidar_stream(
          "completed scan timeout while USB transport remains connected");
      } else {
        RCLCPP_INFO(
          get_logger(),
          "[LIDAR-HARD-RECOVERY] %d soft recoveries produced no fresh revolution; escalating once",
          soft_recovery_failures_.load());
        soft_recovery_failures_.store(0);
        restart_lidar_stack(
          "soft stream resync exhausted while serial transport remained connected");
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_INFO(get_logger(), "LiDAR monitor exception; watchdog will retry: %s", e.what());
  }
}

void LiDARNode::on_lidar_connected()
{
  RCLCPP_INFO(get_logger(), "[INFO] LIDAR CONNECTED");

  // A reconnect callback runs on the connector worker, while the watchdog runs
  // in the ROS executor. Serialize stream re-arming with the same recovery flag
  // used by soft/hard recovery so two threads never stop/probe/start the reader
  // at the same time.
  bool expected = false;
  if (!recovery_in_progress_.compare_exchange_strong(expected, true)) {
    return;
  }

  try {
    // Reconnect callbacks can arrive while ScanReader already exists. Stop it
    // before probing AA55 so two consumers never read the same serial bytes.
    if (reader_) {
      reader_->stop();
      reader_->reset_accumulator();
    }
    if (motor_) {
      motor_->start_scan(900);
    }
    if (reader_) {
      reader_->start();
    }

    {
      std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
      const auto now = std::chrono::steady_clock::now();
      last_scan_wall_time_ = now;
      last_published_scan_wall_time_ = now;
      recovery_grace_until_ = now + std::chrono::milliseconds(
        static_cast<int>(recovery_grace_sec_ * 1000.0));
      connector_unhealthy_since_ = {};
    }
  } catch (const std::exception & e) {
    RCLCPP_INFO(get_logger(), "[LIDAR-RECONNECT] stream re-arm exception: %s", e.what());
  }

  recovery_in_progress_.store(false);
}

void LiDARNode::on_lidar_disconnected()
{
  RCLCPP_INFO(get_logger(), "[WARNING] LIDAR DISCONNECTED");
}

void LiDARNode::on_lidar_error(const std::string & error_msg)
{
  RCLCPP_INFO(get_logger(), "[LIDAR-TRANSPORT] %s", error_msg.c_str());

  // Do not destroy the connector from its own reconnect callback. A failed
  // open can be reported by the reconnect worker itself; joining/resetting that
  // worker from this callback can self-deadlock. The monitor callback owns
  // escalation and the connector worker keeps re-resolving the stable by-path.
  if (!SerialPort::path_exists(port_)) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "[LIDAR-TRANSPORT] physical alias temporarily absent: %s", port_.c_str());
  }
}

void LiDARNode::log_scan_reader_stats()
{
  if (!reader_) {
    RCLCPP_INFO(get_logger(), "[ScanReader] not started");
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "[ScanReader] mode=%s intensity_bits=%d bytes/sample=%zu complete=%lu ring=%lu incomplete_rev=%lu "
    "valid_packets=%lu invalid_packets=%lu malformed=%lu checksum_fail=%lu checksum_drop=%lu "
    "parser_locked=%s layout_switches=%lu pending_points=%zu published=%lu sparse_quality=%lu "
    "sparse_published=%lu isolated_bins_removed=%lu recoveries=%lu",
    reader_->sample_bytes() == 4U ? "4B-SH" : (reader_->sample_bytes() == 3U ? "3B" : "2B"),
    reader_->intensity_bits(), reader_->sample_bytes(),
    static_cast<unsigned long>(reader_->complete_scans()),
    static_cast<unsigned long>(reader_->ring_starts()),
    static_cast<unsigned long>(reader_->incomplete_revolutions()),
    static_cast<unsigned long>(reader_->valid_packets()),
    static_cast<unsigned long>(reader_->invalid_packets()),
    static_cast<unsigned long>(reader_->malformed_packets()),
    static_cast<unsigned long>(reader_->checksum_failures()),
    static_cast<unsigned long>(reader_->checksum_drops()),
    reader_->parser_layout_locked() ? "true" : "false",
    static_cast<unsigned long>(reader_->parser_layout_switches()),
    reader_->pending_points(),
    static_cast<unsigned long>(published_scans_.load()),
    static_cast<unsigned long>(dropped_sparse_scans_.load()),
    static_cast<unsigned long>(sparse_published_scans_.load()),
    static_cast<unsigned long>(filtered_isolated_bins_.load()),
    static_cast<unsigned long>(stream_recoveries_.load()));
}

}  // namespace b1_lidar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<b1_lidar::LiDARNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
