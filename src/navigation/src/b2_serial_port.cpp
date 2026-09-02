#include "imu_ros2/serial_port.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

namespace imu_ros2
{
namespace
{
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
  std::cerr << "[IMU-USB-RECOVERY] kernel tty I/O fault on " << port
            << " (" << std::strerror(error_code)
            << "); requesting serialized CP210x repair" << std::endl;
  // Run the role-specific helper synchronously and bounded.  Do NOT append '&':
  // an untracked background recovery can outlive this mapping session, keep the
  // global recovery lock, and collide with the next START after STOP/SAVE.
  const int recovery_rc = std::system(
    "timeout 25s sudo -n /usr/local/sbin/agv-sensor-recover imu "
    ">>/tmp/agv_sensor_recover.log 2>&1");
  if (recovery_rc == -1) {
    std::cerr << "[IMU-USB-RECOVERY] failed to start recovery helper: "
              << std::strerror(errno) << std::endl;
  } else if (recovery_rc != 0) {
    std::cerr << "[IMU-USB-RECOVERY] recovery helper exited rc="
              << recovery_rc << "; reconnect will retry" << std::endl;
  }
}
}  // namespace

SerialPort::SerialPort(std::string port, int baudrate, double timeout_sec)
: port_(std::move(port))
, baudrate_(baudrate)
, timeout_sec_(timeout_sec)
, fd_(-1)
{}

SerialPort::~SerialPort()
{
  close();
}

speed_t SerialPort::baud_to_constant(int baudrate)
{
  switch (baudrate) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default:     return B115200;
  }
}

bool SerialPort::configure_port()
{
  struct termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    const int saved_errno = errno;
    std::cerr << "[IMU-TRANSPORT] tcgetattr failed on " << port_ << ": "
              << std::strerror(saved_errno) << std::endl;
    request_cp210x_recovery(saved_errno, port_);
    return false;
  }

  cfmakeraw(&tty);
  speed_t speed = baud_to_constant(baudrate_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  // Preserve modem-control state across close.  With CP210x on this AGV,
  // HUPCL can generate an unwanted DTR/RTS edge during rapid stop/start.
  tty.c_cflag &= ~HUPCL;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = static_cast<cc_t>(timeout_sec_ * 10);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const int saved_errno = errno;
    std::cerr << "[IMU-TRANSPORT] tcsetattr failed on " << port_ << ": "
              << std::strerror(saved_errno) << std::endl;
    request_cp210x_recovery(saved_errno, port_);
    return false;
  }
  if (tcflush(fd_, TCIOFLUSH) != 0 && errno != EINVAL) {
    const int saved_errno = errno;
    if (is_cp210x_io_fault(saved_errno)) {
      std::cerr << "[IMU-TRANSPORT] tcflush failed on " << port_ << ": "
                << std::strerror(saved_errno) << std::endl;
      request_cp210x_recovery(saved_errno, port_);
      return false;
    }
  }

  // TIOCEXCL: exclusive serial ownership. Prevents other processes (LiDAR node,
  // modem manager, etc.) from opening the same port. Kept until close().
  if (ioctl(fd_, TIOCEXCL) != 0) {
    const int saved_errno = errno;
    std::cout << "[IMU-TRANSPORT] exclusive-open pending on " << port_ << ": "
              << std::strerror(saved_errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    request_cp210x_recovery(saved_errno, port_);
    return false;
  }

  return true;
}

bool SerialPort::open()
{
  if (fd_ >= 0) return true;

  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    const int saved_errno = errno;
    std::cout << "[IMU-TRANSPORT] open pending on " << port_ << ": "
              << std::strerror(saved_errno) << std::endl;
    request_cp210x_recovery(saved_errno, port_);
    return false;
  }

  if (!configure_port()) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Set non-blocking
  int flags = fcntl(fd_, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);
  }

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
  return fd_ >= 0;
}

std::vector<uint8_t> SerialPort::read(size_t size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<uint8_t> buffer;
  buffer.reserve(size);

  if (fd_ < 0) return buffer;

  uint8_t tmp[256];
  while (buffer.size() < size) {
    ssize_t n = ::read(fd_, tmp, std::min(sizeof(tmp), size - buffer.size()));
    if (n > 0) {
      buffer.insert(buffer.end(), tmp, tmp + n);
    } else if (n == 0) {
      break;
    } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    } else {
      const int saved_errno = errno;
      std::cerr << "Read error on " << port_ << ": " << strerror(saved_errno) << std::endl;
      // A CP210x unplug/hub bounce can leave a stale descriptor behind. Mark
      // this SerialPort closed immediately so IMUDriver/WT901 reconnect through
      // the stable physical by-path instead of waiting on a dead fd.
      ::close(fd_);
      fd_ = -1;
      break;
    }
  }
  return buffer;
}

ssize_t SerialPort::write_bytes(const uint8_t * data, size_t size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) return -1;
  return ::write(fd_, data, size);
}

int SerialPort::available()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) return 0;
  int count = 0;
  if (ioctl(fd_, FIONREAD, &count) < 0) {
    // Treat EIO/ENODEV/etc. as a real disconnect so WT901 reconnects through
    // the stable by-path immediately instead of keeping a dead tty descriptor.
    ::close(fd_);
    fd_ = -1;
    return 0;
  }
  return count;
}

void SerialPort::flush()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    tcflush(fd_, TCIOFLUSH);
  }
}

bool SerialPort::path_exists(const std::string & path)
{
  return access(path.c_str(), F_OK) == 0;
}

bool SerialPort::is_port_accessible(const std::string & path)
{
  return access(path.c_str(), R_OK | W_OK) == 0;
}

}  // namespace imu_ros2