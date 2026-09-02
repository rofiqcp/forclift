#include "serial/serial.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace serial
{
namespace
{
std::string errnoMessage(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}
}  // namespace

Serial::~Serial()
{
  std::lock_guard<std::mutex> lock(mutex_);
  closeUnlocked();
}

void Serial::setPort(const std::string & port)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    throw SerialException("cannot change port while serial device is open");
  }
  port_ = port;
}

void Serial::setBaudrate(uint32_t baudrate)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    throw SerialException("cannot change baudrate while serial device is open");
  }
  baudrate_ = baudrate;
}

void Serial::setTimeout(const Timeout & timeout)
{
  std::lock_guard<std::mutex> lock(mutex_);
  timeout_ = timeout;
}

unsigned int Serial::baudToTermios(uint32_t baudrate)
{
  switch (baudrate) {
    case 9600U: return B9600;
    case 19200U: return B19200;
    case 38400U: return B38400;
    case 57600U: return B57600;
    case 115200U: return B115200;
#ifdef B230400
    case 230400U: return B230400;
#endif
#ifdef B460800
    case 460800U: return B460800;
#endif
#ifdef B921600
    case 921600U: return B921600;
#endif
    default:
      throw SerialException("unsupported serial baudrate: " + std::to_string(baudrate));
  }
}

void Serial::open()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    return;
  }
  if (port_.empty()) {
    throw SerialException("serial port is empty");
  }

  const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    throw IOException(errnoMessage("failed to open " + port_));
  }

  // Keep one physical serial device owned by exactly one sensor node. This
  // prevents LiDAR and IMU auto-detection from simultaneously consuming the
  // same USB serial stream. The lock is released automatically on close().
  if (ioctl(fd, TIOCEXCL) != 0) {
    const std::string message = errnoMessage("TIOCEXCL failed for " + port_);
    ::close(fd);
    throw IOException(message);
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    const std::string message = errnoMessage("tcgetattr failed for " + port_);
    ::close(fd);
    throw IOException(message);
  }

  cfmakeraw(&tty);
  const speed_t speed = static_cast<speed_t>(baudToTermios(baudrate_));
  if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
    const std::string message = errnoMessage("failed to set baudrate on " + port_);
    ::close(fd);
    throw IOException(message);
  }

  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
#ifdef CRTSCTS
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    const std::string message = errnoMessage("tcsetattr failed for " + port_);
    ::close(fd);
    throw IOException(message);
  }

  if (tcflush(fd, TCIFLUSH) != 0 && errno != EINVAL) {
    const std::string message = errnoMessage("tcflush failed for " + port_);
    ::close(fd);
    throw IOException(message);
  }

  fd_ = fd;
}

void Serial::closeUnlocked() noexcept
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void Serial::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  closeUnlocked();
}

bool Serial::isOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0;
}

size_t Serial::available()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    throw SerialException("serial device is not open");
  }

  int bytes = 0;
  if (ioctl(fd_, FIONREAD, &bytes) != 0) {
    throw IOException(errnoMessage("FIONREAD failed on " + port_));
  }
  return bytes > 0 ? static_cast<size_t>(bytes) : 0U;
}

std::string Serial::read(size_t size)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    throw SerialException("serial device is not open");
  }
  if (size == 0U) {
    return {};
  }

  std::string output;
  output.resize(size);
  size_t total = 0U;
  const auto timeout = std::chrono::milliseconds(timeout_.read_timeout_constant);
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (total < size) {
    const ssize_t count = ::read(fd_, output.data() + total, size - total);
    if (count > 0) {
      total += static_cast<size_t>(count);
      continue;
    }
    if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    throw IOException(errnoMessage("read failed on " + port_));
  }

  output.resize(total);
  return output;
}

size_t Serial::write(const std::string & data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    throw SerialException("serial device is not open");
  }
  if (data.empty()) {
    return 0U;
  }

  size_t total = 0U;
  while (total < data.size()) {
    const ssize_t count = ::write(fd_, data.data() + total, data.size() - total);
    if (count > 0) {
      total += static_cast<size_t>(count);
      continue;
    }
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    throw IOException(errnoMessage("write failed on " + port_));
  }

  return total;
}

void Serial::flushInput()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    throw SerialException("serial device is not open");
  }
  if (tcflush(fd_, TCIFLUSH) != 0) {
    throw IOException(errnoMessage("tcflush failed on " + port_));
  }
}

}  // namespace serial
