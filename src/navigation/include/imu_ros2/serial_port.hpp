#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include <termios.h>
#include <unistd.h>

namespace imu_ros2
{

class SerialPort
{
public:
  SerialPort() = default;
  SerialPort(std::string port, int baudrate, double timeout_sec = 0.1);
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;
  SerialPort(SerialPort &&) = delete;
  SerialPort & operator=(SerialPort &&) = delete;

  bool open();
  void close();
  bool is_open() const;
  std::vector<uint8_t> read(size_t size);
  ssize_t write_bytes(const uint8_t * data, size_t size);
  int available();
  void flush();
  static speed_t baud_to_constant(int baudrate);

  const std::string & port() const noexcept { return port_; }
  int baudrate() const noexcept { return baudrate_; }

  static bool path_exists(const std::string & path);
  static bool is_port_accessible(const std::string & path);

private:
  bool configure_port();

  std::string port_;
  int baudrate_{921600};
  double timeout_sec_{0.1};
  int fd_{-1};
  mutable std::mutex mutex_;
};

}  // namespace imu_ros2
