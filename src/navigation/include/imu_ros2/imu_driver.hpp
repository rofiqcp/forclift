#pragma once

#include "imu_ros2/serial_port.hpp"
#include "imu_ros2/imu_parser.hpp"
#include <functional>
#include <memory>
#include <string>

namespace imu_ros2
{

class IMUDriver
{
public:
  IMUDriver(std::string port, int baudrate = 921600, double timeout_sec = 0.1);
  ~IMUDriver();

  bool connect();
  void disconnect();
  bool is_connected() const { return connected_.load(); }

  std::vector<uint8_t> read_raw(size_t size);
  int in_waiting() const;

  const std::string & port() const noexcept { return port_; }
  int baudrate() const noexcept { return baudrate_; }

  static std::optional<std::string> detect_imu_port(
      const std::vector<int> & baudrates = {921600, 115200, 230400, 460800, 57600, 38400, 19200, 9600});
  static bool is_port_in_use(const std::string & port);

private:
  std::string port_;
  int baudrate_;
  double timeout_sec_;
  std::unique_ptr<SerialPort> serial_;
  std::atomic<bool> connected_{false};
};

}  // namespace imu_ros2