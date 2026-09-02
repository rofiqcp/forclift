#include "imu_ros2/imu_driver.hpp"

#include <algorithm>
#include <chrono>
#include <dirent.h>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace imu_ros2
{

IMUDriver::IMUDriver(std::string port, int baudrate, double timeout_sec)
: port_(std::move(port)), baudrate_(baudrate), timeout_sec_(timeout_sec)
{
}

IMUDriver::~IMUDriver()
{
  disconnect();
}

bool IMUDriver::connect()
{
  serial_ = std::make_unique<SerialPort>(port_, baudrate_, timeout_sec_);
  if (serial_->open()) {
    connected_.store(true);
    return true;
  }
  connected_.store(false);
  return false;
}

void IMUDriver::disconnect()
{
  if (serial_) {
    serial_->close();
    serial_.reset();
  }
  connected_.store(false);
}

std::vector<uint8_t> IMUDriver::read_raw(size_t size)
{
  if (!connected_.load() || !serial_) {
    return {};
  }
  auto data = serial_->read(size);
  if (data.empty() && !serial_->is_open()) {
    connected_.store(false);
  }
  return data;
}

int IMUDriver::in_waiting() const
{
  if (!connected_.load() || !serial_) {
    return 0;
  }
  return serial_->available();
}

std::optional<std::string> IMUDriver::detect_imu_port(
    const std::vector<int> & baudrates)
{
  std::vector<std::string> candidates;

  // 1. Priority: /dev/serial/by-id/* (stable symlinks)
  DIR * dir = opendir("/dev/serial/by-id");
  if (dir) {
    struct dirent * ent;
    while ((ent = readdir(dir)) != nullptr) {
      if (ent->d_name[0] != '.') {
        candidates.push_back("/dev/serial/by-id/" + std::string(ent->d_name));
      }
    }
    closedir(dir);
  }

  // 2. Fallback: /dev/ttyUSB* /dev/ttyACM*
  for (int i = 0; i < 8; ++i) {
    candidates.emplace_back("/dev/ttyUSB" + std::to_string(i));
  }
  for (int i = 0; i < 4; ++i) {
    candidates.emplace_back("/dev/ttyACM" + std::to_string(i));
  }

  // 3. Scan /dev for any ttyUSB/ttyACM
  dir = opendir("/dev");
  if (dir) {
    struct dirent * ent;
    while ((ent = readdir(dir)) != nullptr) {
      std::string name = ent->d_name;
      if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0) {
        std::string path = "/dev/" + name;
        if (std::find(candidates.begin(), candidates.end(), path) == candidates.end()) {
          candidates.push_back(std::move(path));
        }
      }
    }
    closedir(dir);
  }

  for (const auto & port : candidates) {
    if (!SerialPort::path_exists(port)) continue;
    if (!SerialPort::is_port_accessible(port)) continue;
    if (is_port_in_use(port)) continue;

    for (int br : baudrates) {
      SerialPort test_port(port, br, 1.0);
      if (!test_port.open()) continue;

      std::vector<uint8_t> rx_buf;
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      for (int tries = 0; tries < 20; ++tries) {
        auto chunk = test_port.read(64);
        rx_buf.insert(rx_buf.end(), chunk.begin(), chunk.end());
        if (rx_buf.size() >= 1000) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      test_port.close();

      if (rx_buf.size() < 11) continue;

      // Validate real WT901 frames: 0x55 + type(51/52/53/54) + 8 data + checksum
      int valid_count = 0;
      int i = 0;
      while (i <= static_cast<int>(rx_buf.size()) - 11) {
        if (rx_buf[i] == 0x55) {
          uint8_t type = rx_buf[i + 1];
          if (type == 0x51 || type == 0x52 || type == 0x53 || type == 0x54) {
            uint8_t expected = (0x55 + type);
            for (int j = 0; j < 8; ++j) {
              expected = (expected + rx_buf[i + 2 + j]) & 0xFF;
            }
            if (expected == rx_buf[i + 10]) {
              valid_count++;
              i += 11;
              if (valid_count >= 3) {
                return port;
              }
              continue;
            }
          }
        }
        ++i;
      }
    }
  }
  return std::nullopt;
}

bool IMUDriver::is_port_in_use(const std::string & port)
{
  int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) return false;
  struct flock fl{};
  fl.l_type = F_WRLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start = 0;
  fl.l_len = 0;
  if (fcntl(fd, F_SETLK, &fl) == 0) {
    fl.l_type = F_UNLCK;
    fcntl(fd, F_SETLK, &fl);
    ::close(fd);
    return false;
  }
  ::close(fd);
  return true;
}

}  // namespace imu_ros2
