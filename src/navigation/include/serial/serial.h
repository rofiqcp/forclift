#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>

namespace serial
{

class IOException : public std::runtime_error
{
public:
  explicit IOException(const std::string & message) : std::runtime_error(message) {}
};

class SerialException : public std::runtime_error
{
public:
  explicit SerialException(const std::string & message) : std::runtime_error(message) {}
};

struct Timeout
{
  uint32_t read_timeout_constant{20};

  static Timeout simpleTimeout(uint32_t timeout_ms)
  {
    Timeout timeout;
    timeout.read_timeout_constant = timeout_ms;
    return timeout;
  }
};

class Serial
{
public:
  Serial() = default;
  ~Serial();

  Serial(const Serial &) = delete;
  Serial & operator=(const Serial &) = delete;

  void setPort(const std::string & port);
  void setBaudrate(uint32_t baudrate);
  void setTimeout(const Timeout & timeout);

  void open();
  void close();
  bool isOpen() const;

  size_t available();
  std::string read(size_t size);
  size_t write(const std::string & data);
  void flushInput();

private:
  static unsigned int baudToTermios(uint32_t baudrate);
  void closeUnlocked() noexcept;

  mutable std::mutex mutex_;
  std::string port_;
  uint32_t baudrate_{115200};
  Timeout timeout_{};
  int fd_{-1};
};

}  // namespace serial
