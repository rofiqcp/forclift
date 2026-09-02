#pragma once

#include "esc/protocol.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace esc {

/*
 * Satu objek SerialBoard mewakili satu STM32 ESC.
 * Koneksi dirancang hot-plug: port yang hilang ditutup, lalu dicoba dibuka lagi
 * secara periodik tanpa mematikan node ROS 2.
 */
class SerialBoard {
 public:
  explicit SerialBoard(std::string port = "", int baud = 115200);
  ~SerialBoard();

  void set_port(const std::string &port) { port_ = port; }
  bool poll();
  bool send(CommandFrame frame);
  void close_port();

  bool connected() const { return fd_ >= 0; }
  bool has_basic() const { return have_basic_; }
  const TelemetryBasic &basic() const { return basic_; }
  std::uint8_t status() const { return status_; }
  std::uint8_t error_left() const { return error_left_; }
  std::uint8_t error_right() const { return error_right_; }
  std::uint64_t connection_generation() const { return generation_; }
  std::uint64_t basic_generation() const { return basic_generation_; }
  std::chrono::steady_clock::time_point last_basic_feedback_time() const { return last_basic_feedback_; }

 private:
  bool open_port();
  void feed(const std::uint8_t *data, std::size_t length);
  void process_frame(const FeedbackFrame &frame);
  bool feedback_timed_out() const;

  std::string port_;
  int baud_;
  int fd_{-1};
  std::vector<std::uint8_t> rx_;
  TelemetryBasic basic_{};
  bool have_basic_{false};
  bool have_any_feedback_{false};
  std::uint8_t status_{0};
  std::uint8_t error_left_{0};
  std::uint8_t error_right_{0};
  std::uint16_t sequence_{0};
  std::uint64_t generation_{0};
  std::uint64_t basic_generation_{0};
  std::chrono::steady_clock::time_point last_open_try_{};
  std::chrono::steady_clock::time_point opened_at_{};
  std::chrono::steady_clock::time_point last_feedback_{};
  std::chrono::steady_clock::time_point last_basic_feedback_{};
};

}  // namespace esc
