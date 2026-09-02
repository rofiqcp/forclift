#include "esc/serial_board.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <utility>

namespace esc {

/* Menyimpan path serial. Port baru benar-benar dibuka ketika poll() dipanggil. */
SerialBoard::SerialBoard(std::string port, int baud)
    : port_(std::move(port)), baud_(baud) {
  rx_.reserve(512);
}

/* Menjamin file descriptor ditutup ketika node dihentikan. */
SerialBoard::~SerialBoard() { close_port(); }

/* Menutup port dan membuang telemetry lama agar reconnect tidak memakai data stale. */
void SerialBoard::close_port() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
  fd_ = -1;
  have_basic_ = false;
  have_any_feedback_ = false;
  status_ = 0;
  error_left_ = 0;
  error_right_ = 0;
  rx_.clear();
}

/* Membuka USART host 115200 8N1 dan memulai handshake HELLO dalam kondisi DISARM. */
bool SerialBoard::open_port() {
  const auto now = std::chrono::steady_clock::now();
  if (now - last_open_try_ < std::chrono::seconds(1)) {
    return false;
  }
  last_open_try_ = now;

  if (port_.empty()) {
    return false;
  }

  const int new_fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (new_fd < 0) {
    return false;
  }

  // Keep the ESC UART exclusively owned by this driver. This prevents a
  // second process from consuming protocol bytes and corrupting telemetry.
  if (ioctl(new_fd, TIOCEXCL) != 0) {
    ::close(new_fd);
    return false;
  }

  termios settings{};
  if (tcgetattr(new_fd, &settings) != 0) {
    ::close(new_fd);
    return false;
  }

  cfmakeraw(&settings);
  (void)baud_;  // Firmware ini sengaja dikunci ke USART3 115200.
  cfsetispeed(&settings, B115200);
  cfsetospeed(&settings, B115200);
  settings.c_cflag |= CLOCAL | CREAD;
  settings.c_cflag &= ~CRTSCTS;

  if (tcsetattr(new_fd, TCSANOW, &settings) != 0) {
    ::close(new_fd);
    return false;
  }

  tcflush(new_fd, TCIOFLUSH);
  fd_ = new_fd;
  ++generation_;
  opened_at_ = now;
  last_feedback_ = now;
  rx_.clear();
  have_basic_ = false;
  have_any_feedback_ = false;

  auto hello = command(HELLO);
  (void)send(hello);

  auto telemetry = command(TELEMETRY_CONFIG);
  telemetry.telemetry_page = 0;  // BASIC
  telemetry.telemetry_rate_hz = 50;
  telemetry.telemetry_mask = 0xFFFFFFFFU;
  (void)send(telemetry);
  return true;
}

/* Menganggap link serial mati bila satu detik tidak ada frame CRC-valid. */
bool SerialBoard::feedback_timed_out() const {
  if (fd_ < 0) {
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (!have_any_feedback_) {
    return now - opened_at_ > std::chrono::seconds(1);
  }
  return now - last_feedback_ > std::chrono::seconds(1);
}

/*
 * Membaca seluruh byte yang tersedia. Error EIO/ENODEV atau feedback timeout
 * dianggap hot-unplug dan port akan dibuka ulang oleh poll berikutnya.
 */
bool SerialBoard::poll() {
  if (fd_ < 0) {
    (void)open_port();
    return fd_ >= 0;
  }

  std::uint8_t buffer[256];
  for (;;) {
    const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
    if (count > 0) {
      feed(buffer, static_cast<std::size_t>(count));
      continue;
    }
    if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    close_port();
    return false;
  }

  if (feedback_timed_out()) {
    close_port();
    return false;
  }
  return true;
}

/* Mengirim tepat satu frame 64 byte; write error akan memicu state reconnect. */
bool SerialBoard::send(CommandFrame frame) {
  if (fd_ < 0) {
    return false;
  }

  frame.sequence = ++sequence_;
  finalize(frame);
  const auto *data = reinterpret_cast<const std::uint8_t *>(&frame);
  std::size_t remaining = sizeof(frame);

  while (remaining > 0) {
    const ssize_t count = ::write(fd_, data, remaining);
    if (count > 0) {
      data += count;
      remaining -= static_cast<std::size_t>(count);
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      /* Frame parsial tidak boleh dilanjutkan pada siklus berikutnya. Tutup agar parser sinkron ulang. */
      close_port();
      return false;
    }
    close_port();
    return false;
  }
  return true;
}

/* Parser stream mencari CD AB dan CRC sehingga tahan terhadap byte sampah setelah reconnect. */
void SerialBoard::feed(const std::uint8_t *data, std::size_t length) {
  rx_.insert(rx_.end(), data, data + length);
  const std::uint8_t start_low = static_cast<std::uint8_t>(kStart);
  const std::uint8_t start_high = static_cast<std::uint8_t>(kStart >> 8U);

  while (rx_.size() >= 2) {
    std::size_t start_index = rx_.size();
    for (std::size_t index = 0; index + 1 < rx_.size(); ++index) {
      if (rx_[index] == start_low && rx_[index + 1] == start_high) {
        start_index = index;
        break;
      }
    }

    if (start_index == rx_.size()) {
      const std::uint8_t last = rx_.back();
      rx_.clear();
      if (last == start_low) {
        rx_.push_back(last);
      }
      break;
    }

    if (start_index > 0) {
      rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(start_index));
    }
    if (rx_.size() < kFrameSize) {
      break;
    }

    FeedbackFrame frame{};
    std::memcpy(&frame, rx_.data(), sizeof(frame));
    if (valid(frame)) {
      process_frame(frame);
      rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(kFrameSize));
    } else {
      rx_.erase(rx_.begin());
    }
  }
}

/* Menyimpan header/status dan payload BASIC terbaru untuk driver kendaraan. */
void SerialBoard::process_frame(const FeedbackFrame &frame) {
  status_ = frame.status;
  error_left_ = frame.error_left;
  error_right_ = frame.error_right;
  have_any_feedback_ = true;
  last_feedback_ = std::chrono::steady_clock::now();

  if (frame.page == 0) {
    std::memcpy(&basic_, frame.payload, sizeof(basic_));
    have_basic_ = true;
    last_basic_feedback_ = last_feedback_;
    ++basic_generation_;
  }
}

}  // namespace esc
