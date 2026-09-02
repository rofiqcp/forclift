#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace esc {

constexpr std::uint16_t kStart = 0xABCD;
constexpr std::uint8_t kVersion = 4;
constexpr std::size_t kFrameSize = 64;

/* Jenis pesan harus sama dengan firmware/Src/esc_protocol.h. */
enum Message : std::uint8_t {
  HELLO = 1,
  CONTROL = 2,
  PID_CONFIG = 3,
  TELEMETRY_CONFIG = 4,
  SAVE_EEPROM = 5,
  LOAD_EEPROM = 6,
  ZERO_POSITION = 7,
  DISARM = 8,
  REQUEST_CONFIG = 9,
  TELEMETRY = 0x80,
};

enum Motor : std::uint8_t { LEFT = 0, RIGHT = 1, BOTH = 2 };
enum Mode : std::uint8_t { OPEN = 0, VLT = 1, SPD = 2, TRQ = 3, POS = 4 };

constexpr std::uint8_t FLAG_ARM = 1U << 0;
constexpr std::uint8_t STATUS_LINK_OK = 1U << 0;
constexpr std::uint8_t STATUS_ARMED = 1U << 1;
constexpr std::uint8_t STATUS_EEPROM_OK = 1U << 2;
constexpr std::uint8_t STATUS_TIMEOUT = 1U << 3;
constexpr std::uint8_t STATUS_EEPROM_VERIFIED = 1U << 4;
constexpr std::uint8_t STATUS_SETTINGS_DIRTY = 1U << 5;

#pragma pack(push, 1)
struct CommandFrame {
  std::uint16_t start;
  std::uint8_t version;
  std::uint8_t type;
  std::uint16_t sequence;
  std::uint8_t flags;
  std::uint8_t motor;
  std::uint8_t mode_left;
  std::uint8_t mode_right;
  std::uint8_t loop;
  std::uint8_t telemetry_page;
  std::uint16_t telemetry_rate_hz;
  std::int32_t setpoint_left;
  std::int32_t setpoint_right;
  std::int32_t kp_q16;
  std::int32_t ki_q16;
  std::int32_t kd_q16;
  std::int32_t i_limit_q16;
  std::int32_t output_min;
  std::int32_t output_max;
  std::int32_t position_min;
  std::int32_t position_max;
  std::uint32_t telemetry_mask;
  std::uint16_t reserved0;
  std::uint16_t reserved1;
  std::uint16_t checksum;
};

struct FeedbackFrame {
  std::uint16_t start;
  std::uint8_t version;
  std::uint8_t type;
  std::uint16_t sequence;
  std::uint8_t page;
  std::uint8_t status;
  std::uint32_t uptime_ms;
  std::uint8_t mode_left;
  std::uint8_t mode_right;
  std::uint8_t error_left;
  std::uint8_t error_right;
  std::uint8_t payload[46];
  std::uint16_t checksum;
};

struct TelemetryBasic {
  std::int32_t position_left;
  std::int32_t position_right;
  std::int32_t setpoint_left;
  std::int32_t setpoint_right;
  std::int16_t speed_left;
  std::int16_t speed_right;
  std::int16_t battery_centi_volt;
  std::int16_t temperature_deci_c;
  std::int16_t dc_current_left_centi_amp;
  std::int16_t dc_current_right_centi_amp;
  std::int16_t command_left;
  std::int16_t command_right;
  std::uint16_t link_age_ms;
  std::uint16_t telemetry_rate_hz;
};
#pragma pack(pop)

static_assert(sizeof(CommandFrame) == kFrameSize, "CommandFrame harus 64 byte");
static_assert(sizeof(FeedbackFrame) == kFrameSize, "FeedbackFrame harus 64 byte");

/* Menghitung CRC16-CCITT-FALSE yang sama dengan STM32 dan GUI. */
inline std::uint16_t crc16(const std::uint8_t *data, std::size_t length) {
  std::uint16_t crc = 0xFFFF;
  for (std::size_t index = 0; index < length; ++index) {
    crc ^= static_cast<std::uint16_t>(data[index]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U)
                ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<std::uint16_t>(crc << 1U);
    }
  }
  return crc;
}

/* Membuat command dengan default aman. Sequence akan diisi SerialBoard::send(). */
inline CommandFrame command(std::uint8_t type) {
  CommandFrame frame{};
  frame.start = kStart;
  frame.version = kVersion;
  frame.type = type;
  frame.telemetry_rate_hz = 50;
  frame.output_min = -1000;
  frame.output_max = 1000;
  frame.position_min = -200000;
  frame.position_max = 200000;
  frame.telemetry_mask = 0xFFFFFFFFU;
  frame.reserved1 = 2U;  // default position deadband, sama dengan GUI/firmware
  return frame;
}

/* Mengisi CRC command setelah seluruh field selesai diubah. */
inline void finalize(CommandFrame &frame) {
  frame.checksum = crc16(reinterpret_cast<const std::uint8_t *>(&frame), 62);
}

/* Memastikan feedback benar-benar berasal dari protokol ESC versi yang sama. */
inline bool valid(const FeedbackFrame &frame) {
  return frame.start == kStart && frame.version == kVersion &&
         frame.type == TELEMETRY &&
         frame.checksum == crc16(reinterpret_cast<const std::uint8_t *>(&frame), 62);
}

}  // namespace esc
