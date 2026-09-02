#include "imu_ros2/imu_parser.hpp"

#include <algorithm>
#include <cmath>

namespace imu_ros2
{

IMUParser::IMUParser(double accel_range, double gyro_range)
: accel_range_(accel_range), gyro_range_(gyro_range)
{
}

int16_t IMUParser::le_i16(const uint8_t * p)
{
  return static_cast<int16_t>(p[0] | (p[1] << 8));
}

std::optional<PacketType> IMUParser::parse_packet(const std::array<uint8_t, kPacketSize> & packet)
{
  if (packet[0] != kSyncByte) {
    return std::nullopt;
  }

  const uint8_t pkg_type = packet[1];
  const uint8_t * payload = packet.data() + 2;
  const uint8_t checksum = packet[10];

  // Verify checksum
  uint8_t expected = (kSyncByte + pkg_type);
  for (size_t i = 0; i < kPayloadSize; ++i) {
    expected = (expected + payload[i]) & 0xFF;
  }
  if (expected != checksum) {
    return std::nullopt;
  }

  switch (pkg_type) {
    case 0x51: return PacketType::Accel;
    case 0x52: return PacketType::Gyro;
    case 0x53: return PacketType::Angle;
    case 0x54: return PacketType::Mag;
    default:   return std::nullopt;
  }
}

AccelData IMUParser::parse_accel(const uint8_t * payload, double accel_range) const
{
  const int16_t ax = le_i16(payload);
  const int16_t ay = le_i16(payload + 2);
  const int16_t az = le_i16(payload + 4);
  const int16_t temp = le_i16(payload + 6);

  const double range = (accel_range > 0.0) ? accel_range : accel_range_;
  const double scale = range * 9.80665 / 32768.0;

  AccelData data;
  data.x = ax * scale;
  data.y = ay * scale;
  data.z = az * scale;
  data.temperature = temp / 100.0;
  return data;
}

GyroData IMUParser::parse_gyro(const uint8_t * payload, double gyro_range) const
{
  const int16_t gx = le_i16(payload);
  const int16_t gy = le_i16(payload + 2);
  const int16_t gz = le_i16(payload + 4);
  const int16_t temp = le_i16(payload + 6);

  const double range = (gyro_range > 0.0) ? gyro_range : gyro_range_;
  const double scale = range * M_PI / 180.0 / 32768.0;

  GyroData data;
  data.x = gx * scale;
  data.y = gy * scale;
  data.z = gz * scale;
  data.temperature = temp / 100.0;
  return data;
}

AngleData IMUParser::parse_angle(const uint8_t * payload) const
{
  const int16_t roll = le_i16(payload);
  const int16_t pitch = le_i16(payload + 2);
  const int16_t yaw = le_i16(payload + 4);

  const double scale = M_PI / 32768.0;

  AngleData data;
  data.roll  = roll  * scale;
  data.pitch = pitch * scale;
  data.yaw   = yaw   * scale;
  return data;
}

MagData IMUParser::parse_mag(const uint8_t * payload) const
{
  const int16_t mx = le_i16(payload);
  const int16_t my = le_i16(payload + 2);
  const int16_t mz = le_i16(payload + 4);
  const int16_t temp = le_i16(payload + 6);

  // 4800 uT full scale
  const double scale = 4800.0 * 1e-6 / 32768.0;

  MagData data;
  data.x = mx * scale;
  data.y = my * scale;
  data.z = mz * scale;
  data.temperature = temp / 100.0;
  return data;
}

std::array<double, 4> IMUParser::euler_to_quaternion(double roll, double pitch, double yaw)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  return {
    sr * cp * cy - cr * sp * sy,  // x
    cr * sp * cy + sr * cp * sy,  // y
    cr * cp * sy - sr * sp * cy,  // z
    cr * cp * cy + sr * sp * sy   // w
  };
}

std::string IMUParser::packet_type_name(PacketType type)
{
  switch (type) {
    case PacketType::Accel: return "accel";
    case PacketType::Gyro:  return "gyro";
    case PacketType::Angle: return "angle";
    case PacketType::Mag:   return "mag";
  }
  return "unknown";
}

void IMUFilter::apply(const Vector3 & accel, const Vector3 & gyro, const Vector3 & mag,
                      Vector3 & accel_cal, Vector3 & gyro_cal, Vector3 & mag_cal) const
{
  accel_cal.x = accel.x - accel_bias_.x;
  accel_cal.y = accel.y - accel_bias_.y;
  accel_cal.z = accel.z - accel_bias_.z;

  gyro_cal.x = gyro.x - gyro_bias_.x;
  gyro_cal.y = gyro.y - gyro_bias_.y;
  gyro_cal.z = gyro.z - gyro_bias_.z;

  const double mx = mag.x - mag_bias_.x;
  const double my = mag.y - mag_bias_.y;
  const double mz = mag.z - mag_bias_.z;

  const auto & s = mag_soft_iron_;
  mag_cal.x = s[0] * mx + s[1] * my + s[2] * mz;
  mag_cal.y = s[3] * mx + s[4] * my + s[5] * mz;
  mag_cal.z = s[6] * mx + s[7] * my + s[8] * mz;
}

}  // namespace imu_ros2