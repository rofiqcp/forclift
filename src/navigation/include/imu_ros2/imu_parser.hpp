#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace imu_ros2
{

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct AccelData : public Vector3
{
  double temperature{25.0};
};

struct GyroData : public Vector3
{
  double temperature{25.0};
};

struct AngleData
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

struct MagData : public Vector3
{
  double temperature{25.0};
};

enum class PacketType
{
  Accel,
  Gyro,
  Angle,
  Mag
};

class IMUParser
{
public:
  static constexpr uint8_t kSyncByte = 0x55;
  static constexpr size_t kPacketSize = 11;
  static constexpr size_t kPayloadSize = 8;

  IMUParser(double accel_range = 16.0, double gyro_range = 2000.0);

  std::optional<PacketType> parse_packet(const std::array<uint8_t, kPacketSize> & packet);
  AccelData parse_accel(const uint8_t * payload, double accel_range = -1.0) const;
  GyroData parse_gyro(const uint8_t * payload, double gyro_range = -1.0) const;
  AngleData parse_angle(const uint8_t * payload) const;
  MagData parse_mag(const uint8_t * payload) const;

  static std::array<double, 4> euler_to_quaternion(double roll, double pitch, double yaw);
  static std::string packet_type_name(PacketType type);

private:
  static int16_t le_i16(const uint8_t * p);

  double accel_range_{16.0};
  double gyro_range_{2000.0};
};

class IMUFilter
{
public:
  void set_accel_bias(Vector3 bias) { accel_bias_ = bias; }
  void set_gyro_bias(Vector3 bias) { gyro_bias_ = bias; }
  void set_mag_bias(Vector3 bias) { mag_bias_ = bias; }
  void set_mag_soft_iron(std::array<double, 9> matrix) { mag_soft_iron_ = matrix; }

  void apply(const Vector3 & accel, const Vector3 & gyro, const Vector3 & mag,
             Vector3 & accel_cal, Vector3 & gyro_cal, Vector3 & mag_cal) const;

private:
  Vector3 accel_bias_{};
  Vector3 gyro_bias_{};
  Vector3 mag_bias_{};
  std::array<double, 9> mag_soft_iron_{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
};

}  // namespace imu_ros2
