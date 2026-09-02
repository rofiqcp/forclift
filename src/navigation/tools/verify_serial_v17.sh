#!/usr/bin/env bash
# Runtime smoke test for V17. Run while mapping/autonomous is active.
set -u

imu_alias=/tmp/agv_devices/imu
lidar_alias=/tmp/agv_devices/lidar

imu_real=$(readlink -f "$imu_alias" 2>/dev/null || true)
lidar_real=$(readlink -f "$lidar_alias" 2>/dev/null || true)

if [[ -z "$imu_real" || -z "$lidar_real" || "$imu_real" == "$lidar_real" ]]; then
  echo "[V17-VERIFY] FAIL topology imu=${imu_real:-WAIT} lidar=${lidar_real:-WAIT}" >&2
  exit 2
fi

echo "[V17-VERIFY] topology PASS imu=$imu_real lidar=$lidar_real"

if ! timeout 6 ros2 topic echo /imu/data --once >/dev/null 2>&1; then
  echo "[V17-VERIFY] FAIL /imu/data has no message" >&2
  exit 3
fi
if ! timeout 6 ros2 topic echo /scan --once >/dev/null 2>&1; then
  echo "[V17-VERIFY] FAIL /scan has no message" >&2
  exit 4
fi

echo "[V17-VERIFY] PASS /imu/data + /scan are publishing"
exit 0
