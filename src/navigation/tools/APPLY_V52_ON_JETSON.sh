#!/usr/bin/env bash
# One-time V52 deployment for LiDAR + IMU CP210x EIO recovery and LiDAR auto parser.
set -euo pipefail

WS="${ROS_WS:-$HOME/ros}"
NAV_SRC="${NAV_SRC:-$WS/src/navigation}"
LOGIN_USER="${SUDO_USER:-${USER:-otomasi2}}"
HELPER="/usr/local/sbin/agv-sensor-recover"
EXPECTED="AGV-SERIAL-RECOVERY-V52"

say() { printf '\n[V52] %s\n' "$*"; }
fail() { echo "[V52][ERROR] $*" >&2; exit 1; }

[[ -d "$NAV_SRC" ]] || fail "navigation source not found: $NAV_SRC"
[[ -f "$NAV_SRC/tools/install_sensor_recovery.sh" ]] || fail "install_sensor_recovery.sh missing"
[[ -f "$NAV_SRC/tools/cp210x_recover.py" ]] || fail "cp210x_recover.py missing"
[[ -f /opt/ros/humble/setup.bash ]] || fail "ROS 2 Humble not found"

say "Stopping stale IMU/LiDAR owners before touching USB transports"
pkill -TERM -f '/navigation/lib/navigation/(imu_node|lidar_node)' 2>/dev/null || true
sleep 1
pkill -KILL -f '/navigation/lib/navigation/(imu_node|lidar_node)' 2>/dev/null || true
rm -f /tmp/agv_devices/imu /tmp/agv_devices/lidar

say "Installing V52 recovery helper + udev policy (sudo may ask once)"
sudo /usr/bin/env AGV_TARGET_USER="$LOGIN_USER" \
  bash "$NAV_SRC/tools/install_sensor_recovery.sh" --install-only

version="$(sudo -n "$HELPER" --version 2>/dev/null || true)"
[[ "$version" == "$EXPECTED" ]] || fail "installed helper version is '$version', expected '$EXPECTED'"
echo "[V52] helper=$version"

say "Repairing and validating both CP210x transports"
if ! sudo -n "$HELPER" both; then
  echo "[V52][ERROR] CP210x recovery could not obtain two I/O-healthy endpoints." >&2
  echo "[V52][ERROR] Check USB power/cable/hub if this persists." >&2
  exit 2
fi
udevadm settle --timeout=10 >/dev/null 2>&1 || true

say "Current serial inventory"
ls -l /dev/ttyUSB* 2>/dev/null || true
ls -l /dev/serial/by-path/* 2>/dev/null || true

say "Clean-building navigation"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
rm -rf "$WS/build/navigation" "$WS/install/navigation"
cd "$WS"
colcon build --symlink-install --packages-select navigation --event-handlers console_direct+

say "V52 installed successfully"
echo "Run: source /opt/ros/humble/setup.bash && source $WS/install/setup.bash"
echo "Then start mapping with your normal command (or: ros2 launch navigation mapping_runtime.launch.py)"
echo "Verify while mapping is running: $NAV_SRC/tools/verify_v52_sensor_live.sh"
