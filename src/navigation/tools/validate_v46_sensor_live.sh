#!/usr/bin/env bash
# V46 live publication verifier.
# Usage:
#   validate_v46_sensor_live.sh sensor       # physical IMU/LiDAR streams
#   validate_v46_sensor_live.sh localization # + lidar odom, EKF, map
#   validate_v46_sensor_live.sh navigation   # + AMCL/costmaps/interlocks (after 2D Pose Estimate)
#   validate_v46_sensor_live.sh goal         # + non-empty /smac_plan (after Nav2 Goal)
set -euo pipefail
MODE="${1:-sensor}"
WS="${AGV_WS:-/home/otomasi2/forclift}"
[[ -f /opt/ros/humble/setup.bash ]] || { echo '[V46-LIVE] FAIL /opt/ros/humble/setup.bash missing' >&2; exit 2; }
[[ -f "$WS/install/setup.bash" ]] || { echo "[V46-LIVE] FAIL $WS/install/setup.bash missing" >&2; exit 2; }
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
fail(){ echo "[V46-LIVE] FAIL $*" >&2; exit 2; }
pass(){ echo "[V46-LIVE] PASS $*"; }
check_topic(){
  local topic="$1" seconds="${2:-20}" durability="${3:-volatile}"
  echo "[V46-LIVE] waiting real message: $topic"
  local cmd=(ros2 topic echo "$topic" --once)
  if [[ "$durability" == "transient_local" ]]; then
    cmd+=(--qos-durability transient_local --qos-reliability reliable)
  fi
  if timeout "${seconds}s" "${cmd[@]}" >/dev/null 2>&1; then
    pass "$topic"
  else
    fail "no message from $topic within ${seconds}s"
  fi
}
check_nonempty_path(){
  local tmp
  tmp="$(mktemp)"
  trap 'rm -f "$tmp"' RETURN
  echo '[V46-LIVE] waiting non-empty Hybrid-A* path: /smac_plan'
  if ! timeout 30s ros2 topic echo /smac_plan --once --qos-durability transient_local --qos-reliability reliable >"$tmp" 2>/dev/null; then
    rm -f "$tmp"; trap - RETURN
    fail 'no /smac_plan message within 30s; set 2D Pose Estimate then Nav2 Goal in free space'
  fi
  if grep -Eq '^poses:[[:space:]]*\[\][[:space:]]*$' "$tmp" || ! grep -q '^poses:' "$tmp"; then
    rm -f "$tmp"; trap - RETURN
    fail '/smac_plan arrived but contains no path poses'
  fi
  rm -f "$tmp"; trap - RETURN
  pass '/smac_plan non-empty'
}

case "$MODE" in
  sensor|localization|navigation|goal) ;;
  *) fail "unknown mode '$MODE' (sensor|localization|navigation|goal)" ;;
esac

[[ -e /tmp/agv_devices/imu ]] || fail '/tmp/agv_devices/imu alias missing'
[[ -e /tmp/agv_devices/lidar ]] || fail '/tmp/agv_devices/lidar alias missing'
pass "IMU alias -> $(readlink -f /tmp/agv_devices/imu)"
pass "LiDAR alias -> $(readlink -f /tmp/agv_devices/lidar)"

# Physical sensor publications used by the GUI / localization stack.
check_topic /imu/data 25
check_topic /imu/gyro 12
check_topic /imu/accel 12
check_topic /imu/mag 12
check_topic /imu/status 12
check_topic /scan_nav 30
check_topic /scan_safety 20
check_topic /lidar/safety_healthy 15
check_topic /lidar/status 15

if [[ "$MODE" == "sensor" ]]; then
  echo '[V46-LIVE] PASS: all primary IMU/LiDAR GUI streams publish real messages.'
  exit 0
fi

# Derived localization publications. /map is latched/transient-local.
check_topic /lidar/odom 25
check_topic /odometry/filtered 25
check_topic /map 20 transient_local

if [[ "$MODE" == "localization" ]]; then
  echo '[V46-LIVE] PASS: sensors + LiDAR odometry + EKF + map are publishing.'
  exit 0
fi

# Run this only after 2D Pose Estimate has been given and AMCL has converged.
check_topic /amcl_pose 30
check_topic /global_costmap/costmap 30 transient_local
check_topic /local_costmap/costmap 30 transient_local
check_topic /navigation/planner_status 20
check_topic /system/autonomy_motion_allowed 20
check_topic /system/manual_motion_allowed 20
check_topic /esc/ready 20

if [[ "$MODE" == "navigation" ]]; then
  echo '[V46-LIVE] PASS: navigation foundation topics are publishing after localization.'
  exit 0
fi

# Goal-dependent publication must contain actual path poses.
check_nonempty_path
echo '[V46-LIVE] PASS: all checked runtime streams + non-empty Hybrid-A* path are publishing.'
