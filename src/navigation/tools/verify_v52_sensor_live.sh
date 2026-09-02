#!/usr/bin/env bash
# Run while mapping/allsystem is active. Exits non-zero if IMU or LiDAR is not publishing.
set -u
WS="${ROS_WS:-$HOME/ros}"
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WS/install/setup.bash" 2>/dev/null || true

pass=1
check_topic() {
  local topic="$1" label="$2"
  echo "[V52-CHECK] waiting for $label on $topic ..."
  if timeout 10s ros2 topic echo "$topic" --once >/tmp/v52_topic_check.txt 2>/tmp/v52_topic_check.err; then
    echo "[V52-CHECK][ON] $label is publishing: $topic"
  else
    echo "[V52-CHECK][OFF] no message from $topic within 10 s"
    sed -n '1,8p' /tmp/v52_topic_check.err >&2 || true
    pass=0
  fi
}

if [[ -x /usr/local/sbin/agv-sensor-recover ]]; then
  echo "[V52-CHECK] helper=$(sudo -n /usr/local/sbin/agv-sensor-recover --version 2>/dev/null || echo unavailable)"
fi
check_topic /imu/data IMU
check_topic /scan_nav LiDAR

if (( pass == 1 )); then
  echo "[V52-CHECK][PASS] IMU and LiDAR are both delivering ROS messages."
  exit 0
fi

echo "[V52-CHECK][FAIL] one or more sensor topics are not live." >&2
exit 3
