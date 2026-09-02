#!/usr/bin/env bash
set -euo pipefail

WS="${AGV_WS:-$HOME/ros}"
SRC="$WS/src"
fail(){ echo "[V54-BUILD] FAIL: $*" >&2; exit 2; }

[[ -f /opt/ros/humble/setup.bash ]] || fail '/opt/ros/humble/setup.bash missing'
[[ -f "$SRC/navigation/package.xml" ]] || fail "navigation source missing under $SRC"
[[ -f "$SRC/yolo_obstacle_detection_ros2/package.xml" ]] || fail 'yolo_obstacle_detection_ros2 source missing'
[[ -f "$SRC/esc/package.xml" ]] || fail 'esc source missing'

echo '[V54-BUILD] Full GUI telemetry + fork startup fix'
source /opt/ros/humble/setup.bash
cd "$WS"

# Rebuild all three directly involved packages so the GUI cannot link against a
# stale custom-message install and the current fork heartbeat/node is installed.
rm -rf \
  build/navigation install/navigation \
  build/yolo_obstacle_detection_ros2 install/yolo_obstacle_detection_ros2 \
  build/esc install/esc

colcon build --symlink-install \
  --packages-select yolo_obstacle_detection_ros2 esc navigation \
  --event-handlers console_direct+

source "$WS/install/local_setup.bash"
bash "$SRC/navigation/tools/verify_v54_full_gui_telemetry.sh"

check_exe(){
  local pkg="$1" exe="$2"
  ros2 pkg executables "$pkg" | awk '{print $2}' | grep -Fxq "$exe" || fail "missing $pkg/$exe"
}
check_exe navigation agv_gui_cpp
check_exe yolo_obstacle_detection_ros2 obstacle_detector_node
check_exe yolo_obstacle_detection_ros2 astra_rgb_v4l2_node
check_exe yolo_obstacle_detection_ros2 hole_block_alignment_node.py
check_exe esc esc_driver
check_exe esc winch_serial_node

echo '[V54-BUILD] PASS'
echo 'Run GUI with: bash src/RUN_GUI.sh'
