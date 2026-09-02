#!/usr/bin/env bash
set -Eeuo pipefail
WS="${AGV_WS:-$HOME/ros}"
SRC="$WS/src"
fail(){ echo "[V58-BUILD] FAIL: $*" >&2; exit 2; }
info(){ echo "[V58-BUILD] $*"; }
[[ -f /opt/ros/humble/setup.bash ]] || fail '/opt/ros/humble/setup.bash missing'
[[ -f "$SRC/navigation/package.xml" ]] || fail "navigation source missing under $SRC"
[[ -f "$SRC/yolo_obstacle_detection_ros2/package.xml" ]] || fail 'yolo source missing'
[[ -f "$SRC/esc/package.xml" ]] || fail 'esc source missing'

chmod +x \
  "$SRC/RUN_GUI.sh" "$SRC/RUN_MAPPING.sh" "$SRC/RUN_AUTONOMOUS.sh" "$SRC/RUN_CHECK.sh" \
  "$SRC/RUN_ACCEPTANCE_V58.sh" "$SRC/RUN_GOAL_ACCEPTANCE_V58.sh" \
  "$SRC/navigation/scripts/agv_gui.py" "$SRC/navigation/scripts/agv_gui_launcher.sh" \
  "$SRC/navigation/scripts/autonomous_runtime_check.py" \
  "$SRC/navigation/tools/rviz_robot_model_ready_gate.py" \
  "$SRC/navigation/tools/run_v58.sh" "$SRC/navigation/tools/verify_v58_complete_runtime.sh" || true

info 'Building V58 complete runtime fix: decoupled AMCL/perception + costmap/inflation + Smac/MPPI GUI'
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/jetson_install/ros_overlay/install/setup.bash" ]]; then
  source "$HOME/jetson_install/ros_overlay/install/setup.bash"
elif [[ -f /home/otomasi2/jetson_install/ros_overlay/install/setup.bash ]]; then
  source /home/otomasi2/jetson_install/ros_overlay/install/setup.bash
fi

if ! env -u PYTHONHOME -u VIRTUAL_ENV PYTHONPATH= /usr/bin/python3 -c 'import sys, PyQt5; assert sys.version_info[:2] == (3,10)' >/dev/null 2>&1; then
  fail 'Ubuntu Python 3.10 + PyQt5 not available'
fi

# Validate source BEFORE the long colcon build.
export AGV_WS="$WS"
bash "$SRC/navigation/tools/verify_v58_complete_runtime.sh"

cd "$WS"
rm -rf \
  build/navigation install/navigation \
  build/yolo_obstacle_detection_ros2 install/yolo_obstacle_detection_ros2 \
  build/esc install/esc
colcon build --symlink-install \
  --packages-select yolo_obstacle_detection_ros2 esc navigation \
  --event-handlers console_direct+

source "$WS/install/setup.bash"
[[ -f "$WS/install/local_setup.bash" ]] && source "$WS/install/local_setup.bash"

# Verify both source and installed-root command resolution.
bash "$SRC/navigation/tools/verify_v58_complete_runtime.sh"
bash "$SRC/navigation/tools/run_v58.sh" check

info 'PASS: V58 built and root launch paths validated.'
echo 'Autonomous:          bash src/RUN_AUTONOMOUS.sh'
echo 'GUI:                 bash src/RUN_GUI.sh'
echo 'Static/live check:   bash src/RUN_CHECK.sh'
echo 'Live acceptance:     bash src/RUN_ACCEPTANCE_V58.sh'
echo 'After setting Goal:  bash src/RUN_GOAL_ACCEPTANCE_V58.sh'
