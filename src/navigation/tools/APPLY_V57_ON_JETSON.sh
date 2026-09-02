#!/usr/bin/env bash
set -Eeuo pipefail

WS="${AGV_WS:-$HOME/ros}"
SRC="$WS/src"
fail(){ echo "[V57-BUILD] FAIL: $*" >&2; exit 2; }
info(){ echo "[V57-BUILD] $*"; }

[[ -f /opt/ros/humble/setup.bash ]] || fail '/opt/ros/humble/setup.bash missing'
[[ -f "$SRC/navigation/package.xml" ]] || fail "navigation source missing under $SRC"
[[ -f "$SRC/yolo_obstacle_detection_ros2/package.xml" ]] || fail 'yolo_obstacle_detection_ros2 source missing'
[[ -f "$SRC/esc/package.xml" ]] || fail 'esc source missing'

chmod +x \
  "$SRC/RUN_GUI.sh" "$SRC/RUN_MAPPING.sh" "$SRC/RUN_AUTONOMOUS.sh" "$SRC/RUN_CHECK.sh" \
  "$SRC/navigation/scripts/agv_gui.py" "$SRC/navigation/scripts/agv_gui_launcher.sh" \
  "$SRC/navigation/tools/rviz_robot_model_ready_gate.py" \
  "$SRC/navigation/tools/run_v57.sh" \
  "$SRC/navigation/tools/verify_v56_master_gui_robot_model.sh" \
  "$SRC/navigation/tools/verify_v57_gui_goal_amcl_camera.sh" || true

info 'V57 GUI goal-isolation + AMCL graph + camera preview/data fix build'
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/jetson_install/ros_overlay/install/setup.bash" ]]; then
  source "$HOME/jetson_install/ros_overlay/install/setup.bash"
elif [[ -f /home/otomasi2/jetson_install/ros_overlay/install/setup.bash ]]; then
  source /home/otomasi2/jetson_install/ros_overlay/install/setup.bash
fi

# PyQt is runtime-only and does not participate in CMake compilation, but report
# it early so GUI startup never fails with a vague import error later.
if ! env -u PYTHONHOME -u VIRTUAL_ENV PYTHONPATH= /usr/bin/python3 -c 'import sys, PyQt5; assert sys.version_info[:2] == (3,10)' >/dev/null 2>&1; then
  fail 'Ubuntu Python 3.10 + PyQt5 not available. Install python3-pyqt5 before RUN_GUI.'
fi

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

bash "$SRC/navigation/tools/verify_v54_full_gui_telemetry.sh"
bash "$SRC/navigation/tools/verify_v56_master_gui_robot_model.sh"
bash "$SRC/navigation/tools/verify_v57_gui_goal_amcl_camera.sh"
bash "$SRC/navigation/tools/run_v57.sh" check

info 'PASS'
echo 'Run GUI:        bash src/RUN_GUI.sh'
echo 'Run GUI only:   bash src/RUN_GUI.sh start_autonomous:=false'
echo 'Run Mapping:    bash src/RUN_MAPPING.sh'
echo 'Run Autonomous: bash src/RUN_AUTONOMOUS.sh'
echo 'Check only:     bash src/RUN_CHECK.sh'
