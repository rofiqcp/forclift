#!/usr/bin/env bash
set -Eeuo pipefail

WS="${AGV_WS:-$HOME/ros}"
SRC="$WS/src"
fail(){ echo "[V55-BUILD] FAIL: $*" >&2; exit 2; }
info(){ echo "[V55-BUILD] $*"; }

[[ -f /opt/ros/humble/setup.bash ]] || fail '/opt/ros/humble/setup.bash missing'
[[ -f "$SRC/navigation/package.xml" ]] || fail "navigation source missing under $SRC"
[[ -f "$SRC/yolo_obstacle_detection_ros2/package.xml" ]] || fail 'yolo_obstacle_detection_ros2 source missing'
[[ -f "$SRC/esc/package.xml" ]] || fail 'esc source missing'

chmod +x \
  "$SRC/RUN_GUI.sh" "$SRC/RUN_MAPPING.sh" "$SRC/RUN_AUTONOMOUS.sh" "$SRC/RUN_CHECK.sh" \
  "$SRC/navigation/tools/run_v55.sh" \
  "$SRC/navigation/tools/run_gui" "$SRC/navigation/tools/run_mapping" "$SRC/navigation/tools/run_autonomous" \
  "$SRC/navigation/tools/verify_v55_three_commands.sh" || true

info 'V55 three-command runtime recovery build'
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/jetson_install/ros_overlay/install/setup.bash" ]]; then
  source "$HOME/jetson_install/ros_overlay/install/setup.bash"
elif [[ -f /home/otomasi2/jetson_install/ros_overlay/install/setup.bash ]]; then
  source /home/otomasi2/jetson_install/ros_overlay/install/setup.bash
fi
cd "$WS"

# Rebuild the three directly coupled packages. The log supplied for V54 showed
# these packages compile/install successfully; V55 changes the runtime layer and
# keeps the telemetry/fork fixes intact.
rm -rf \
  build/navigation install/navigation \
  build/yolo_obstacle_detection_ros2 install/yolo_obstacle_detection_ros2 \
  build/esc install/esc

colcon build --symlink-install \
  --packages-select yolo_obstacle_detection_ros2 esc navigation \
  --event-handlers console_direct+

# Runtime must use the same overlay chain as the build, then the current
# workspace. setup.bash is preferred because it preserves the recorded underlay.
source "$WS/install/setup.bash"
[[ -f "$WS/install/local_setup.bash" ]] && source "$WS/install/local_setup.bash"

bash "$SRC/navigation/tools/verify_v54_full_gui_telemetry.sh"
bash "$SRC/navigation/tools/verify_v55_three_commands.sh" "$SRC"
bash "$SRC/navigation/tools/run_v55.sh" check

info 'PASS'
echo 'Run GUI:        bash src/RUN_GUI.sh'
echo 'Run Mapping:    bash src/RUN_MAPPING.sh'
echo 'Run Autonomous: bash src/RUN_AUTONOMOUS.sh'
echo 'Check only:     bash src/RUN_CHECK.sh'
