#!/usr/bin/env bash
set -euo pipefail
WS="${AGV_WS:-/home/otomasi2/forclift}"
NAV="$WS/src/navigation"
fail(){ echo "[V47] FAIL: $*" >&2; exit 2; }
[[ -d "$NAV" ]] || fail "navigation source not found at $NAV"
[[ -f /opt/ros/humble/setup.bash ]] || fail '/opt/ros/humble/setup.bash not found'

# Remove stale references to this workspace before deleting build/install.
prune_var(){
  local name="$1" value="${!1-}" out="" part
  IFS=':' read -ra parts <<< "$value"
  for part in "${parts[@]}"; do
    [[ -z "$part" ]] && continue
    case "$part" in
      "$WS/install"|"$WS/install/"*) continue ;;
      "$WS/build"|"$WS/build/"*) continue ;;
    esac
    out="${out:+$out:}$part"
  done
  export "$name=$out"
}
for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PATH; do prune_var "$var"; done
export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin${PATH:+:$PATH}"
source /opt/ros/humble/setup.bash

# Native Qt/C++ GUI dependency.
if ! pkg-config --exists Qt5Widgets 2>/dev/null; then
  echo '[V47] Installing qtbase5-dev...'
  sudo apt-get update
  sudo apt-get install -y qtbase5-dev
fi

echo '[V47] 1/7 source verification'
bash "$NAV/tools/verify_v44_humble_build_fix.sh" "$NAV"
bash "$NAV/tools/verify_v45_warning_free_build.sh" "$NAV"
bash "$NAV/tools/verify_v46_gui_sensor_startup.sh" "$NAV"
bash "$NAV/tools/verify_v47_commands.sh" "$NAV"

echo '[V47] 2/7 clean stale build/install/log'
rm -rf "$WS/build" "$WS/install" "$WS/log"
mkdir -p "$WS/log"
cd "$WS"

echo '[V47] 3/7 dependency check'
if command -v rosdep >/dev/null 2>&1; then
  rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
fi

echo '[V47] 4/7 clean build'
colcon build --symlink-install --event-handlers console_direct+
source "$WS/install/setup.bash"

echo '[V47] 5/7 executable discovery'
check_exe(){
  local pkg="$1" exe="$2"
  ros2 pkg executables "$pkg" | awk '{print $2}' | grep -Fxq "$exe" || fail "missing executable: $pkg/$exe"
  echo "[V47] PASS executable $pkg/$exe"
}
for exe in agv_gui_cpp mapping_gui_cpp goal_pose_nav2_bridge scan_self_filter navigation_runtime_validator imu_node lidar_node hector_slam_node imu_visual_tf_node map_monitor_node allsystem_gate; do check_exe navigation "$exe"; done
for exe in esc_driver esc_command_mux esc_keyboard_teleop winch_serial_node ackermann_controller_server; do check_exe esc "$exe"; done
for exe in astra_rgb_v4l2_node obstacle_detector_node hole_block_alignment_node.py; do check_exe yolo_obstacle_detection_ros2 "$exe"; done

echo '[V47] 6/7 launch discovery/parse'
for spec in \
  'navigation gui.launch.py' \
  'navigation map.launch.py' \
  'navigation autonomous.launch.py' \
  'navigation allsystem.launch.py' \
  'navigation imu.launch.py' \
  'navigation lidar.launch.py' \
  'esc esc.launch.py' \
  'esc winch.launch.py' \
  'yolo_obstacle_detection_ros2 perception_all.launch.py' \
  'yolo_obstacle_detection_ros2 camera.launch.py'; do
  set -- $spec
  ros2 launch "$1" "$2" --show-args >/dev/null || fail "launch parse failed: $1 $2"
  echo "[V47] PASS launch $1/$2"
done

echo '[V47] 7/7 warning/error scan'
# Do not treat words embedded in documentation filenames as warnings/errors.
if grep -RniE --include='stderr.log' '(^|[^[:alpha:]])(warning:|error:|CMake Warning|FAILED)([^[:alpha:]]|$)' "$WS/log"; then
  fail 'build stderr contains warning/error text'
fi
if [[ -f "$WS/log/latest_build/logger_all.log" ]] && grep -nE ' (WARNING|ERROR) ' "$WS/log/latest_build/logger_all.log"; then
  fail 'colcon logger contains WARNING/ERROR'
fi

echo '[V47] COMPLETE — build, executable discovery and launch parsing passed.'
echo '[V47] Run GUI:        bash src/navigation/tools/run_v47.sh gui'
echo '[V47] Run Mapping:    bash src/navigation/tools/run_v47.sh map'
echo '[V47] Run Autonomous: bash src/navigation/tools/run_v47.sh autonomous'
