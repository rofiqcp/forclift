#!/usr/bin/env bash
set -euo pipefail
NAV="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
SRC="$(cd "$NAV/.." && pwd)"
PASS=0; FAIL=0
ok(){ echo "[V47] PASS $*"; PASS=$((PASS+1)); }
bad(){ echo "[V47] FAIL $*" >&2; FAIL=$((FAIL+1)); }
need_file(){ [[ -f "$1" ]] && ok "exists ${1#$SRC/}" || bad "missing ${1#$SRC/}"; }
need_grep(){ grep -Eq -- "$2" "$1" && ok "$3" || bad "$3"; }
forbid_grep(){ if grep -Eq -- "$2" "$1"; then bad "$3"; else ok "$3"; fi; }

CMAKE="$NAV/CMakeLists.txt"
PKG="$NAV/package.xml"
need_file "$CMAKE"
need_file "$PKG"
for f in agv_gui.cpp mapping_gui.cpp goal_pose_nav2_bridge.cpp scan_self_filter.cpp runtime_validator.cpp; do need_file "$NAV/src/$f"; done
for f in gui.launch.py map.launch.py autonomous.launch.py allsystem.launch.py imu.launch.py lidar.launch.py; do need_file "$NAV/launch/$f"; done
for f in esc.launch.py winch.launch.py; do need_file "$SRC/esc/launch/$f"; done
for f in perception_all.launch.py camera.launch.py; do need_file "$SRC/yolo_obstacle_detection_ros2/launch/$f"; done

# Requested production runtime must be native C++.
need_grep "$CMAKE" 'add_executable\(agv_gui_cpp[[:space:]]+src/agv_gui.cpp\)' 'agv_gui_cpp target installed from C++'
need_grep "$CMAKE" 'add_executable\(mapping_gui_cpp[[:space:]]+src/mapping_gui.cpp\)' 'mapping_gui_cpp target installed from C++'
need_grep "$CMAKE" 'add_executable\(goal_pose_nav2_bridge[[:space:]]+src/goal_pose_nav2_bridge.cpp\)' 'GoalPose bridge target installed from C++'
need_grep "$CMAKE" 'add_executable\(scan_self_filter[[:space:]]+src/scan_self_filter.cpp\)' 'scan filter target installed from C++'
forbid_grep "$CMAKE" 'scripts/(agv_gui|mapping_gui|goal_pose_nav2_bridge|scan_self_filter)\.py' 'obsolete production Python entry points are not installed'
[[ ! -e "$NAV/scripts/agv_gui.py" ]] && ok 'agv_gui.py removed from production source' || bad 'agv_gui.py still present'
[[ ! -e "$NAV/scripts/mapping_gui.py" ]] && ok 'mapping_gui.py removed from production source' || bad 'mapping_gui.py still present'
[[ ! -e "$NAV/scripts/goal_pose_nav2_bridge.py" ]] && ok 'goal_pose_nav2_bridge.py removed from production source' || bad 'goal_pose_nav2_bridge.py still present'
[[ ! -e "$NAV/scripts/scan_self_filter.py" ]] && ok 'scan_self_filter.py removed from production source' || bad 'scan_self_filter.py still present'
[[ ! -d "$NAV/python/navigation_gui" ]] && ok 'old Python GUI package removed' || bad 'old Python GUI package still present'

# GUI visual identity / restored legacy layout guard.
GUI="$NAV/src/agv_gui.cpp"
need_grep "$GUI" 'Autonomous Vehicle Interface — Sekolah Vokasi UNDIP' 'legacy GUI window title preserved'
need_grep "$GUI" 'background:#20252b' 'legacy dark GUI palette preserved'
need_grep "$GUI" 'Connection & System Overview' 'legacy connection overview preserved'
need_grep "$GUI" 'Map Comparison' 'legacy Map Comparison page preserved'
need_grep "$GUI" 'Smac Hybrid-A\*' 'legacy Smac Hybrid-A* page preserved'
need_grep "$GUI" 'Fork Alignment' 'legacy Fork Alignment card preserved'
need_grep "$GUI" 'QTabWidget' 'legacy Topics/TF tab structure present'

# Launch -> executable wiring.
need_grep "$NAV/launch/gui.launch.py" "executable='agv_gui_cpp'" 'gui.launch uses native C++ GUI'
need_grep "$NAV/launch/map.launch.py" "executable='mapping_gui_cpp'" 'map.launch uses native C++ mapping GUI'
need_grep "$NAV/launch/autonomous.launch.py" 'executable="goal_pose_nav2_bridge"' 'autonomous.launch uses native C++ GoalPose bridge'
need_grep "$NAV/launch/lidar.launch.py" "executable='scan_self_filter'" 'lidar.launch uses native C++ scan filter'
forbid_grep "$NAV/launch/autonomous.launch.py" 'goal_pose_nav2_bridge\.py' 'autonomous.launch has no Python GoalPose bridge'
forbid_grep "$NAV/launch/lidar.launch.py" 'scan_self_filter\.py' 'lidar.launch has no Python scan filter'

# Core C++ executables expected by all launch modes.
for exe in imu_node lidar_node hector_slam_node imu_visual_tf_node map_monitor_node allsystem_gate goal_pose_nav2_bridge scan_self_filter navigation_runtime_validator agv_gui_cpp mapping_gui_cpp; do
  need_grep "$CMAKE" "add_executable\\($exe([[:space:]]|$)" "CMake target: $exe"
done

# Cross-package expected executables.
ESC_CMAKE="$SRC/esc/CMakeLists.txt"
for exe in esc_driver esc_command_mux esc_keyboard_teleop winch_serial_node ackermann_controller_server; do
  need_grep "$ESC_CMAKE" "add_executable\\($exe([[:space:]]|$)" "ESC target: $exe"
done
YOLO_CMAKE="$SRC/yolo_obstacle_detection_ros2/CMakeLists.txt"
for exe in astra_rgb_v4l2_node obstacle_detector_node; do
  need_grep "$YOLO_CMAKE" "add_executable\\($exe([[:space:]]|$)" "YOLO target: $exe"
done
need_grep "$YOLO_CMAKE" 'hole_block_alignment_node\.py' 'hole alignment compatibility node is installed'

# Syntax checks that are available without ROS being installed.
if command -v python3 >/dev/null 2>&1; then
  if python3 -m py_compile "$NAV"/launch/*.launch.py "$SRC/esc"/launch/*.launch.py "$SRC/yolo_obstacle_detection_ros2"/launch/*.launch.py; then
    ok 'all ROS 2 Python launch descriptions parse as Python'
  else
    bad 'Python launch-description syntax error'
  fi
fi
for f in "$NAV"/tools/*.sh "$NAV"/scripts/*.sh "$SRC/esc"/tools/*.sh "$SRC/yolo_obstacle_detection_ros2"/tools/*.sh; do
  [[ -f "$f" ]] || continue
  if bash -n "$f"; then :; else bad "shell syntax: ${f#$SRC/}"; fi
done
ok 'shell syntax scan completed'

printf '[V47] STATIC RESULT: %d PASS / %d FAIL\n' "$PASS" "$FAIL"
(( FAIL == 0 ))
