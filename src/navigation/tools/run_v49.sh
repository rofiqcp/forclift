#!/usr/bin/env bash
set -euo pipefail

# V48 deterministic launcher. It deliberately ignores a previously sourced
# workspace overlay and reconstructs the environment from Humble + this install.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$SCRIPT_DIR" == */install/navigation/lib/navigation ]]; then
  DEFAULT_WS="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
else
  DEFAULT_WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
fi
WS="${AGV_WS:-$DEFAULT_WS}"
MODE="${1:-gui}"

fail(){ echo "[V49-RUN] FAIL: $*" >&2; exit 2; }
[[ -f /opt/ros/humble/setup.bash ]] || fail '/opt/ros/humble/setup.bash missing'
[[ -f "$WS/install/local_setup.bash" ]] || fail "$WS/install/local_setup.bash missing; run bash src/BUILD_V48.sh first"

# Remove only ROS/workspace state that can poison package discovery. Keep CUDA,
# DISPLAY, WAYLAND, device permissions and the user's ordinary environment.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION || true
# Drop stale copies of this workspace from library search paths without removing CUDA.
prune_path_var(){
  local name="$1" value="${!1-}" out="" part
  IFS=':' read -ra parts <<< "$value"
  for part in "${parts[@]}"; do
    [[ -z "$part" ]] && continue
    case "$part" in
      "$WS/install"|"$WS/install/"*|"$WS/build"|"$WS/build/"*) continue ;;
    esac
    out="${out:+$out:}$part"
  done
  export "$name=$out"
}
prune_path_var LD_LIBRARY_PATH
prune_path_var PATH

source /opt/ros/humble/setup.bash
# local_setup is intentional: setup.bash may chain parent prefixes captured by
# an old dirty build. The underlay is already sourced explicitly above.
source "$WS/install/local_setup.bash"
export AGV_WS="$WS"
export AGV_RUNTIME_CONFIG_ROOT="${AGV_RUNTIME_CONFIG_ROOT:-$WS/config/runtime}"
mkdir -p "$AGV_RUNTIME_CONFIG_ROOT" "$WS/maps" "$WS/log"

command -v ros2 >/dev/null 2>&1 || fail 'ros2 command not found after environment reconstruction'
nav_prefix="$(ros2 pkg prefix navigation 2>/dev/null || true)"
[[ "$nav_prefix" == "$WS/install/navigation" ]] || fail "navigation resolves to '$nav_prefix' instead of '$WS/install/navigation'"

check_exe(){
  local pkg="$1" exe="$2"
  ros2 pkg executables "$pkg" 2>/dev/null | awk '{print $2}' | grep -Fxq "$exe" || fail "missing executable $pkg/$exe"
}
check_launch(){
  local pkg="$1" launch="$2"
  local prefix
  prefix="$(ros2 pkg prefix "$pkg" 2>/dev/null || true)"
  [[ -n "$prefix" && -f "$prefix/share/$pkg/launch/$launch" ]] || fail "missing launch $pkg/$launch"
}

check_exe navigation agv_gui_cpp
check_exe navigation mapping_gui_cpp
check_exe navigation goal_pose_nav2_bridge
check_exe navigation scan_self_filter
# The GUI now consumes the real YOLO/fork custom messages and should fail fast
# if a stale workspace omitted these runtime packages.
check_exe yolo_obstacle_detection_ros2 obstacle_detector_node
check_exe yolo_obstacle_detection_ros2 astra_rgb_v4l2_node
check_exe yolo_obstacle_detection_ros2 hole_block_alignment_node.py
check_exe esc esc_driver
check_exe esc winch_serial_node
check_launch navigation gui.launch.py
check_launch navigation map.launch.py
check_launch navigation autonomous.launch.py

# Catch unresolved ELF dependencies before ros2 launch reports a vague process death.
for exe in agv_gui_cpp mapping_gui_cpp goal_pose_nav2_bridge scan_self_filter; do
  bin="$WS/install/navigation/lib/navigation/$exe"
  [[ -x "$bin" ]] || fail "installed binary is missing/not executable: $bin"
  if ldd "$bin" 2>/dev/null | grep -q 'not found'; then
    ldd "$bin" | grep 'not found' >&2 || true
    fail "shared-library dependency missing for $exe"
  fi
done

case "$MODE" in
  gui)
    exec ros2 launch navigation gui.launch.py "${@:2}"
    ;;
  map|mapping)
    exec ros2 launch navigation map.launch.py "${@:2}"
    ;;
  autonomous|auto)
    exec ros2 launch navigation autonomous.launch.py "${@:2}"
    ;;
  *)
    fail 'usage: run_v49.sh {gui|map|autonomous} [ROS launch arguments...]'
    ;;
esac
