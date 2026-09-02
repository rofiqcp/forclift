#!/usr/bin/env bash
set -Eeuo pipefail

# V60 runtime launcher
# - One broken/optional subsystem must NOT block all three root commands.
# - Reuses the Jetson ROS overlay that was present during the successful build.
# - Checks only the executable/launch file required by the selected mode.
# - Keeps ROS logs in a deterministic workspace directory for diagnosis.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$SCRIPT_DIR" == */install/navigation/lib/navigation ]]; then
  DEFAULT_WS="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
else
  DEFAULT_WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
fi
WS="${AGV_WS:-$DEFAULT_WS}"
MODE="${1:-check}"
shift || true

info(){ printf '[V60-RUN] %s\n' "$*"; }
warn(){ printf '[V60-RUN] WARN: %s\n' "$*" >&2; }
fail(){ printf '[V60-RUN] FAIL: %s\n' "$*" >&2; exit 2; }

[[ -f /opt/ros/humble/setup.bash ]] || fail '/opt/ros/humble/setup.bash missing'
[[ -d "$WS" ]] || fail "workspace not found: $WS"
[[ -f "$WS/install/setup.bash" || -f "$WS/install/local_setup.bash" ]] || \
  fail "$WS/install setup is missing; build the workspace first"

# Source the same class of underlay used by the Jetson build. Do not erase the
# user's CUDA/display environment. The local workspace is sourced LAST so its
# navigation/yolo/esc packages have highest ament-index priority.
source /opt/ros/humble/setup.bash
JETSON_OVERLAY=""
for candidate in \
  "$HOME/jetson_install/ros_overlay/install/setup.bash" \
  "/home/otomasi2/jetson_install/ros_overlay/install/setup.bash"
do
  if [[ -f "$candidate" ]]; then
    JETSON_OVERLAY="$candidate"
    source "$candidate"
    break
  fi
done

if [[ -f "$WS/install/setup.bash" ]]; then
  source "$WS/install/setup.bash"
else
  source "$WS/install/local_setup.bash"
fi
# Re-source local_setup to force this workspace to the front even when a shell
# was previously polluted by an older overlay.
[[ -f "$WS/install/local_setup.bash" ]] && source "$WS/install/local_setup.bash"

export AGV_WS="$WS"
export AGV_RUNTIME_CONFIG_ROOT="${AGV_RUNTIME_CONFIG_ROOT:-$WS/config/runtime}"
export PYTHONUNBUFFERED=1
mkdir -p "$AGV_RUNTIME_CONFIG_ROOT" "$WS/maps" "$WS/log"

command -v ros2 >/dev/null 2>&1 || fail 'ros2 command not found after sourcing runtime environment'

nav_prefix="$(ros2 pkg prefix navigation 2>/dev/null || true)"
if [[ "$nav_prefix" != "$WS/install/navigation" ]]; then
  # Last deterministic recovery: put the isolated package prefixes explicitly
  # in front of AMENT_PREFIX_PATH. This does not touch LD_LIBRARY_PATH/CUDA.
  prepend=""
  for p in navigation yolo_obstacle_detection_ros2 esc; do
    [[ -d "$WS/install/$p" ]] && prepend="${prepend:+$prepend:}$WS/install/$p"
  done
  export AMENT_PREFIX_PATH="${prepend}${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  nav_prefix="$(ros2 pkg prefix navigation 2>/dev/null || true)"
fi
[[ "$nav_prefix" == "$WS/install/navigation" ]] || \
  fail "navigation resolves to '${nav_prefix:-<none>}' instead of '$WS/install/navigation'"

check_file(){ [[ -f "$1" ]] || fail "missing file: $1"; }
check_bin(){ [[ -x "$1" ]] || fail "missing/not executable: $1"; }
check_ldd(){
  local bin="$1" missing
  missing="$(ldd "$bin" 2>/dev/null | grep 'not found' || true)"
  if [[ -n "$missing" ]]; then
    printf '%s\n' "$missing" >&2
    fail "shared-library dependency missing for $bin"
  fi
}
optional_exe(){
  local pkg="$1" exe="$2"
  if ! ros2 pkg executables "$pkg" 2>/dev/null | awk '{print $2}' | grep -Fxq "$exe"; then
    warn "optional runtime executable not discovered: $pkg/$exe (selected command will still start)"
  fi
}

check_mode(){
  local mode="$1" launch="" bin=""
  case "$mode" in
    gui)
      launch="$WS/install/navigation/share/navigation/launch/gui.launch.py"
      bin="$WS/install/navigation/lib/navigation/agv_gui_launcher.sh"
      check_file "$WS/install/navigation/lib/navigation/navigation_gui/main_window.py"
      check_file "$WS/install/navigation/lib/navigation/navigation_gui/map_canvas.py"
      check_file "$WS/install/navigation/lib/navigation/navigation_gui/widgets.py"
      ;;
    map|mapping)
      launch="$WS/install/navigation/share/navigation/launch/map.launch.py"
      bin="$WS/install/navigation/lib/navigation/mapping_gui_cpp"
      ;;
    autonomous|auto)
      launch="$WS/install/navigation/share/navigation/launch/autonomous.launch.py"
      # autonomous.launch.py is staged/gated and has many children; do not make
      # optional camera/fork/winch discovery a root-launch blocker.
      ;;
    acceptance|acceptance-goal)
      check_file "$WS/install/navigation/lib/navigation/autonomous_runtime_check.py"
      ;;
    perception|perception-check)
      check_file "$WS/install/navigation/lib/navigation/perception_runtime_check.py"
      ;;
    perception-only)
      check_file "$WS/install/yolo_obstacle_detection_ros2/share/yolo_obstacle_detection_ros2/launch/perception_all.launch.py"
      ;;
    *) fail 'usage: run_v60.sh {gui|map|autonomous|check|acceptance|acceptance-goal|perception-check|perception-only} [ROS launch arguments...]' ;;
  esac
  if [[ -n "$launch" ]]; then
    check_file "$launch"
  fi
  if [[ -n "$bin" ]]; then
    check_bin "$bin"
    if [[ "$bin" != *.sh ]]; then
      check_ldd "$bin"
    fi
  fi
}

if [[ "$MODE" == "check" ]]; then
  check_mode gui
  check_mode map
  check_mode autonomous
  info "workspace: $WS"
  info "navigation prefix: $nav_prefix"
  [[ -n "$JETSON_OVERLAY" ]] && info "Jetson overlay: $JETSON_OVERLAY" || warn 'Jetson custom overlay not found; continuing with Humble + workspace'
  for req_pkg in rviz2 robot_state_publisher joint_state_publisher nav2_amcl nav2_planner nav2_smac_planner nav2_mppi_controller nav2_bt_navigator; do
    ros2 pkg prefix "$req_pkg" >/dev/null 2>&1 || fail "required runtime package not found: $req_pkg"
  done
  optional_exe yolo_obstacle_detection_ros2 obstacle_detector_node
  optional_exe yolo_obstacle_detection_ros2 astra_rgb_v4l2_node
  optional_exe yolo_obstacle_detection_ros2 hole_block_alignment_node.py
  optional_exe esc esc_driver
  optional_exe esc winch_serial_node
  [[ -x /usr/bin/python3 ]] || fail '/usr/bin/python3 missing for master PyQt GUI'
  env -u PYTHONHOME -u VIRTUAL_ENV PYTHONPATH= /usr/bin/python3 -c 'import sys, PyQt5; assert sys.version_info[:2] == (3,10)' || fail 'Python 3.10 / PyQt5 preflight failed'
  info 'PASS: GUI + independent perception + bounded RViz + resilient AMCL/Smac/MPPI runtime paths are runnable.'
  exit 0
fi

check_mode "$MODE"

case "$MODE" in
  acceptance)
    export AUTONOMOUS_CHECK_SECONDS="${AUTONOMOUS_CHECK_SECONDS:-15}"
    export AUTONOMOUS_CHECK_REQUIRE_PERCEPTION="${AUTONOMOUS_CHECK_REQUIRE_PERCEPTION:-1}"
    export AUTONOMOUS_CHECK_REQUIRE_MPPI_OUTPUT=0
    export AUTONOMOUS_CHECK_REQUIRE_RVIZ="${AUTONOMOUS_CHECK_REQUIRE_RVIZ:-1}"
    exec ros2 run navigation autonomous_runtime_check.py "$@"
    ;;
  acceptance-goal)
    export AUTONOMOUS_CHECK_SECONDS="${AUTONOMOUS_CHECK_SECONDS:-15}"
    export AUTONOMOUS_CHECK_REQUIRE_PERCEPTION="${AUTONOMOUS_CHECK_REQUIRE_PERCEPTION:-1}"
    export AUTONOMOUS_CHECK_REQUIRE_MPPI_OUTPUT=1
    export AUTONOMOUS_CHECK_REQUIRE_RVIZ="${AUTONOMOUS_CHECK_REQUIRE_RVIZ:-1}"
    exec ros2 run navigation autonomous_runtime_check.py "$@"
    ;;
  perception|perception-check)
    export PERCEPTION_CHECK_SECONDS="${PERCEPTION_CHECK_SECONDS:-12}"
    exec ros2 run navigation perception_runtime_check.py "$@"
    ;;
  perception-only)
    exec ros2 launch yolo_obstacle_detection_ros2 perception_all.launch.py "$@"
    ;;
esac

case "$MODE" in
  gui|map|mapping)
    if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
      fail 'no graphical display detected (DISPLAY/WAYLAND_DISPLAY empty); run from the Jetson desktop session'
    fi
    ;;
  autonomous|auto)
    rviz_disabled=0
    for arg in "$@"; do
      [[ "$arg" == "enable_rviz:=false" || "$arg" == "enable_rviz:=0" ]] && rviz_disabled=1
    done
    if [[ "$rviz_disabled" -eq 0 && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
      fail 'RViz requested but no graphical display detected (DISPLAY/WAYLAND_DISPLAY empty). Run from the Jetson desktop session or pass enable_rviz:=false.'
    fi
    ;;
esac

STAMP="$(date +%Y%m%d_%H%M%S)"
ROS_LOG_DIR="$WS/log/runtime_v60_${MODE}_${STAMP}"
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR
info "mode=$MODE"
info "ROS log dir: $ROS_LOG_DIR"
[[ -n "$JETSON_OVERLAY" ]] && info "using Jetson overlay: $JETSON_OVERLAY"

# Optional packages are diagnostics only. A missing fork/camera/winch executable
# must be reflected by GUI telemetry, not by preventing the whole GUI/map stack
# from opening.
optional_exe yolo_obstacle_detection_ros2 hole_block_alignment_node.py
optional_exe esc winch_serial_node

case "$MODE" in
  gui)
    exec ros2 launch navigation gui.launch.py "$@"
    ;;
  map|mapping)
    exec ros2 launch navigation map.launch.py "$@"
    ;;
  autonomous|auto)
    exec ros2 launch navigation autonomous.launch.py "$@"
    ;;
esac
