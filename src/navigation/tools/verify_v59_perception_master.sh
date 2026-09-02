#!/usr/bin/env bash
set -Eeuo pipefail
SRC="${AGV_WS:-$HOME/ros}/src"
NAV="$SRC/navigation"
YOLO="$SRC/yolo_obstacle_detection_ros2"
fail(){ echo "[V59-VERIFY] FAIL: $*" >&2; exit 2; }
pass(){ echo "[V59-VERIFY] PASS: $*"; }
req(){ [[ -f "$1" ]] || fail "missing $1"; }

LAUNCH="$NAV/launch/autonomous.launch.py"
CAMLAUNCH="$YOLO/launch/camera.launch.py"
PALL="$YOLO/launch/perception_all.launch.py"
BRIDGE="$NAV/python/navigation_gui/ros_bridge.py"
MAIN="$NAV/python/navigation_gui/main_window.py"
CHECK="$NAV/scripts/perception_runtime_check.py"
AUTOCHECK="$NAV/scripts/autonomous_runtime_check.py"
for f in "$LAUNCH" "$CAMLAUNCH" "$PALL" "$BRIDGE" "$MAIN" "$CHECK" "$AUTOCHECK"; do req "$f"; done

# Master perception topology must be explicit and standalone.
grep -q 'os.path.join(yolo_share, "launch", "camera.launch.py")' "$LAUNCH" || fail 'autonomous camera is not using master camera.launch.py'
grep -q 'executable="obstacle_detector_node"' "$LAUNCH" || fail 'standalone YOLO node missing'
grep -q 'executable="hole_block_alignment_node.py"' "$LAUNCH" || fail 'standalone fork alignment node missing'
grep -q 'TimerAction(period=0.10, actions=\[camera\])' "$LAUNCH" || fail 'master camera stagger missing'
grep -q 'TimerAction(period=0.50, actions=\[yolo\])' "$LAUNCH" || fail 'master YOLO stagger missing'
grep -q 'TimerAction(period=0.80, actions=\[hole_alignment\])' "$LAUNCH" || fail 'master alignment stagger missing'
if grep -A35 'camera = IncludeLaunchDescription' "$LAUNCH" | grep -q 'perception_all.launch.py'; then
  fail 'autonomous still routes camera through perception_all.launch.py'
fi
# Runtime YAML shadow was a key divergence from the uploaded master.
if grep -q 'def _runtime_config' "$CAMLAUNCH"; then fail 'camera.launch still uses persistent runtime shadow YAML'; fi
if grep -q 'use_composition' "$PALL"; then fail 'perception_all still defaults to component composition instead of master standalone topology'; fi

# GUI must expose both raw topic freshness and numeric perception summary.
grep -q 'self._queue_telemetry("perception"' "$BRIDGE" || fail 'GUI perception summary telemetry missing'
grep -q '"perception_ready"' "$BRIDGE" || fail 'GUI perception readiness metric missing'
grep -q '"perception"\]' "$MAIN" || fail 'Perception Validation page does not consume perception summary'
grep -q '/obstacle_detection/obstacles' "$BRIDGE" || fail 'GUI obstacle stream subscription missing'
grep -q '/fork_alignment/state' "$BRIDGE" || fail 'GUI alignment stream subscription missing'

# Dedicated read-only acceptance must cover the complete chain.
grep -q '/camera/color/image_raw' "$CHECK" || fail 'perception checker camera topic missing'
grep -q '/obstacle_detection/obstacles' "$CHECK" || fail 'perception checker obstacle topic missing'
grep -q '/obstacle_detection/visualization' "$CHECK" || fail 'perception checker visualization missing'
grep -q '/fork_alignment/state' "$CHECK" || fail 'perception checker alignment state missing'
grep -q 'perception_runtime_check.py' "$NAV/CMakeLists.txt" || fail 'perception checker not installed'

# Preserve the V58 localization/navigation fixes.
grep -q 'start_pre_amcl_after_sensor_topics = _success_only_exit' "$LAUNCH" || fail 'AMCL decoupling regression'
grep -q 'start_both_costmaps_gate_after_global_inflation' "$LAUNCH" || fail 'costmap join regression'
grep -q 'start_navigation_aux_after_both_costmaps' "$LAUNCH" || fail 'Smac/MPPI activation regression'

python3 -m py_compile "$LAUNCH" "$CAMLAUNCH" "$PALL" "$BRIDGE" "$MAIN" "$CHECK" "$AUTOCHECK"
bash -n "$NAV/tools/run_v59.sh"
pass 'master perception topology + GUI perception telemetry + V58 localization/navigation retained'
