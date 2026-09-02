#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NAV="$ROOT/navigation"
fail(){ echo "[V56-VERIFY] FAIL: $*" >&2; exit 2; }
pass(){ echo "[V56-VERIFY] PASS: $*"; }

for f in \
  "$NAV/python/navigation_gui/main_window.py" \
  "$NAV/python/navigation_gui/pages.py" \
  "$NAV/python/navigation_gui/map_canvas.py" \
  "$NAV/python/navigation_gui/map_model.py" \
  "$NAV/python/navigation_gui/ros_bridge.py" \
  "$NAV/python/navigation_gui/widgets.py" \
  "$NAV/scripts/agv_gui.py" \
  "$NAV/scripts/agv_gui_launcher.sh" \
  "$NAV/launch/gui.launch.py" \
  "$NAV/launch/autonomous.launch.py" \
  "$NAV/tools/rviz_robot_model_ready_gate.py" \
  "$NAV/tools/run_v56.sh"
do
  [[ -f "$f" ]] || fail "missing $f"
done
pass 'master GUI and V56 runtime files present'

python3 -m py_compile \
  "$NAV/python/navigation_gui/main_window.py" \
  "$NAV/python/navigation_gui/pages.py" \
  "$NAV/python/navigation_gui/map_canvas.py" \
  "$NAV/python/navigation_gui/map_model.py" \
  "$NAV/python/navigation_gui/ros_bridge.py" \
  "$NAV/python/navigation_gui/widgets.py" \
  "$NAV/launch/gui.launch.py" \
  "$NAV/launch/autonomous.launch.py" \
  "$NAV/tools/rviz_robot_model_ready_gate.py"
pass 'Python syntax valid'

bash -n "$NAV/scripts/agv_gui_launcher.sh"
bash -n "$NAV/tools/run_v56.sh"
pass 'shell syntax valid'

grep -Fq 'class MapCanvas' "$NAV/python/navigation_gui/map_canvas.py" || fail 'MapCanvas missing'
grep -Fq 'RingPlot' "$NAV/python/navigation_gui/pages.py" || fail 'realtime graph widget not wired into subsystem pages'
grep -Fq 'class RingPlot' "$NAV/python/navigation_gui/widgets.py" || fail 'RingPlot implementation missing'
grep -Fq 'MapPage' "$NAV/python/navigation_gui/main_window.py" || fail 'Map 1/2/3 pages missing'
pass 'master Map + realtime graph GUI restored'

grep -Fq '"/scan_nav"' "$NAV/python/navigation_gui/ros_bridge.py" || fail '/scan_nav telemetry missing'
grep -Fq '"/scan_safety"' "$NAV/python/navigation_gui/ros_bridge.py" || fail '/scan_safety telemetry missing'
grep -Fq '"/lidar/safety_healthy"' "$NAV/python/navigation_gui/ros_bridge.py" || fail 'LiDAR safety health missing'
grep -Fq '"/system/manual_motion_allowed"' "$NAV/python/navigation_gui/ros_bridge.py" || fail 'manual interlock missing'
grep -Fq '"/navigation/planner_status"' "$NAV/python/navigation_gui/ros_bridge.py" || fail 'planner status missing'
grep -Fq '"/esc/mux/selected"' "$NAV/python/navigation_gui/ros_bridge.py" || fail 'ESC mux selected telemetry missing'
if grep -Fq 'create_subscription(LaserScan, "/scan",' "$NAV/python/navigation_gui/ros_bridge.py"; then
  fail 'legacy /scan is still the primary GUI LaserScan subscription'
fi
pass 'V55 live telemetry semantics retained in master GUI'

grep -Fq 'agv_gui_launcher.sh' "$NAV/launch/gui.launch.py" || fail 'gui.launch does not start master PyQt launcher'
grep -Fq 'ExecuteProcess' "$NAV/launch/gui.launch.py" || fail 'GUI is not launched with ABI-safe Python shell launcher'
grep -Fq 'python/navigation_gui' "$NAV/CMakeLists.txt" || fail 'navigation_gui install rule missing'
pass 'GUI install/launch path is production-ready'

grep -Fq 'rviz_robot_model_ready_gate.py' "$NAV/launch/autonomous.launch.py" || fail 'RobotModel readiness gate not used'
grep -Fq '"fail_open": False' "$NAV/launch/autonomous.launch.py" || fail 'RViz gate is not fail-closed for RobotModel startup'
grep -Fq 'start_rviz_after_robot_model_ready' "$NAV/launch/autonomous.launch.py" || fail 'single gated RViz start path missing'
if grep -Fq 'start_rviz_independent' "$NAV/launch/autonomous.launch.py"; then
  fail 'old independent RViz startup race still present'
fi
pass 'autonomous RViz waits for map->visual RobotModel TF instead of racing startup'

for i in 1 2 3; do
  [[ -f "$NAV/maps/map_${i}.yaml" ]] || fail "map_${i}.yaml missing"
  [[ -f "$NAV/maps/map_${i}.pgm" ]] || fail "map_${i}.pgm missing"
done
pass 'Map 1/2/3 source data present'

echo '[V56-VERIFY] ALL STATIC CHECKS PASSED'
