#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
fail(){ echo "[V55-VERIFY] FAIL: $*" >&2; exit 2; }

for f in \
  "$ROOT/RUN_GUI.sh" \
  "$ROOT/RUN_MAPPING.sh" \
  "$ROOT/RUN_AUTONOMOUS.sh" \
  "$ROOT/RUN_CHECK.sh" \
  "$ROOT/navigation/tools/run_v55.sh" \
  "$ROOT/navigation/tools/run_gui" \
  "$ROOT/navigation/tools/run_mapping" \
  "$ROOT/navigation/tools/run_autonomous"
do
  [[ -f "$f" ]] || fail "missing $f"
  bash -n "$f" || fail "bash syntax error: $f"
done

for f in "$ROOT/RUN_GUI.sh" "$ROOT/RUN_MAPPING.sh" "$ROOT/RUN_AUTONOMOUS.sh" \
         "$ROOT/navigation/tools/run_gui" "$ROOT/navigation/tools/run_mapping" "$ROOT/navigation/tools/run_autonomous"; do
  grep -q 'run_v55.sh' "$f" || fail "$f does not use run_v55.sh"
done

# Regression check: optional fork/camera/winch/yolo discovery is warning-only in
# the root launcher. It may never be a common fail-fast prerequisite again.
if grep -nE '^check_exe yolo_obstacle_detection_ros2|^check_exe esc ' "$ROOT/navigation/tools/run_v55.sh"; then
  fail 'optional subsystem restored as hard launch prerequisite'
fi

grep -q 'source "$WS/install/setup.bash"' "$ROOT/navigation/tools/run_v55.sh" || \
  fail 'workspace setup.bash is not sourced'
grep -q 'jetson_install/ros_overlay/install/setup.bash' "$ROOT/navigation/tools/run_v55.sh" || \
  fail 'Jetson runtime overlay recovery missing'
grep -q 'runtime_v55_' "$ROOT/navigation/tools/run_v55.sh" || \
  fail 'deterministic ROS_LOG_DIR missing'
grep -q 'tools/run_v55.sh' "$ROOT/navigation/CMakeLists.txt" || \
  fail 'run_v55.sh is not installed with navigation'

echo '[V55-VERIFY] PASS'
