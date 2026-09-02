#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GUI="$ROOT/navigation/src/agv_gui.cpp"
LAUNCH="$ROOT/navigation/launch/autonomous.launch.py"
CMAKE="$ROOT/navigation/CMakeLists.txt"
PKG="$ROOT/navigation/package.xml"

fail(){ echo "[V54-VERIFY] FAIL: $*" >&2; exit 2; }
pass(){ echo "[V54-VERIFY] PASS: $*"; }

[[ -f "$GUI" ]] || fail "missing $GUI"
[[ -f "$LAUNCH" ]] || fail "missing $LAUNCH"

grep -Fq 'refreshTelemetryPages()' "$GUI" || fail 'live subsystem telemetry refresh missing'
grep -Fq '"/fork_alignment/state"' "$GUI" || fail 'fork alignment state subscription missing'
grep -Fq '"/fork_alignment/image"' "$GUI" || fail 'fork alignment image subscription missing'
grep -Fq '"/obstacle_detection/obstacles"' "$GUI" || fail 'YOLO telemetry missing'
grep -Fq '"/camera/color/image_raw"' "$GUI" || fail 'camera telemetry missing'
grep -Fq '"/winch/connected"' "$GUI" || fail 'winch telemetry missing'
grep -Fq '"/esc/drive_actual_mps"' "$GUI" || fail 'ESC feedback telemetry missing'
if grep -Fq 'NODE UP' "$GUI"; then fail 'ambiguous NODE UP state still exists'; fi
if grep -Fq 'Subsystem telemetry remains available from the Connection page' "$GUI"; then fail 'old placeholder page still exists'; fi
pass 'GUI pages use live ROS telemetry and no NODE UP placeholder remains'

python3 -m py_compile "$LAUNCH"
pass 'autonomous.launch.py parses'

python3 - "$LAUNCH" <<'PY'
from pathlib import Path
import sys
s=Path(sys.argv[1]).read_text()
assert 'start_hole_alignment_independent = TimerAction' in s
assert 'start_hole_alignment_independent,' in s
block=s[s.index('start_perception_after_camera_resolver ='):s.index('camera_ready_notice =', s.index('start_perception_after_camera_resolver ='))]
assert 'actions=[hole_alignment]' not in block, 'fork node still depends on camera resolver block'
print('[V54-VERIFY] PASS: fork alignment has one independent startup path')
PY

grep -Fq 'find_package(yolo_obstacle_detection_ros2 REQUIRED)' "$CMAKE" || fail 'CMake custom message dependency missing'
grep -Fq 'yolo_obstacle_detection_ros2)' "$CMAKE" || fail 'agv_gui target dependency missing'
grep -Fq '<depend>yolo_obstacle_detection_ros2</depend>' "$PKG" || fail 'package.xml build dependency missing'
pass 'navigation links YOLO/fork custom messages correctly'

bash -n "$ROOT/navigation/tools/run_v49.sh"
pass 'launcher shell syntax valid'

echo '[V54-VERIFY] ALL STATIC CHECKS PASSED'
