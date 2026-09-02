#!/usr/bin/env bash
set -euo pipefail
NAV="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PASS=0
FAIL=0
ok(){ echo "[PASS] $*"; PASS=$((PASS+1)); }
bad(){ echo "[FAIL] $*" >&2; FAIL=$((FAIL+1)); }
need(){ local label="$1" file="$2" pattern="$3"; if grep -Eq "$pattern" "$file"; then ok "$label"; else bad "$label"; fi; }

echo '=== V44 ROS 2 Humble / GCC11 build compatibility verifier ==='
need 'scan filter clamps median radius after explicit int conversion' "$NAV/src/scan_self_filter.cpp" 'median_radius_bins_ = std::max\(1, static_cast<int>\(declare_parameter<int>'
need 'scan filter clamps neighbor support after explicit int conversion' "$NAV/src/scan_self_filter.cpp" 'min_neighbor_support_ = std::max\(1, static_cast<int>\(declare_parameter<int>'
need 'GoalPose free threshold uses explicit int conversion' "$NAV/src/goal_pose_nav2_bridge.cpp" 'free_threshold_ = static_cast<int>\(declare_parameter<int>'
need 'GoalPose retry count uses explicit int conversion' "$NAV/src/goal_pose_nav2_bridge.cpp" 'max_plan_retries_ = static_cast<int>\(declare_parameter<int>'
if grep -RInE 'std::(max|min)\([^\n]*declare_parameter<int>' "$NAV/src" --include='*.cpp' >/tmp/v44_bad_parameter_max.txt 2>/dev/null; then
  cat /tmp/v44_bad_parameter_max.txt >&2
  bad 'no mixed int/ROS-integer std::max/min pattern remains'
else
  ok 'no mixed int/ROS-integer std::max/min pattern remains'
fi
for f in agv_gui.cpp mapping_gui.cpp goal_pose_nav2_bridge.cpp scan_self_filter.cpp runtime_validator.cpp lidar_node.cpp imu_node.cpp; do
  if [[ -f "$NAV/src/$f" ]]; then ok "C++ runtime source present: $f"; else bad "C++ runtime source missing: $f"; fi
done
for f in scripts/agv_gui.py scripts/mapping_gui.py scripts/goal_pose_nav2_bridge.py scripts/scan_self_filter.py; do
  if [[ ! -e "$NAV/$f" ]]; then ok "legacy Python runtime absent: $f"; else bad "legacy Python runtime still present: $f"; fi
done
if [[ ! -d "$NAV/python/navigation_gui" ]]; then ok 'legacy Python navigation_gui package absent'; else bad 'legacy Python navigation_gui package still present'; fi

echo
printf 'V44 STATIC RESULT: %d PASS / %d FAIL\n' "$PASS" "$FAIL"
[[ "$FAIL" -eq 0 ]]
