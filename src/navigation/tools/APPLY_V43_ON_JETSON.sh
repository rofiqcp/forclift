#!/usr/bin/env bash
set -euo pipefail
WS="${AGV_WS:-/home/otomasi2/ros}"
NAV="$WS/src/navigation"

fail(){ echo "[V43] FAIL: $*" >&2; exit 2; }
[[ -d "$NAV" ]] || fail "navigation source not found at $NAV"
[[ -f /opt/ros/humble/setup.bash ]] || fail '/opt/ros/humble/setup.bash not found'

# Remove only this workspace's stale overlay entries before deleting its old
# install tree. This prevents the AMENT_PREFIX_PATH/CMAKE_PREFIX_PATH warnings
# seen in log(6)/(7) while preserving /opt/ros and external Jetson overlays.
prune_var() {
  local name="$1" value="${!1-}" out="" part
  IFS=':' read -ra parts <<< "$value"
  for part in "${parts[@]}"; do
    [[ -z "$part" ]] && continue
    [[ "$part" == "$WS/install"* ]] && continue
    out="${out:+$out:}$part"
  done
  export "$name=$out"
}
for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PATH; do
  prune_var "$var"
done
# PATH must still contain the normal OS toolchain after pruning.
export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin${PATH:+:$PATH}"
source /opt/ros/humble/setup.bash

if ! pkg-config --exists Qt5Widgets 2>/dev/null; then
  echo '[V43] Qt5 development headers are required for the native C++ GUI.'
  if command -v sudo >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y qtbase5-dev
  else
    fail 'Qt5Widgets development package missing and sudo is unavailable'
  fi
fi

echo '[V43] 1/6 static source verification'
bash "$NAV/tools/verify_v43_cpp_path_gui.sh" "$NAV"

echo '[V43] 2/6 clean navigation build state'
rm -rf "$WS/build/navigation" "$WS/install/navigation"
mkdir -p "$WS/log"
cd "$WS"

echo '[V43] 3/6 colcon build'
colcon build --symlink-install --event-handlers console_direct+

echo '[V43] 4/6 source new install and verify C++ executables'
source "$WS/install/setup.bash"
EXECUTABLES="$(ros2 pkg executables navigation)"
for exe in agv_gui_cpp mapping_gui_cpp goal_pose_nav2_bridge scan_self_filter navigation_runtime_validator lidar_node imu_node; do
  grep -Eq "^navigation[[:space:]]+$exe$" <<< "$EXECUTABLES" || fail "installed executable missing: $exe"
  echo "[V43] PASS executable: $exe"
done

echo '[V43] 5/6 launch-file syntax and package discovery'
python3 -m py_compile "$NAV"/launch/*.launch.py
ros2 pkg prefix navigation >/dev/null

echo '[V43] 6/6 tests'
colcon test --packages-select navigation --event-handlers console_direct+
colcon test-result --test-result-base "$WS/build/navigation" --verbose

cat <<'EOF'
[V43] COMPLETE: source verifier + clean build + install check + tests passed.
[V43] Start:
  ros2 launch navigation gui.launch.py
[V43] Then set 2D Pose Estimate and test baseline:
  bash src/navigation/tools/validate_v43_navigation_live.sh
[V43] Click a Nav2 Goal, then verify an actual Smac path:
  bash src/navigation/tools/validate_v43_navigation_live.sh path
EOF
