#!/usr/bin/env bash
set -euo pipefail
WS="${AGV_WS:-/home/otomasi2/forclift}"
NAV="$WS/src/navigation"
fail(){ echo "[V45] FAIL: $*" >&2; exit 2; }
[[ -d "$NAV" ]] || fail "navigation source not found at $NAV"
[[ -f /opt/ros/humble/setup.bash ]] || fail '/opt/ros/humble/setup.bash not found'

# Remove entries belonging to this workspace BEFORE deleting build/install.
# This is what prevents colcon's prefix_path warnings on a clean rebuild.
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
export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin${PATH:+:$PATH}"
source /opt/ros/humble/setup.bash

if ! pkg-config --exists Qt5Widgets 2>/dev/null; then
  echo '[V45] Qt5 development headers missing; installing qtbase5-dev.'
  sudo apt-get update
  sudo apt-get install -y qtbase5-dev
fi

# pytest is optional for ordinary colcon build. Install it here so the apply
# workflow can also execute the project's Python unit tests without warnings.
if ! python3 -c 'import pytest' >/dev/null 2>&1; then
  echo '[V45] pytest missing; installing python3-pytest for test execution.'
  sudo apt-get update
  sudo apt-get install -y python3-pytest
fi

echo '[V45] 1/7 static verification'
bash "$NAV/tools/verify_v44_humble_build_fix.sh" "$NAV"
bash "$NAV/tools/verify_v45_warning_free_build.sh" "$NAV"

echo '[V45] 2/7 clean workspace build state'
rm -rf "$WS/build/navigation" "$WS/install/navigation"
mkdir -p "$WS/log"
cd "$WS"

echo '[V45] 3/7 clean colcon build'
colcon build --symlink-install --event-handlers console_direct+

echo '[V45] 4/7 source install + executable verification'
source "$WS/install/setup.bash"
EXECUTABLES="$(ros2 pkg executables navigation)"
for exe in agv_gui_cpp mapping_gui_cpp goal_pose_nav2_bridge scan_self_filter navigation_runtime_validator lidar_node imu_node; do
  grep -Eq "^navigation[[:space:]]+$exe$" <<< "$EXECUTABLES" || fail "installed executable missing: $exe"
  echo "[V45] PASS executable: $exe"
done

echo '[V45] 5/7 launch syntax'
python3 -m py_compile "$NAV"/launch/*.launch.py

echo '[V45] 6/7 tests'
colcon test --packages-select navigation --event-handlers console_direct+
colcon test-result --test-result-base "$WS/build/navigation" --verbose

echo '[V45] 7/7 warning scan for latest navigation build logs'
if find "$WS/log" -type f \( -name 'stderr.log' -o -name 'stderr_stderr.log' \) -path '*navigation*' -print0 2>/dev/null | xargs -0 -r grep -Ein '(^|[^a-z])(warning:|CMake Warning)' ; then
  fail 'navigation build/test log still contains compiler/CMake warnings; inspect lines above'
fi

echo '[V45] COMPLETE: clean build + C++ executable checks + tests passed.'
echo '[V45] Start with: ros2 launch navigation gui.launch.py'
