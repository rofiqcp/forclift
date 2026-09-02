#!/usr/bin/env bash
set -euo pipefail
WS="${AGV_WS:-/home/otomasi2/ros}"
NAV="$WS/src/navigation"
fail(){ echo "[V46] FAIL: $*" >&2; exit 2; }
[[ -d "$NAV" ]] || fail "navigation source not found at $NAV"
[[ -f /opt/ros/humble/setup.bash ]] || fail '/opt/ros/humble/setup.bash not found'

prune_var(){
  local name="$1" value="${!1-}" out="" part
  IFS=':' read -ra parts <<< "$value"
  for part in "${parts[@]}"; do
    [[ -z "$part" ]] && continue
    [[ "$part" == "$WS/install"* ]] && continue
    out="${out:+$out:}$part"
  done
  export "$name=$out"
}
for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PATH; do prune_var "$var"; done
export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin${PATH:+:$PATH}"
source /opt/ros/humble/setup.bash

if ! pkg-config --exists Qt5Widgets 2>/dev/null; then
  sudo apt-get update
  sudo apt-get install -y qtbase5-dev
fi
if ! python3 -c 'import pytest' >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y python3-pytest
fi

echo '[V46] 1/8 install CP210x helper + udev policy'
AGV_TARGET_USER="${USER}" bash "$NAV/tools/install_sensor_recovery.sh" --install-only

echo '[V46] 2/8 safe CP210x enumeration/recovery'
sudo -n /usr/local/sbin/agv-sensor-recover both || true
udevadm settle --timeout=10 >/dev/null 2>&1 || true

echo '[V46] 3/8 static verification'
bash "$NAV/tools/verify_v44_humble_build_fix.sh" "$NAV"
bash "$NAV/tools/verify_v45_warning_free_build.sh" "$NAV"
bash "$NAV/tools/verify_v46_gui_sensor_startup.sh" "$NAV"

echo '[V46] 4/8 clean package build state'
rm -rf "$WS/build/navigation" "$WS/install/navigation" "$WS/log"
mkdir -p "$WS/log"
cd "$WS"

echo '[V46] 5/8 colcon build'
colcon build --symlink-install --event-handlers console_direct+
source "$WS/install/setup.bash"

echo '[V46] 6/8 executable verification'
EXECUTABLES="$(ros2 pkg executables navigation)"
for exe in agv_gui_cpp mapping_gui_cpp goal_pose_nav2_bridge scan_self_filter navigation_runtime_validator lidar_node imu_node; do
  grep -Eq "^navigation[[:space:]]+$exe$" <<< "$EXECUTABLES" || fail "installed executable missing: $exe"
  echo "[V46] PASS executable: $exe"
done

echo '[V46] 7/8 launch/test verification'
python3 -m py_compile "$NAV"/launch/*.launch.py
colcon test --packages-select navigation --event-handlers console_direct+
colcon test-result --test-result-base "$WS/build/navigation" --verbose

echo '[V46] 8/8 compiler/CMake warning scan'
if find "$WS/log" -type f -name 'stderr.log' -path '*navigation*' -print0 2>/dev/null | xargs -0 -r grep -Ein '(^|[^a-z])(warning:|CMake Warning)' ; then
  fail 'navigation build/test log contains compiler/CMake warnings'
fi

echo '[V46] COMPLETE: clean build and runtime prerequisites installed.'
echo '[V46] Start: ros2 launch navigation gui.launch.py'
echo '[V46] After GUI is green, verify real sensor streams with:'
echo "         bash $NAV/tools/validate_v46_sensor_live.sh sensor"
echo '[V46] After 2D Pose Estimate / AMCL convergence:'
echo "         bash $NAV/tools/validate_v46_sensor_live.sh navigation"
echo '[V46] After clicking a Nav2 Goal:'
echo "         bash $NAV/tools/validate_v46_sensor_live.sh goal"
