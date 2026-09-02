#!/usr/bin/env bash
set -Eeuo pipefail

WS="${1:-/home/otomasi2/ros}"
SRC="${WS}/src/navigation"
LOG_DIR="${WS}/log"
LOG_FILE="${LOG_DIR}/navigation_rebuild.log"

fail() {
  echo "[NAV-RECOVERY][FAIL] $*" >&2
  exit 1
}

[[ -f /opt/ros/humble/setup.bash ]] || fail "/opt/ros/humble/setup.bash not found"
[[ -f "${SRC}/package.xml" ]] || fail "${SRC}/package.xml not found"
[[ -f "${SRC}/CMakeLists.txt" ]] || fail "${SRC}/CMakeLists.txt not found"
command -v colcon >/dev/null 2>&1 || fail "colcon not found"

mkdir -p "${LOG_DIR}"
cd "${WS}"

# Start from the ROS base underlay only.  Do not inherit a partially-built
# workspace overlay that may contain esc/yolo but not navigation.
set +u
source /opt/ros/humble/setup.bash
set -u

# Confirm colcon can discover exactly the package we are about to build.
if ! colcon list | awk '$1=="navigation" && $2=="src/navigation" {found=1} END{exit !found}'; then
  fail "colcon cannot discover navigation at src/navigation"
fi

echo "[NAV-RECOVERY] Removing stale navigation build/install state"
rm -rf "${WS}/build/navigation" "${WS}/install/navigation"

# CMake cache from a previous partial install can retain stale Python or prefix
# information, therefore --cmake-clean-cache is intentional here.
echo "[NAV-RECOVERY] Building navigation"
set +e
colcon build \
  --packages-select navigation \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_direct+ 2>&1 | tee "${LOG_FILE}"
rc=${PIPESTATUS[0]}
set -e
[[ ${rc} -eq 0 ]] || fail "navigation build failed; see ${LOG_FILE}"

# A successful colcon return is not enough for this AGV: verify the ament index,
# package share directory and the launch files explicitly.
[[ -f "${WS}/install/navigation/share/ament_index/resource_index/packages/navigation" ]] \
  || fail "ament index marker for navigation was not installed"
[[ -f "${WS}/install/navigation/share/navigation/package.xml" ]] \
  || fail "navigation package.xml was not installed"
[[ -f "${WS}/install/navigation/share/navigation/launch/autonomous.launch.py" ]] \
  || fail "autonomous.launch.py was not installed"
[[ -f "${WS}/install/navigation/share/navigation/launch/gui.launch.py" ]] \
  || fail "gui.launch.py was not installed"
[[ -f "${WS}/install/navigation/share/navigation/launch/map.launch.py" ]] \
  || fail "map.launch.py was not installed"
[[ -f "${WS}/install/setup.bash" ]] || fail "workspace install/setup.bash missing"

# Re-source from a clean shell context and verify ROS 2 package discovery.
set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

prefix="$(ros2 pkg prefix navigation 2>/dev/null || true)"
[[ "${prefix}" == "${WS}/install/navigation" ]] \
  || fail "ros2 pkg prefix navigation returned '${prefix:-<empty>}'"

# Do not fail on harmless third-party text, but report compiler/CMake warnings
# from this package build explicitly so the user knows whether the build was clean.
warning_count="$(grep -Eic '(^|[[:space:]])warning:|CMake Warning' "${LOG_FILE}" || true)"
error_count="$(grep -Eic '(^|[[:space:]])error:|CMake Error' "${LOG_FILE}" || true)"

if [[ "${error_count}" -ne 0 ]]; then
  fail "build log still contains ${error_count} error line(s); see ${LOG_FILE}"
fi
if [[ "${warning_count}" -ne 0 ]]; then
  echo "[NAV-RECOVERY][WARN] build succeeded but ${warning_count} warning line(s) remain in ${LOG_FILE}" >&2
else
  echo "[NAV-RECOVERY] Build log clean: no compiler/CMake warnings detected"
fi

echo "[NAV-RECOVERY][OK] navigation=${prefix}"
echo "[NAV-RECOVERY][OK] autonomous/gui/map launch files installed"
echo "[NAV-RECOVERY][NEXT] source ${WS}/install/setup.bash"
