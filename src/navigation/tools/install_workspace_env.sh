#!/usr/bin/env bash
set -Eeuo pipefail

WS="${1:-/home/otomasi2/ros}"
BASHRC="${HOME}/.bashrc"
BEGIN='# >>> AGV ROS2 WORKSPACE >>>'
END='# <<< AGV ROS2 WORKSPACE <<<'

fail() {
  echo "[AGV-ENV][FAIL] $*" >&2
  exit 1
}

[[ -f /opt/ros/humble/setup.bash ]] || fail "/opt/ros/humble/setup.bash not found"
[[ -f "${WS}/install/setup.bash" ]] || fail "${WS}/install/setup.bash not found. Build the workspace first."

# IMPORTANT: install/setup.bash can exist after a partial build containing only
# esc/yolo.  Verify navigation itself before persisting the workspace overlay.
[[ -f "${WS}/install/navigation/share/ament_index/resource_index/packages/navigation" ]] \
  || fail "navigation is not present in the ament index. Run navigation/tools/rebuild_navigation_clean.sh first."
[[ -f "${WS}/install/navigation/share/navigation/package.xml" ]] \
  || fail "navigation share directory is incomplete. Rebuild navigation first."

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u
prefix="$(ros2 pkg prefix navigation 2>/dev/null || true)"
[[ "${prefix}" == "${WS}/install/navigation" ]] \
  || fail "ROS 2 cannot discover navigation from ${WS}/install (got '${prefix:-<empty>}')"

mkdir -p "${WS}/log"
touch "${BASHRC}"

tmp="$(mktemp)"
awk -v b="$BEGIN" -v e="$END" '
  $0==b {skip=1; next}
  $0==e {skip=0; next}
  !skip {print}
' "$BASHRC" > "$tmp" || true
cat >> "$tmp" <<EOF2
$BEGIN
# ROS 2 Humble base -> Jetson overlay -> AGV workspace. Keep workspace LAST.
source /opt/ros/humble/setup.bash
if [ -f /home/otomasi2/jetson_install/ros_overlay/install/setup.bash ]; then
  source /home/otomasi2/jetson_install/ros_overlay/install/setup.bash
fi
if [ -f ${WS}/install/setup.bash ]; then
  source ${WS}/install/setup.bash
fi
mkdir -p ${WS}/log
$END
EOF2
mv "$tmp" "$BASHRC"

echo "[AGV-ENV][OK] Persistent ROS environment installed in ${BASHRC}"
echo "[AGV-ENV][OK] navigation=${prefix}"
echo "[AGV-ENV][NEXT] source ${BASHRC}"
