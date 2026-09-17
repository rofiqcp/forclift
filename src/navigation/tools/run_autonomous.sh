#!/usr/bin/env bash
set -euo pipefail
WS="${AGV_WS:-/home/otomasi2/forclift}"
export AGV_WS="$WS"
export AGV_RUNTIME_CONFIG_ROOT="${AGV_RUNTIME_CONFIG_ROOT:-$WS/config/runtime}"
MAP="${1:-auto}"
source /opt/ros/humble/setup.bash
if [[ -f /home/otomasi2/jetson_install/ros_overlay/install/setup.bash ]]; then
  source /home/otomasi2/jetson_install/ros_overlay/install/setup.bash
fi
source "${WS}/install/setup.bash"

NAV_PREFIX="$(ros2 pkg prefix navigation 2>/dev/null || true)"
if [[ "${NAV_PREFIX}" != "${WS}/install/navigation" ]]; then
  echo "[AGV-AUTONOMOUS][FAIL] navigation package is not resolved from ${WS}" >&2
  echo "[AGV-AUTONOMOUS][FAIL] resolved=${NAV_PREFIX:-<empty>}" >&2
  echo "[AGV-AUTONOMOUS][HINT] run: bash ${WS}/src/navigation/tools/rebuild_navigation_clean.sh" >&2
  exit 1
fi

echo "[AGV-AUTONOMOUS] navigation=${NAV_PREFIX}"
mkdir -p "${WS}/log"
LOG="${WS}/log/autonomous_v39_$(date +%Y%m%d_%H%M%S).txt"
echo "[AGV-AUTONOMOUS] logging to ${LOG}"
ros2 launch navigation autonomous.launch.py map:="${MAP}" 2>&1 | tee "${LOG}"
