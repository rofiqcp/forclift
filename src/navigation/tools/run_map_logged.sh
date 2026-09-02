#!/usr/bin/env bash
set -euo pipefail
WS="${AGV_WS:-/home/otomasi2/ros}"
source /opt/ros/humble/setup.bash
if [[ -f /home/otomasi2/jetson_install/ros_overlay/install/setup.bash ]]; then
  source /home/otomasi2/jetson_install/ros_overlay/install/setup.bash
fi
source "${WS}/install/setup.bash"
mkdir -p "${WS}/log"
LOG="${WS}/log/starburst_fix_$(date +%Y%m%d_%H%M%S).txt"
echo "[AGV-MAP] logging to ${LOG}"
ros2 launch navigation map.launch.py 2>&1 | tee "${LOG}"
