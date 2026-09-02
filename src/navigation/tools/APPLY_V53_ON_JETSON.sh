#!/usr/bin/env bash
set -euo pipefail

WS="${HOME}/ros"
SRC="${WS}/src"

echo "[V53] Applying warning-free navigation build fix"

# Keep V52 sensor-recovery installation/current hardware fixes.
if [[ -x "${SRC}/navigation/tools/install_sensor_recovery.sh" ]]; then
  "${SRC}/navigation/tools/install_sensor_recovery.sh"
fi

cd "${WS}"
source /opt/ros/humble/setup.bash
rm -rf build/navigation install/navigation
colcon build --symlink-install --event-handlers console_direct+

echo "[V53] Build completed. Verify that no 'warning:' and no 'had stderr output' remain."
