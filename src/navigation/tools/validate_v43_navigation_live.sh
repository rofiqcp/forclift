#!/usr/bin/env bash
set -euo pipefail
WS="${AGV_WS:-/home/otomasi2/ros}"
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo '[V43-LIVE] FAIL: workspace is not built; install/setup.bash is missing' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
REQ_PATH="${1:-baseline}"
case "$REQ_PATH" in
  path|PATH|goal|GOAL|true|1)
    echo '[V43-LIVE] Checking sensors + localization + costmaps + action servers + /smac_plan.'
    echo '[V43-LIVE] Set a valid 2D Pose Estimate first, then click a Nav2 Goal in known free space.'
    exec ros2 run navigation navigation_runtime_validator --ros-args -p require_path:=true -p require_amcl:=true
    ;;
  *)
    echo '[V43-LIVE] Checking sensors + localization + costmaps + Nav2 action servers.'
    exec ros2 run navigation navigation_runtime_validator --ros-args -p require_path:=false -p require_amcl:=true
    ;;
esac
