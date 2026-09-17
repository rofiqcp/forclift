#!/usr/bin/env bash
set -euo pipefail
WS="${AGV_WS:-/home/otomasi2/forclift}"
MODE="${1:-gui}"
[[ -f /opt/ros/humble/setup.bash ]] || { echo '[V47-RUN] /opt/ros/humble/setup.bash missing' >&2; exit 2; }
[[ -f "$WS/install/setup.bash" ]] || { echo "[V47-RUN] $WS/install/setup.bash missing; run APPLY_V47_ON_JETSON.sh first" >&2; exit 2; }
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
case "$MODE" in
  gui) exec ros2 launch navigation gui.launch.py "${@:2}" ;;
  map|mapping) exec ros2 launch navigation map.launch.py "${@:2}" ;;
  autonomous|auto) exec ros2 launch navigation autonomous.launch.py "${@:2}" ;;
  allsystem|all) exec ros2 launch navigation allsystem.launch.py "${@:2}" ;;
  imu) exec ros2 launch navigation imu.launch.py "${@:2}" ;;
  lidar) exec ros2 launch navigation lidar.launch.py "${@:2}" ;;
  esc) exec ros2 launch esc esc.launch.py "${@:2}" ;;
  yolo|perception) exec ros2 launch yolo_obstacle_detection_ros2 perception_all.launch.py "${@:2}" ;;
  validate) exec ros2 run navigation navigation_runtime_validator "${@:2}" ;;
  *) echo 'Usage: run_v47.sh {gui|map|autonomous|allsystem|imu|lidar|esc|yolo|validate} [ROS args...]' >&2; exit 2 ;;
esac
