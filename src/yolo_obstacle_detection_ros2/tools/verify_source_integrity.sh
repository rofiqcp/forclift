#!/usr/bin/env bash
set -euo pipefail
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
required=(
  CMakeLists.txt package.xml
  msg/Obstacle.msg msg/ObstacleArray.msg msg/Pose2DYaw.msg
  msg/DockingState.msg msg/PalletPose.msg msg/FloorMarking.msg msg/AlignmentState.msg
  src/obstacle_detector_node.cpp src/pallet_pose_estimator.cpp src/astra_rgb_v4l2_node.cpp
  include/yolo_obstacle_detection_ros2/astra_rgb_driver.hpp
  hole_block_alignment/hole_block_alignment_node.py
  launch/camera.launch.py launch/perception_all.launch.py
  config/camera_v4l2.yaml config/test_params.yaml config/alignment_realtime.yaml
)
missing=0
for f in "${required[@]}"; do
  if [[ -s "$PKG_DIR/$f" ]]; then
    printf '[PASS] %s\n' "$f"
  else
    printf '[FAIL] %s\n' "$f"
    missing=1
  fi
done
if [[ ! -x /usr/bin/python3 ]]; then
  echo '[FAIL] /usr/bin/python3 missing'
  missing=1
else
  pyver="$(/usr/bin/python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  if [[ "$pyver" == "3.10" ]]; then
    echo "[PASS] /usr/bin/python3 = Python $pyver"
  else
    echo "[FAIL] /usr/bin/python3 = Python $pyver (ROS 2 Humble requires 3.10 here)"
    missing=1
  fi
fi
if (( missing )); then
  echo '[YOLO-SOURCE-CHECK] FAIL'
  exit 2
fi
echo '[YOLO-SOURCE-CHECK] PASS'
