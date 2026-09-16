#!/usr/bin/env bash
# V33 perception-only ownership cleanup. This exits quickly and is deliberately
# independent of the serial IMU/LiDAR preflight.
set -u

echo "[PERCEPTION-PREFLIGHT-V59] clearing stale camera/YOLO/alignment/component owners"
patterns=(
  "/yolo_obstacle_detection_ros2/lib/yolo_obstacle_detection_ros2/astra_rgb_v4l2_node"
  "/yolo_obstacle_detection_ros2/lib/yolo_obstacle_detection_ros2/obstacle_detector_node"
  "/yolo_obstacle_detection_ros2/lib/yolo_obstacle_detection_ros2/warehouse_person_detector_node"
  "hole_block_alignment_node.py"
  "__node:=perception_container"
  "/perception_container"
)

for pat in "${patterns[@]}"; do
  pkill -TERM -f -- "$pat" 2>/dev/null || true
done
sleep 0.35
for pat in "${patterns[@]}"; do
  pkill -KILL -f -- "$pat" 2>/dev/null || true
done

# Lock files are advisory only; after owners are gone they may be removed safely.
rm -f /tmp/agv_camera_video*.lock 2>/dev/null || true

# Let udev finish any UVC enumeration already in progress. Never block startup
# indefinitely; the camera node itself keeps rediscovering every second.
if command -v udevadm >/dev/null 2>&1; then
  udevadm settle --timeout=3 >/dev/null 2>&1 || true
fi

echo "[PERCEPTION-PREFLIGHT] video nodes:"
ls -l /dev/video* 2>/dev/null || echo "[PERCEPTION-PREFLIGHT] no /dev/video* yet; camera node will keep retrying"
echo "[PERCEPTION-PREFLIGHT] done"
exit 0
