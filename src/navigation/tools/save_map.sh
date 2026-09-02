#!/usr/bin/env bash
set -euo pipefail
NAME="${1:-agv_map}"
mkdir -p "$(dirname "$NAME")" 2>/dev/null || true
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$NAME'}}"
echo "Map save requested: $NAME.yaml + image"
