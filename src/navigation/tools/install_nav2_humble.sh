#!/usr/bin/env bash
set -euo pipefail
if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "[NAV2-INSTALL] Expected ROS_DISTRO=humble. Source /opt/ros/humble/setup.bash first." >&2
  exit 2
fi
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
printf '%s\n' '[NAV2-INSTALL] Nav2 Humble installation completed.'
