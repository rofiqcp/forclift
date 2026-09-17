#!/usr/bin/env bash
# Environment portable untuk workspace AGV.
# Folder project utama bernama "forclift"; lokasi HOME boleh berbeda antar PC.

export AGV_ROOT="${AGV_ROOT:-$HOME/forclift}"
export AGV_PYTHON="${AGV_PYTHON:-/usr/bin/python3}"

# Samakan ROS CLI dengan domain/local-only yang dipakai autonomous.launch.py.
# Environment existing tetap dihormati; default project adalah domain 42 lokal-host.
export AGV_ROS_DOMAIN_ID="${AGV_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-42}}"
export AGV_ROS_LOCALHOST_ONLY="${AGV_ROS_LOCALHOST_ONLY:-${ROS_LOCALHOST_ONLY:-1}}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$AGV_ROS_DOMAIN_ID}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-$AGV_ROS_LOCALHOST_ONLY}"
export PATH="$HOME/.local/bin:$PATH"

export AGV_CONFIG_DIR="${AGV_CONFIG_DIR:-$AGV_ROOT/src/navigation/config}"
export AGV_ESC_CONFIG_DIR="${AGV_ESC_CONFIG_DIR:-$AGV_ROOT/src/esc/config}"
export AGV_PERCEPTION_CONFIG_DIR="${AGV_PERCEPTION_CONFIG_DIR:-$AGV_ROOT/src/yolo_obstacle_detection_ros2/config}"

export YOLOPV2_PT_PATH="${YOLOPV2_PT_PATH:-$AGV_ROOT/models/yolopv2.pt}"
export YOLOP_ENGINE_PATH="${YOLOP_ENGINE_PATH:-$AGV_ROOT/models/yolopv2.engine}"
export ASTRA_YOLOP_MODELS_DIR="${ASTRA_YOLOP_MODELS_DIR:-$AGV_ROOT/models}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [[ -f "$AGV_ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "$AGV_ROOT/install/setup.bash"
fi

# Prefer the locally rebuilt navigation package over older overlays.
if [[ -f "$AGV_ROOT/install/navigation/share/navigation/local_setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "$AGV_ROOT/install/navigation/share/navigation/local_setup.bash"
fi
