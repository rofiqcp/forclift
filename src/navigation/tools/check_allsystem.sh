#!/bin/bash
# All-system diagnostic: verify all AGV topics and nodes are active.
#
# Usage:
#   bash src/navigation/tools/check_allsystem.sh
#
# Checks:
#   /imu/data          — IMU sensor
#   /scan_nav          — LiDAR navigation scan
#   /scan_safety       — minimally filtered collision-safety scan
#   /lidar/safety_healthy — continuous LiDAR safety quality gate
#   /map               — SLAM map
#   /camera/color/image_raw   — Astra RGB camera
#   /camera/color/camera_info — Camera calibration info
#   /obstacle_detection/obstacles     — YOLO detections
#   /obstacle_detection/visualization — YOLO annotated image
#
# For each topic: echo + topic list + 1-message echo (best effort).
# Exit code: 0 if all OK, 1 if any FAIL.

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Topics to check
TOPICS=(
  "/imu/data"
  "/scan_nav"
  "/scan_safety"
  "/lidar/safety_healthy"
  "/map"
  "/camera/color/image_raw"
  "/camera/color/camera_info"
  "/obstacle_detection/obstacles"
  "/obstacle_detection/visualization"
)

# Human-readable names
NAMES=(
  "IMU data"
  "LiDAR navigation scan"
  "LiDAR safety scan"
  "LiDAR safety health"
  "SLAM map"
  "Camera RGB"
  "Camera info"
  "YOLO obstacles"
  "YOLO visualization"
)

check_topic() {
  local topic="$1"
  local label="$2"
  local timeout_sec="${3:-3}"

  # 1. Check topic exists
  if ! ros2 topic list 2>/dev/null | grep -qFx "$topic"; then
    echo -e "  ${RED}[FAIL]${RESET} $label ($topic) — not found in topic list"
    return 1
  fi

  # 2. Echo one message (best effort)
  local output
  output=$(timeout "$timeout_sec" ros2 topic echo "$topic" --once 2>/dev/null || true)

  if [ -z "$output" ]; then
    echo -e "  ${YELLOW}[WARN]${RESET} $label ($topic) — topic exists but no message in ${timeout_sec}s"
    return 0  # topic exists, just no message right now
  fi

  echo -e "  ${GREEN}[OK]${RESET} $label ($topic)"
  return 0
}

echo -e "${BOLD}=== AGV All-System Diagnostic ===${RESET}"
echo "Workspace: $ROS_WS"
echo ""

# Source ROS environment
if [ -f "$ROS_WS/install/setup.bash" ]; then
  source "$ROS_WS/install/setup.bash" 2>/dev/null || true
elif [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash 2>/dev/null || true
fi

# Check ros2 CLI available
if ! command -v ros2 &>/dev/null; then
  echo -e "${RED}[ERROR] ros2 CLI not found. Source your ROS environment first.${RESET}"
  exit 1
fi

# ── Node list ─────────────────────────────────────────────────────────────────
echo -e "${BOLD}=== Active Nodes ===${RESET}"
NODES=$(ros2 node list 2>/dev/null || echo "")
if [ -z "$NODES" ]; then
  echo -e "  ${YELLOW}[WARN] No nodes active${RESET}"
else
  echo "$NODES" | while read -r n; do
    echo -e "  ${CYAN}$n${RESET}"
  done
fi
echo ""

# ── Topic checks ──────────────────────────────────────────────────────────────
echo -e "${BOLD}=== Topic Status ===${RESET}"
all_ok=0
for i in "${!TOPICS[@]}"; do
  t="${TOPICS[$i]}"
  n="${NAMES[$i]}"
  if ! check_topic "$t" "$n"; then
    all_ok=1
  fi
done
echo ""

# ── Frame rate check ──────────────────────────────────────────────────────────
echo -e "${BOLD}=== Topic Hz (5s sample) ===${RESET}"
echo -e "  Sampling for 5 seconds... use Ctrl+C to skip."
for i in "${!TOPICS[@]}"; do
  t="${TOPICS[$i]}"
  n="${NAMES[$i]}"
  hz=$(timeout 5 ros2 topic hz "$t" 2>/dev/null | grep "average rate" | awk '{print $4}' || echo "N/A")
  if [ "$hz" != "N/A" ] && [ -n "$hz" ]; then
    echo -e "  ${CYAN}$n${RESET}: ${GREEN}${hz} Hz${RESET}"
  else
    echo -e "  ${CYAN}$n${RESET}: ${YELLOW}no data${RESET}"
  fi
done
echo ""

# ── USB Camera check ──────────────────────────────────────────────────────────
echo -e "${BOLD}=== USB V4L2 Devices ===${RESET}"
if [ -d /dev/v4l/by-id ]; then
  for f in /dev/v4l/by-id/*; do
    [ -e "$f" ] || continue
    target=$(readlink -f "$f" 2>/dev/null || echo "unknown")
    echo -e "  ${GREEN}$(basename $f)${RESET} -> $target"
  done
fi
if ls /dev/video[0-9]* &>/dev/null; then
  echo "  /dev/video* devices:"
  for d in /dev/video[0-9]*; do
    echo -e "    ${GREEN}$d${RESET}"
  done
fi
echo ""

# ── Gate status ───────────────────────────────────────────────────────────────
echo -e "${BOLD}=== AllSystem Gate Status ===${RESET}"
for gate in sensor camera; do
  topic="/allsystem_gate/${gate}/ready"
  val=$(timeout 2 ros2 topic echo "$topic" --once 2>/dev/null | grep "data:" | awk '{print $2}' || echo "unknown")
  if [ "$val" = "True" ] || [ "$val" = "true" ]; then
    echo -e "  ${GREEN}${gate}-gate: READY${RESET}"
  elif [ "$val" = "False" ] || [ "$val" = "false" ]; then
    echo -e "  ${YELLOW}${gate}-gate: WAITING${RESET}"
  else
    echo -e "  ${YELLOW}${gate}-gate: UNKNOWN${RESET} (topic may not exist yet)"
  fi
done
echo ""

# ── Summary ───────────────────────────────────────────────────────────────────
echo -e "${BOLD}=== Summary ===${RESET}"
if [ "$all_ok" -eq 0 ]; then
  echo -e "  ${GREEN}All topics active${RESET}"
else
  echo -e "  ${RED}Some topics missing or unreachable${RESET}"
fi
echo ""
echo "Done."
exit $all_ok
