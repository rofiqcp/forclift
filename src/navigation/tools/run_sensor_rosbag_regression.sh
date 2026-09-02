#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 1 ]]; then echo "Usage: $0 BAG_DIR [duration_sec]" >&2; exit 2; fi
BAG=$1; DURATION=${2:-30}
if ! command -v ros2 >/dev/null; then echo "ros2 not found" >&2; exit 2; fi
if [[ ! -e "$BAG" ]]; then echo "bag not found: $BAG" >&2; exit 2; fi
PREFIX=$(ros2 pkg prefix navigation 2>/dev/null || true)
OBSERVER="${PREFIX:+$PREFIX/lib/navigation/}sensor_replay_regression_observer.py"
if [[ -z "$PREFIX" || ! -f "$OBSERVER" ]]; then
  # Source-tree fallback for developer use.
  OBSERVER="$(cd "$(dirname "$0")/../scripts" && pwd)/sensor_replay_regression_observer.py"
fi
python3 "$OBSERVER" --duration "$DURATION" & OBS=$!
trap 'kill $OBS 2>/dev/null || true' EXIT
# Replay only sensor/hardware-observation inputs. EKF/AMCL/Smac outputs are
# omitted so the currently built stack must recompute them.
if ! ros2 bag play "$BAG" --clock --rate 1.0 --topics \
  /scan_nav /scan_safety /lidar/status /imu/data /esc/odom /esc/ready \
  /tf_static /initialpose_safe; then
  echo "rosbag playback failed" >&2
  kill $OBS 2>/dev/null || true
  wait $OBS 2>/dev/null || true
  exit 3
fi
wait $OBS
trap - EXIT
