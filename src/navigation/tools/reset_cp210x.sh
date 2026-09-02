#!/usr/bin/env bash
# Compatibility wrapper for dynamic V41 CP210x recovery.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-both}"
case "$MODE" in imu|lidar|both) ;; *) echo "Usage: $0 [imu|lidar|both]" >&2; exit 2;; esac
if [[ -x /usr/local/sbin/agv-sensor-recover ]]; then
  exec sudo /usr/local/sbin/agv-sensor-recover "$MODE"
fi
exec sudo /usr/bin/python3 "$SCRIPT_DIR/cp210x_recover.py" "$MODE"
