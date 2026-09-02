#!/usr/bin/env bash
set -euo pipefail
# One-time setup so esc_keyboard_teleop can read Linux evdev without root.
# Re-login is required after group membership changes.
if ! getent group input >/dev/null; then
  echo "ERROR: Linux group 'input' not found." >&2
  exit 1
fi
sudo usermod -aG input "${SUDO_USER:-$USER}"
echo "Added ${SUDO_USER:-$USER} to group input. Log out and log back in, then verify: groups"
echo "No chmod 777 and no root ROS process are required."
