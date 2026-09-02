#!/usr/bin/env bash
# V52 one-time installer for AGV CP210x EIO-aware sensor recovery and udev policy.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HELPER_SRC="$SCRIPT_DIR/cp210x_recover.py"
HELPER_DST="/usr/local/sbin/agv-sensor-recover"
SUDOERS_DST="/etc/sudoers.d/agv-sensor-recover"
UDEV_DST="/etc/udev/rules.d/71-agv-cp210x-sensors.rules"
AGV_USER="${AGV_TARGET_USER:-${SUDO_USER:-${USER:-otomasi2}}}"
INSTALL_ONLY=0
[[ "${1:-}" == "--install-only" ]] && INSTALL_ONLY=1

if [[ ! -f "$HELPER_SRC" ]]; then
  echo "[AGV-SERIAL-SETUP] helper source missing: $HELPER_SRC" >&2
  exit 2
fi

as_root() {
  if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

as_root install -o root -g root -m 0755 "$HELPER_SRC" "$HELPER_DST"

# Limit passwordless privilege to the fixed recovery executable only.
printf '%s ALL=(root) NOPASSWD: %s\n' "$AGV_USER" "$HELPER_DST" | as_root tee "$SUDOERS_DST" >/dev/null
as_root chmod 0440 "$SUDOERS_DST"
as_root visudo -cf "$SUDOERS_DST" >/dev/null

cat <<'RULES' | as_root tee "$UDEV_DST" >/dev/null
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0660", TAG+="uaccess", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1"
ACTION=="add|change", SUBSYSTEM=="usb", ATTR{idVendor}=="10c4", ATTR{idProduct}=="ea60", TEST=="power/control", ATTR{power/control}="on"
ACTION=="add|change", SUBSYSTEM=="usb", ATTR{idVendor}=="10c4", ATTR{idProduct}=="ea60", TEST=="power/autosuspend_delay_ms", ATTR{power/autosuspend_delay_ms}="-1"
RULES
as_root chmod 0644 "$UDEV_DST"

# Keep dialout as a secondary permission path. uaccess covers the current GUI
# session immediately; group membership becomes useful after the next login.
if id "$AGV_USER" >/dev/null 2>&1; then
  as_root usermod -aG dialout "$AGV_USER" || true
fi

as_root /sbin/modprobe usbserial >/dev/null 2>&1 || true
as_root /sbin/modprobe cp210x >/dev/null 2>&1 || true
as_root udevadm control --reload-rules
as_root udevadm trigger --subsystem-match=usb --action=change >/dev/null 2>&1 || true
as_root udevadm trigger --subsystem-match=tty --action=change >/dev/null 2>&1 || true
as_root udevadm settle --timeout=10 >/dev/null 2>&1 || true

echo "[AGV-SERIAL-SETUP] recovery helper and CP210x udev policy installed"

if (( INSTALL_ONLY == 0 )); then
  if [[ ${EUID:-$(id -u)} -eq 0 ]]; then
    "$HELPER_DST" both
  else
    sudo -n "$HELPER_DST" both
  fi
fi
