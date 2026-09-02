#!/usr/bin/env bash
set -u

echo '=== SERIAL BY-ID ==='
for p in /dev/serial/by-id/*; do
  [ -e "$p" ] || continue
  printf '%s -> %s\n' "$p" "$(readlink -f "$p")"
done

echo
echo '=== SERIAL BY-PATH ==='
for p in /dev/serial/by-path/*; do
  [ -e "$p" ] || continue
  printf '%s -> %s\n' "$p" "$(readlink -f "$p")"
done

echo
echo '=== TTY USB PROPERTIES ==='
for p in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$p" ] || continue
  echo "--- $p ---"
  if command -v udevadm >/dev/null 2>&1; then
    udevadm info -q property -n "$p" 2>/dev/null | \
      grep -E '^(ID_VENDOR|ID_VENDOR_ID|ID_MODEL|ID_MODEL_ID|ID_SERIAL|ID_SERIAL_SHORT|ID_PATH|ID_USB_DRIVER|DEVPATH)=' || true
  fi
  tty="${p##*/}"
  [ -L "/sys/class/tty/$tty/device/driver" ] && echo "DRIVER=$(basename "$(readlink -f "/sys/class/tty/$tty/device/driver")")"
done

echo
echo '=== V4L BY-ID ==='
for p in /dev/v4l/by-id/*; do
  [ -e "$p" ] || continue
  printf '%s -> %s\n' "$p" "$(readlink -f "$p")"
done

echo
echo '=== V4L BY-PATH ==='
for p in /dev/v4l/by-path/*; do
  [ -e "$p" ] || continue
  printf '%s -> %s\n' "$p" "$(readlink -f "$p")"
done

echo
echo '=== VIDEO PROPERTIES ==='
for p in /dev/video*; do
  [ -e "$p" ] || continue
  echo "--- $p ---"
  if command -v udevadm >/dev/null 2>&1; then
    udevadm info -q property -n "$p" 2>/dev/null | \
      grep -E '^(ID_VENDOR|ID_VENDOR_ID|ID_MODEL|ID_MODEL_ID|ID_SERIAL|ID_SERIAL_SHORT|ID_PATH|ID_V4L_PRODUCT|DEVPATH)=' || true
  fi
  if command -v v4l2-ctl >/dev/null 2>&1; then
    v4l2-ctl -d "$p" --all 2>/dev/null | grep -E 'Driver name|Card type|Bus info|Capabilities' | head -n 8 || true
  fi
done

echo
echo '=== RESOLVED AGV ROLES ==='
for role in imu lidar camera; do
  p="/tmp/agv_devices/$role"
  if [ -L "$p" ] || [ -e "$p" ]; then
    printf '%-8s %s -> %s\n' "$role" "$p" "$(readlink -f "$p" 2>/dev/null || true)"
    [ -L "$p" ] && printf '         first-hop -> %s\n' "$(readlink "$p" 2>/dev/null || true)"
  else
    printf '%-8s %s\n' "$role" '[NOT RESOLVED]'
  fi
done
if [ -f /tmp/agv_devices/roles.json ]; then
  echo
  cat /tmp/agv_devices/roles.json
fi
