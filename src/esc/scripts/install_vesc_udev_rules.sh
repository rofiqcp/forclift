#!/usr/bin/env bash
set -euo pipefail
DRIVE=${1:-}; STEER=${2:-}
if [[ -z "$DRIVE" || -z "$STEER" ]]; then
  echo "Usage: sudo $0 <drive-tty-or-by-id> <steer-tty-or-by-id>" >&2
  echo "The script uses USB serial IDs when unique, otherwise stable physical ID_PATH." >&2
  exit 2
fi
for p in "$DRIVE" "$STEER"; do [[ -e "$p" ]] || { echo "Missing device: $p" >&2; exit 2; }; done
props(){ udevadm info -q property -n "$(readlink -f "$1")"; }
prop(){ props "$1" | sed -n "s/^$2=//p" | head -1; }
rule_for(){
  local dev=$1 alias=$2 other=$3
  local vid pid serial path other_serial
  vid=$(prop "$dev" ID_VENDOR_ID); pid=$(prop "$dev" ID_MODEL_ID)
  serial=$(prop "$dev" ID_SERIAL_SHORT); path=$(prop "$dev" ID_PATH)
  other_serial=$(prop "$other" ID_SERIAL_SHORT)
  [[ -n "$vid" && -n "$pid" ]] || { echo "Cannot identify VID/PID for $dev" >&2; return 1; }
  if [[ -n "$serial" && "$serial" != "$other_serial" ]]; then
    printf 'SUBSYSTEM=="tty", ATTRS{idVendor}=="%s", ATTRS{idProduct}=="%s", ATTRS{serial}=="%s", SYMLINK+="%s", MODE="0660", GROUP="dialout"\n' "$vid" "$pid" "$serial" "$alias"
  else
    [[ -n "$path" ]] || { echo "Duplicate/missing serial and no ID_PATH for $dev" >&2; return 1; }
    printf 'SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="%s", ENV{ID_MODEL_ID}=="%s", ENV{ID_PATH}=="%s", SYMLINK+="%s", MODE="0660", GROUP="dialout"\n' "$vid" "$pid" "$path" "$alias"
  fi
}
TMP=$(mktemp)
trap 'rm -f "$TMP"' EXIT
{
  echo '# AGV role aliases generated from explicitly selected physical adapters.'
  rule_for "$DRIVE" vesc_drive "$STEER"
  rule_for "$STEER" vesc_steer "$DRIVE"
} >"$TMP"
install -m 0644 "$TMP" /etc/udev/rules.d/98-agv-vesc.rules
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
echo "Installed role aliases. Verify: ls -l /dev/vesc_drive /dev/vesc_steer"
