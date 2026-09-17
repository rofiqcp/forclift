#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="/usr/bin/python3"
GUI_ENTRY="$SCRIPT_DIR/agv_gui.py"
LOG_DIR="/home/otomasi2/forclift/log"
STARTUP_LOG="$LOG_DIR/agv_gui_startup.log"

mkdir -p "$LOG_DIR"
# Keep each launch diagnostic clean.  Rotate the previous startup log so old
# tracebacks / Qt parser messages cannot be mistaken for the current run.
if [[ -s "$STARTUP_LOG" ]]; then
  prev="$LOG_DIR/agv_gui_startup_prev_$(date '+%Y%m%d_%H%M%S').log"
  mv "$STARTUP_LOG" "$prev" || true
fi
touch "$STARTUP_LOG"

log() {
  local msg="[AGV-GUI] $*"
  printf '%s\n' "$msg"
  printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$msg" >> "$STARTUP_LOG"
}
fail() {
  local msg="[AGV-GUI][ERROR] $*"
  printf '%s\n' "$msg" >&2
  printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$msg" >> "$STARTUP_LOG"
  exit 2
}

[[ -x "$PYTHON_BIN" ]] || fail "$PYTHON_BIN tidak ditemukan / tidak executable"
[[ -f "$GUI_ENTRY" ]] || fail "GUI entry tidak ditemukan: $GUI_ENTRY"

PY_VER="$($PYTHON_BIN -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
[[ "$PY_VER" == "3.10" ]] || fail "GUI wajib Python 3.10, tetapi $PYTHON_BIN adalah Python $PY_VER"

# Never inherit Hermes/venv Python ABI into ROS Humble GUI.
unset PYTHONHOME || true
unset VIRTUAL_ENV || true

clean_pythonpath=""
IFS=':' read -r -a _pp_entries <<< "${PYTHONPATH:-}"
for p in "${_pp_entries[@]}"; do
  [[ -z "$p" ]] && continue
  case "$p" in
    *python3.11*|*python3.12*|*python3.13*|*.hermes*|*/venv/*|*/site-packages*)
      if [[ "$p" == *python3.10* || "$p" == /opt/ros/humble/* ]]; then
        :
      else
        continue
      fi
      ;;
  esac
  clean_pythonpath="${clean_pythonpath:+$clean_pythonpath:}$p"
done
export PYTHONPATH="$SCRIPT_DIR:$SCRIPT_DIR/../python${clean_pythonpath:+:$clean_pythonpath}"
export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1

log "launcher=$0"
log "python=$PYTHON_BIN ($($PYTHON_BIN --version 2>&1))"
log "entry=$GUI_ENTRY"
log "DISPLAY=${DISPLAY:-<unset>} WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-<unset>} XAUTHORITY=${XAUTHORITY:-<unset>}"
log "startup_log=$STARTUP_LOG"

[[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || \
  fail "DISPLAY/WAYLAND_DISPLAY tidak tersedia. Jalankan dari terminal desktop Jetson atau X forwarding yang valid."

# Qt is the only hard dependency for GUI startup. ROS is intentionally optional
# until after the window is visible.
$PYTHON_BIN - <<'PY'
import sys
assert sys.version_info[:2] == (3, 10), sys.version
import PyQt5
print(f"[AGV-GUI] Python/PyQt preflight PASS: Python {sys.version.split()[0]}, PyQt5=OK", flush=True)
PY

# Real display smoke test, bounded so a broken Qt platform plugin cannot hang
# ros2 launch forever without an explanation.
if command -v timeout >/dev/null 2>&1; then
  if ! timeout 6s "$PYTHON_BIN" - <<'PY'
from PyQt5.QtWidgets import QApplication
app = QApplication([])
print("[AGV-GUI] Qt display preflight PASS", flush=True)
app.quit()
PY
  then
    fail "Qt QApplication tidak dapat terhubung ke display dalam 6 detik. Cek DISPLAY/XAUTHORITY/Qt xcb plugin."
  fi
fi

# rclpy check is diagnostic only and bounded. It must never prevent the window.
if command -v timeout >/dev/null 2>&1; then
  if timeout 4s "$PYTHON_BIN" -c 'import rclpy; print("[AGV-GUI] ROS diagnostic: rclpy import OK", flush=True)' ; then
    :
  else
    log "NOTICE: rclpy import gagal/timeout. GUI tetap dibuka OFFLINE; ROS bridge akan dicoba asynchronous."
  fi
fi

log "starting Qt window now"
# agv_gui.py knows stdout/stderr are already captured and therefore does not
# append the same messages to the startup log a second time.
export AGV_GUI_STDIO_TEE=1
exec "$PYTHON_BIN" -u "$GUI_ENTRY" "$@" \
  > >(tee -a "$STARTUP_LOG") \
  2> >(tee -a "$STARTUP_LOG" >&2)
