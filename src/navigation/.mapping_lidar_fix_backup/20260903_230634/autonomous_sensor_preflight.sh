#!/usr/bin/env bash
# Autonomous sensor ownership preflight.
# The physical IMU/LiDAR devices are exclusive resources. Camera/perception ownership
# is handled separately by perception_preflight.sh so serial recovery can never block images.
# A stale mapping runtime or an older autonomous run can keep TIOCEXCL/flock ownership even
# though a new RViz window opens normally. Clear only sensor/mapping-runtime
# owners here; do not touch the new launch's map_server, AMCL, EKF, RSP or RViz.
set -u

echo "[AUTONOMOUS-SENSOR-PREFLIGHT] clearing stale sensor owners"

# V24 STALE-LAUNCH / ZOMBIE FIX ----------------------------------------------
# This script is used by autonomous.launch.py and mapping_runtime.launch.py and
# may also run under gui.launch.py. Repeated GUI tests can leave an OLD launch
# parent alive. That parent may respawn lidar_node/imu_node immediately after we
# kill them, making this preflight wait forever while the LiDAR motor still spins.
#
# Build the complete CURRENT ancestor set once, then stop only OLD launch
# parents which are not ancestors of this preflight process. This protects the
# active gui.launch.py/autonomous.launch.py while cleaning previous sessions.
ancestors=" $$ "
p="$PPID"
while [[ -n "$p" && "$p" =~ ^[0-9]+$ && "$p" -gt 1 ]]; do
  ancestors+="$p "
  p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
done

is_current_ancestor() {
  local pid="$1"
  [[ " $ancestors " == *" $pid "* ]]
}

is_zombie() {
  local pid="$1" stat
  stat=$(ps -o stat= -p "$pid" 2>/dev/null | awk '{print $1}')
  [[ "$stat" == Z* ]]
}

stop_old_launch_parents() {
  local launch_name oldpid
  for launch_name in \
      mapping_runtime.launch.py \
      map.launch.py \
      autonomous.launch.py \
      gui.launch.py \
      allsystem.launch.py \
      lidar.launch.py \
      imu.launch.py; do
    while read -r oldpid; do
      [[ -z "$oldpid" ]] && continue
      if is_current_ancestor "$oldpid"; then
        continue
      fi
      if is_zombie "$oldpid"; then
        echo "[AUTONOMOUS-SENSOR-PREFLIGHT] ignoring zombie old launch pid=$oldpid ($launch_name)"
        continue
      fi
      echo "[AUTONOMOUS-SENSOR-PREFLIGHT] stopping OLD launch pid=$oldpid ($launch_name)"
      kill -TERM "$oldpid" 2>/dev/null || true
    done < <(pgrep -f "ros2.*launch.*navigation.*${launch_name}" 2>/dev/null || true)
  done
}

stop_old_launch_parents
sleep 1.2

# Clear only stale recovery helpers from a previous mapping session before the
# new resolver/driver lifecycle starts.  This is intentionally narrow: only the
# root helper command is targeted, not unrelated sudo/python processes.
if pgrep -f '^sudo -n /usr/local/sbin/agv-sensor-recover' >/dev/null 2>&1 || \
   pgrep -f '^/usr/local/sbin/agv-sensor-recover' >/dev/null 2>&1; then
  echo "[AUTONOMOUS-SENSOR-PREFLIGHT] stopping stale agv-sensor-recover from previous session"
  pkill -TERM -f '^sudo -n /usr/local/sbin/agv-sensor-recover' 2>/dev/null || true
  pkill -TERM -f '^/usr/local/sbin/agv-sensor-recover' 2>/dev/null || true
  sleep 0.8
fi

# Graceful first.  These executable patterns are intentionally narrow.
patterns=(
  # Mapping / sensors
  "/navigation/lib/navigation/imu_node"
  "/navigation/lib/navigation/lidar_node"
  "/navigation/lib/navigation/hector_slam_node"
  "async_slam_toolbox_node"
  "sync_slam_toolbox_node"
  "/navigation/lib/navigation/imu_visual_tf_node"
  "/navigation/lib/navigation/map_monitor_node"

  # Stale localization / TF owners from an older autonomous run. V5 starts
  # this preflight before creating any new child, so these patterns cannot hit
  # the new run. Clearing them prevents duplicate /map and map->odom owners.
  "/nav2_map_server/map_server"
  "/nav2_amcl/amcl"
  "/robot_localization/ekf_node"
  "/robot_state_publisher/robot_state_publisher"
  "/joint_state_publisher/joint_state_publisher"

  # Stale Nav2 managed servers / lifecycle managers.
  "/nav2_planner/planner_server"
  "/nav2_controller/controller_server"
  "/nav2_behaviors/behavior_server"
  "/nav2_bt_navigator/bt_navigator"
  "/nav2_velocity_smoother/velocity_smoother"
  "/nav2_collision_monitor/collision_monitor"
  "/nav2_lifecycle_manager/lifecycle_manager"
  "/esc/lib/esc/esc_driver"
  "/esc/lib/esc/esc_command_mux"
  "/esc/lib/esc/winch_serial_node"
  "/esc/lib/esc/ackermann_controller_server"
  "/navigation/lib/navigation/agv_web_gui"
  "/navigation/lib/navigation/goal_pose_nav2_bridge"
  "/navigation/lib/navigation/autonomous_sensor_cmd_guard.py"
)

for pat in "${patterns[@]}"; do
  pkill -TERM -f -- "$pat" 2>/dev/null || true
done
sleep 1.0

# Force-kill only sensor/mapping-runtime processes that survived SIGTERM.
for pat in "${patterns[@]}"; do
  pkill -KILL -f -- "$pat" 2>/dev/null || true
done
sleep 0.3

# Aliases are cheap state; remove them only after old owners have been stopped.
rm -f /tmp/agv_devices/imu /tmp/agv_devices/lidar /tmp/agv_devices/camera
rm -rf /tmp/agv_autonomous_roles
mkdir -p /tmp/agv_devices /tmp/agv_autonomous_roles

# V42: self-bootstrapping, non-destructive CP210x recovery.
# ---------------------------------------------------------------------------
# IMPORTANT:
#   * Healthy serial transports are NEVER rebound/reset on every launch.
#   * Privileged recovery is used only when fewer than two CP210x tty endpoints
#     exist.
#   * If the permanent helper has not been installed yet, the desktop launch may
#     bootstrap it through sudo-askpass/pkexec.  This removes the old V41 single
#     point of failure where missing helper => exit 42 => whole mapping shutdown.
#   * If authorization is cancelled or hardware is physically absent, preflight
#     does not kill the complete ROS graph.  The role resolver keeps retrying the
#     installed helper and waits for the sensors to enumerate.
RECOVER_HELPER=/usr/local/sbin/agv-sensor-recover
REQUIRED_RECOVERY_VERSION="AGV-SERIAL-RECOVERY-V55"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NAV_ROOT="$(cd "$SCRIPT_DIR/.." 2>/dev/null && pwd || true)"
# Installed PROGRAMS live together in lib/navigation.  When this script is run
# directly from the source tree, the recovery utilities live in ../tools.
RECOVER_FALLBACK="$SCRIPT_DIR/cp210x_recover.py"
INSTALLER="$SCRIPT_DIR/install_sensor_recovery.sh"
if [[ ! -f "$RECOVER_FALLBACK" && -f "$NAV_ROOT/tools/cp210x_recover.py" ]]; then
  RECOVER_FALLBACK="$NAV_ROOT/tools/cp210x_recover.py"
fi
if [[ ! -f "$INSTALLER" && -f "$NAV_ROOT/tools/install_sensor_recovery.sh" ]]; then
  INSTALLER="$NAV_ROOT/tools/install_sensor_recovery.sh"
fi
AGV_SENSOR_USER="${SUDO_USER:-${USER:-otomasi2}}"

settle_udev() {
  if command -v udevadm >/dev/null 2>&1; then
    udevadm settle --timeout=6 >/dev/null 2>&1 || true
  fi
}

tty_inventory() {
  ls /dev/ttyUSB* 2>/dev/null | tr '\n' ' ' || true
}

cp210x_tty_count() {
  local tty driver n=0
  for tty in /sys/class/tty/ttyUSB*; do
    [[ -e "$tty" ]] || continue
    driver=$(basename "$(readlink -f "$tty/device/driver" 2>/dev/null || true)")
    if [[ "$driver" == "cp210x" && -e "/dev/$(basename "$tty")" ]]; then
      n=$((n + 1))
    fi
  done
  printf '%s' "$n"
}

cp210x_usb_inventory() {
  local item base vid pid out=""
  for item in /sys/bus/usb/devices/*/idVendor; do
    [[ -f "$item" ]] || continue
    base="${item%/idVendor}"
    vid=$(cat "$item" 2>/dev/null || true)
    pid=$(cat "$base/idProduct" 2>/dev/null || true)
    if [[ "$vid" == "10c4" && "$pid" == "ea60" ]]; then
      out+="$(basename "$base") "
    fi
  done
  printf '%s' "$out"
}

helper_usable() {
  [[ -x "$RECOVER_HELPER" ]] && sudo -n "$RECOVER_HELPER" --help >/dev/null 2>&1
}

helper_current() {
  [[ -x "$RECOVER_HELPER" ]] || return 1
  local version
  version=$(sudo -n "$RECOVER_HELPER" --version 2>/dev/null || true)
  [[ "$version" == "$REQUIRED_RECOVERY_VERSION" ]]
}

bootstrap_helper() {
  [[ -x "$INSTALLER" ]] || return 1

  # 1) Existing no-password sudo policy / cached root path.
  if sudo -n /usr/bin/env AGV_TARGET_USER="$AGV_SENSOR_USER" bash "$INSTALLER" --install-only >/dev/null 2>&1; then
    echo "[AUTONOMOUS-SENSOR-PREFLIGHT] recovery helper bootstrapped with sudo policy"
    return 0
  fi

  # 2) Desktop askpass.  The Jetson logs show SUDO_ASKPASS is configured.
  if [[ -n "${SUDO_ASKPASS:-}" && -x "${SUDO_ASKPASS:-/nonexistent}" ]]; then
    echo "[AUTONOMOUS-SENSOR-PREFLIGHT] requesting one-time sensor setup authorization"
    if AGV_TARGET_USER="$AGV_SENSOR_USER" sudo -A /usr/bin/env \
        AGV_TARGET_USER="$AGV_SENSOR_USER" bash "$INSTALLER" --install-only >/dev/null 2>&1; then
      return 0
    fi
  fi

  # 3) GNOME/desktop PolicyKit fallback.  This displays the normal system auth dialog.
  if command -v pkexec >/dev/null 2>&1 && [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo "[AUTONOMOUS-SENSOR-PREFLIGHT] requesting PolicyKit authorization for sensor recovery setup"
    if pkexec /usr/bin/env AGV_TARGET_USER="$AGV_SENSOR_USER" bash "$INSTALLER" --install-only >/dev/null 2>&1; then
      return 0
    fi
  fi
  return 1
}

run_recovery_once() {
  # Bounded recovery: never let a hung privileged helper block parent launch.
  if helper_usable; then
    timeout 20s sudo -n "$RECOVER_HELPER" both
    return $?
  fi
  return 1
}

role_topology_ready() {
  local role="$1" device iface tty driver auth
  case "$role" in
    imu) device="1-2.1.1"; iface="1-2.1.1:1.0" ;;
    lidar) device="1-2.1.4"; iface="1-2.1.4:1.0" ;;
    *) return 1 ;;
  esac
  [[ -d "/sys/bus/usb/devices/$device" && -d "/sys/bus/usb/devices/$iface" ]] || return 1
  [[ "$(cat "/sys/bus/usb/devices/$device/idVendor" 2>/dev/null)" == "10c4" ]] || return 1
  [[ "$(cat "/sys/bus/usb/devices/$device/idProduct" 2>/dev/null)" == "ea60" ]] || return 1
  auth=$(cat "/sys/bus/usb/devices/$device/authorized" 2>/dev/null || true)
  [[ "$auth" == "1" ]] || return 1
  driver=$(basename "$(readlink -f "/sys/bus/usb/devices/$iface/driver" 2>/dev/null || true)")
  [[ "$driver" == "cp210x" ]] || return 1
  for tty in /sys/class/tty/ttyUSB*; do
    [[ -e "$tty" ]] || continue
    # /sys/class/tty/ttyUSBX/device resolves to .../<iface>/ttyUSBX.
    # Compare the PARENT directory with the expected USB interface, not the
    # tty leaf name itself; the old comparison always returned false and
    # triggered destructive CP210x recovery on healthy devices.
    tty_iface=$(basename "$(dirname "$(readlink -f "$tty/device" 2>/dev/null || true)")")
    [[ "$tty_iface" == "$iface" ]] || continue
    [[ -e "/dev/$(basename "$tty")" ]] || continue
    return 0
  done
  return 1
}

both_role_topologies_ready() {
  role_topology_ready imu && role_topology_ready lidar
}

settle_udev

# V53: NON-INVASIVE preflight.  Verify the two CP210x sensor bridges strictly by
# physical USB topology (device 1-2.1.1 and 1-2.1.4, both cp210x-bound,
# authorized, with their ttyUSB endpoints present and correctly wired).  This
# does NOT open()/close() the serial ports, so it can never induce EIO on a
# healthy bridge.  Destructive recovery is deferred to the resolver and to the
# real sensor drivers only when they themselves receive EIO/ENODEV/ENXIO.
if ! helper_current; then
  echo "[AUTONOMOUS-SENSOR-PREFLIGHT] recovery helper is missing/stale; attempting V54 upgrade"
  bootstrap_helper || true
fi
if both_role_topologies_ready; then
  echo "[AUTONOMOUS-SENSOR-PREFLIGHT] IMU topology 1-2.1.1 -> cp210x tty endpoint READY"
  echo "[AUTONOMOUS-SENSOR-PREFLIGHT] LiDAR topology 1-2.1.4 -> cp210x tty endpoint READY"
  echo "[AUTONOMOUS-SENSOR-PREFLIGHT] two CP210x topology endpoints ready; destructive recovery skipped"
else
  if helper_current; then
    echo "[AUTONOMOUS-SENSOR-PREFLIGHT] sensor topology incomplete; running bounded CP210x recovery"
    run_recovery_once || true
    settle_udev
  else
    echo "[AUTONOMOUS-SENSOR-PREFLIGHT] recovery helper V54 not installed; sensors will retry via resolver"
  fi
fi

initial_cp210x=$(cp210x_tty_count)
echo "[AUTONOMOUS-SENSOR-PREFLIGHT] transport inventory: cp210x_tty=${initial_cp210x} usb=$(cp210x_usb_inventory) dev=$(tty_inventory)"

# Topology path: do nothing destructive. Rebinding a visible CP210x on every
# launch can itself make the device vanish from the USB tree. Real stream health
# is proven only by imu_node/lidar_node after exclusive open.
if both_role_topologies_ready; then
  echo "[AUTONOMOUS-SENSOR-PREFLIGHT] topology endpoints present; destructive recovery skipped"
else
  if ! helper_usable; then
    bootstrap_helper || true
  fi

  if helper_usable; then
    echo "[AUTONOMOUS-SENSOR-PREFLIGHT] recovering missing CP210x transports"
    run_recovery_once || true
    settle_udev
  fi

  # Give kernel/udev a short stable window.  The resolver also retries recovery
  # later, so this stage never turns a sensor enumeration problem into a total
  # ROS graph shutdown.
  for _ in $(seq 1 20); do
    current=$(cp210x_tty_count)
    (( current >= 2 )) && break
    sleep 0.25
  done
fi

final_cp210x=$(cp210x_tty_count)
echo "[AUTONOMOUS-SENSOR-PREFLIGHT] final transport inventory: cp210x_tty=${final_cp210x} usb=$(cp210x_usb_inventory) dev=$(tty_inventory)"
if (( final_cp210x >= 2 )); then
  echo "[AUTONOMOUS-SENSOR-PREFLIGHT] IMU/LiDAR kernel transports ready"
else
  echo "[AUTONOMOUS-SENSOR-PREFLIGHT] sensor transports pending; role resolver will keep recovery/re-enumeration active"
fi

# Best-effort runtime-PM lock for currently enumerated CP210x devices.  This is
# intentionally non-destructive and safe when the user has no sysfs write access.
for v in /sys/bus/usb/devices/*/idVendor; do
  [[ -f "$v" ]] || continue
  usbbase="${v%/idVendor}"
  [[ "$(cat "$v" 2>/dev/null)" == "10c4" ]] || continue
  [[ "$(cat "$usbbase/idProduct" 2>/dev/null)" == "ea60" ]] || continue
  base="$usbbase/power"
  if [[ -d "$base" ]]; then
    if [[ -w "$base/control" ]]; then echo on > "$base/control" 2>/dev/null || true; fi
    if [[ -w "$base/autosuspend_delay_ms" ]]; then echo -1 > "$base/autosuspend_delay_ms" 2>/dev/null || true; fi
  fi
done
# V46: bounded stale-owner cleanup.  A hardware cleanup helper is not allowed to
# gate the whole ROS foundation forever.  Make a few aggressive passes, then let
# the serial-role resolver + kernel flock arbitration handle any late release.
# This guarantees Map Server / EKF / Nav2 diagnostics still start even if a USB
# adapter or an old process takes longer than expected to disappear.
max_rounds=6
for wait_round in $(seq 1 "$max_rounds"); do
  left=0
  blockers=()
  for pat in "${patterns[@]}"; do
    while read -r pid; do
      [[ -z "$pid" ]] && continue
      if is_zombie "$pid" || is_current_ancestor "$pid"; then
        continue
      fi
      left=1
      blockers+=("$pid:$pat")
      kill -TERM "$pid" 2>/dev/null || true
    done < <(pgrep -f -- "$pat" 2>/dev/null || true)
  done

  if [[ $left -eq 0 ]]; then
    echo "[AUTONOMOUS-SENSOR-PREFLIGHT] stale sensor ownership clean"
    exit 0
  fi

  sleep 0.35
  for entry in "${blockers[@]}"; do
    pid="${entry%%:*}"
    if [[ -n "$pid" ]] && ! is_zombie "$pid" && ! is_current_ancestor "$pid" && kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  stop_old_launch_parents
  sleep 0.25
done

echo "[AUTONOMOUS-SENSOR-PREFLIGHT] bounded cleanup complete; serial arbiter will handle any late owner release"
exit 0
