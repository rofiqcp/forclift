#!/usr/bin/env bash
set -euo pipefail
NAV="${1:-$(cd "$(dirname "$0")/.." && pwd)}"
PASS=0; FAIL=0
ok(){ echo "[V46] PASS $*"; PASS=$((PASS+1)); }
bad(){ echo "[V46] FAIL $*" >&2; FAIL=$((FAIL+1)); }
need(){ local file="$1" pat="$2" msg="$3"; grep -q -- "$pat" "$file" && ok "$msg" || bad "$msg"; }
forbid(){ local file="$1" pat="$2" msg="$3"; if grep -q -- "$pat" "$file"; then bad "$msg"; else ok "$msg"; fi; }

GUI="$NAV/src/agv_gui.cpp"
GL="$NAV/launch/gui.launch.py"
AUTO="$NAV/launch/autonomous.launch.py"
MAP="$NAV/launch/mapping_runtime.launch.py"
RES="$NAV/tools/resolve_usb_roles.py"
PRE="$NAV/scripts/autonomous_sensor_preflight.sh"

for f in "$GUI" "$GL" "$AUTO" "$MAP" "$RES" "$PRE"; do [[ -f "$f" ]] && ok "exists $(basename "$f")" || bad "missing $f"; done
need "$GUI" 'background:#20252b' 'dark legacy GUI palette restored in C++'
need "$GUI" 'Autonomous Vehicle Interface' 'legacy branding present'
need "$GUI" 'Connection & System Overview' 'connection overview restored'
need "$GUI" 'Map Comparison' 'sidebar MAP pages restored'
need "$GUI" 'Smac Hybrid-A\*' 'sidebar Hybrid-A page restored'
need "$GUI" 'QTabWidget' 'Topics/TF tabs implemented in C++'
need "$GUI" 'Fork Alignment' 'full status-card set restored'
need "$GL" 'IncludeLaunchDescription' 'GUI directly includes autonomous runtime'
forbid "$GL" 'ExecuteProcess' 'no nested ros2-launch subprocess in GUI launcher'
need "$PRE" 'max_rounds=6' 'sensor preflight is bounded'
forbid "$PRE" '^while true' 'sensor preflight has no unbounded owner loop'
need "$RES" 'publish each resolved serial role immediately' 'resolver supports partial role handoff'
need "$AUTO" 'imu_transport_ready = ExecuteProcess' 'autonomous has independent IMU transport gate'
need "$AUTO" 'lidar_transport_ready = ExecuteProcess' 'autonomous has independent LiDAR transport gate'
need "$AUTO" 'start_imu_after_transport_ready' 'autonomous starts IMU independently'
need "$AUTO" 'start_lidar_after_transport_ready' 'autonomous starts LiDAR independently'
forbid "$AUTO" 'serial_transport_ready = ExecuteProcess' 'autonomous all-or-nothing serial gate removed'
need "$MAP" 'start_imu_after_transport_ready' 'mapping starts IMU independently'
need "$MAP" 'start_lidar_after_transport_ready' 'mapping starts LiDAR independently'
forbid "$MAP" 'serial_transport_ready = ExecuteProcess' 'mapping all-or-nothing serial gate removed'
need "$AUTO" 'lidar_strict_checksum", default_value="true"' 'autonomous strict LiDAR checksum enabled'
need "$PRE" 'NAV_ROOT=.*SCRIPT_DIR/\.\.' 'preflight source-tree recovery fallback is present'
need "$PRE" 'NAV_ROOT/tools/install_sensor_recovery\.sh' 'preflight can locate source tools installer'
need "$GUI" 'background:#20252b' 'native C++ GUI keeps requested dark theme'
need "$NAV/src/mapping_gui.cpp" 'static const QStringList extensions' 'mapping reset uses stable QStringList without GCC range-loop warning'
forbid "$NAV/src/mapping_gui.cpp" 'for (const QString ext :' 'temporary QString value range-loop removed'
need "$NAV/tools/validate_v46_sensor_live.sh" '/imu/status' 'live verifier checks IMU status stream'
need "$NAV/tools/validate_v46_sensor_live.sh" '/lidar/status' 'live verifier checks LiDAR status stream'
need "$NAV/tools/validate_v46_sensor_live.sh" 'check_nonempty_path' 'live verifier supports non-empty goal path validation'

printf '[V46] STATIC RESULT: %d PASS / %d FAIL\n' "$PASS" "$FAIL"
(( FAIL == 0 ))
