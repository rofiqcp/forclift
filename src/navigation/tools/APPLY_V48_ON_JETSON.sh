#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
WS="${AGV_WS:-$DEFAULT_WS}"
NAV="$WS/src/navigation"
fail(){ echo "[V48-BUILD] FAIL: $*" >&2; exit 2; }
[[ -d "$NAV" ]] || fail "navigation source not found: $NAV"
[[ -f /opt/ros/humble/setup.bash ]] || fail '/opt/ros/humble/setup.bash missing'

# Re-enter once with all stale ROS overlay variables removed. This happens
# BEFORE colcon starts, so the four AMENT/CMAKE prefix warnings cannot occur.
if [[ "${V48_SANITIZED_ENV:-0}" != 1 ]]; then
  exec env \
    -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    V48_SANITIZED_ENV=1 AGV_WS="$WS" \
    bash "$0" "$@"
fi

# Preserve CUDA/non-ROS library paths but remove stale copies of this workspace.
prune_path_var(){
  local name="$1" value="${!1-}" out="" part
  IFS=':' read -ra parts <<< "$value"
  for part in "${parts[@]}"; do
    [[ -z "$part" ]] && continue
    case "$part" in
      "$WS/install"|"$WS/install/"*|"$WS/build"|"$WS/build/"*) continue ;;
    esac
    out="${out:+$out:}$part"
  done
  export "$name=$out"
}
prune_path_var LD_LIBRARY_PATH
prune_path_var PATH
source /opt/ros/humble/setup.bash

# Build dependencies used by native Qt GUI.
if ! pkg-config --exists Qt5Widgets 2>/dev/null; then
  sudo apt-get update
  sudo apt-get install -y qtbase5-dev
fi

cd "$WS"
echo '[V48-BUILD] removing old build/install/log generated from dirty overlays'
rm -rf build install log
mkdir -p log maps config/runtime

if command -v rosdep >/dev/null 2>&1; then
  rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
fi

echo '[V48-BUILD] clean colcon build'
colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# Never source install/setup.bash here; it is allowed to chain recorded parents.
source "$WS/install/local_setup.bash"
export AGV_WS="$WS"
export AGV_RUNTIME_CONFIG_ROOT="$WS/config/runtime"

# Strong package provenance check.
for pkg in esc yolo_obstacle_detection_ros2 navigation; do
  got="$(ros2 pkg prefix "$pkg" 2>/dev/null || true)"
  expected="$WS/install/$pkg"
  [[ "$got" == "$expected" ]] || fail "$pkg resolves to '$got', expected '$expected'"
  echo "[V48-BUILD] PASS package $pkg -> $got"
done

check_exe(){
  local pkg="$1" exe="$2"
  ros2 pkg executables "$pkg" 2>/dev/null | awk '{print $2}' | grep -Fxq "$exe" || fail "missing executable $pkg/$exe"
  echo "[V48-BUILD] PASS executable $pkg/$exe"
}
for exe in agv_gui_cpp mapping_gui_cpp goal_pose_nav2_bridge scan_self_filter navigation_runtime_validator imu_node lidar_node hector_slam_node imu_visual_tf_node map_monitor_node allsystem_gate; do check_exe navigation "$exe"; done
for exe in run_gui run_mapping run_autonomous; do check_exe navigation "$exe"; done
for exe in esc_driver esc_command_mux esc_keyboard_teleop winch_serial_node ackermann_controller_server; do check_exe esc "$exe"; done
for exe in astra_rgb_v4l2_node obstacle_detector_node hole_block_alignment_node.py; do check_exe yolo_obstacle_detection_ros2 "$exe"; done

# Catch the exact class of runtime error found in V47 (undefined globals in launch files).
python3 - "$WS/src" <<'PY'
import ast, builtins, pathlib, symtable, sys
root=pathlib.Path(sys.argv[1]); builtin=set(dir(builtins)); failures=[]
for p in root.rglob('*.launch.py'):
    src=p.read_text(encoding='utf-8',errors='ignore')
    ast.parse(src,filename=str(p))
    st=symtable.symtable(src,str(p),'exec')
    module_defined={s.get_name() for s in st.get_symbols() if s.is_imported() or s.is_assigned() or s.is_namespace() or s.is_parameter()}
    refs=set()
    def walk(t):
        for s in t.get_symbols():
            if s.is_referenced() and s.is_global(): refs.add(s.get_name())
        for c in t.get_children(): walk(c)
    walk(st)
    undef=sorted(x for x in refs if x not in module_defined and x not in builtin and x not in {'__file__','__name__','__package__','__doc__'})
    if undef: failures.append((p,undef))
if failures:
    for p,n in failures: print(f'[V48-BUILD] undefined launch global: {p}: {n}',file=sys.stderr)
    raise SystemExit(2)
print('[V48-BUILD] PASS launch undefined-global scan')
PY

for spec in 'navigation gui.launch.py' 'navigation map.launch.py' 'navigation autonomous.launch.py'; do
  set -- $spec
  ros2 launch "$1" "$2" --show-args >/dev/null || fail "launch parse failed $1/$2"
  echo "[V48-BUILD] PASS launch parse $1/$2"
done

# Linker check for native production runtime.
for exe in agv_gui_cpp mapping_gui_cpp goal_pose_nav2_bridge scan_self_filter; do
  bin="$WS/install/navigation/lib/navigation/$exe"
  [[ -x "$bin" ]] || fail "not executable: $bin"
  if ldd "$bin" 2>/dev/null | grep -q 'not found'; then
    ldd "$bin" | grep 'not found' >&2 || true
    fail "unresolved shared library for $exe"
  fi
  echo "[V48-BUILD] PASS ldd $exe"
done

# Strip ANSI before checking colcon's own warning/error records.
python3 - "$WS/log" <<'PY'
import pathlib,re,sys
root=pathlib.Path(sys.argv[1]); bad=[]
for p in root.rglob('logger_all.log'):
    text=re.sub(r'\x1b\[[0-9;]*m','',p.read_text(errors='ignore'))
    for line in text.splitlines():
        if ' WARNING ' in line or ' ERROR ' in line:
            bad.append((p,line))
for p,line in bad:
    print(f'[V48-BUILD] {p}: {line}',file=sys.stderr)
if bad: raise SystemExit(2)
print('[V48-BUILD] PASS colcon logger: no WARNING / ERROR records')
PY

echo '[V48-BUILD] COMPLETE'
echo 'Run GUI:        bash src/RUN_GUI.sh'
echo 'Run Mapping:    bash src/RUN_MAPPING.sh'
echo 'Run Autonomous: bash src/RUN_AUTONOMOUS.sh'
