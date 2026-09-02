#!/usr/bin/env bash
set -Eeuo pipefail
SRC="${AGV_WS:-$HOME/ros}/src"
NAV="$SRC/navigation"
fail(){ echo "[V58-VERIFY] FAIL: $*" >&2; exit 2; }
pass(){ echo "[V58-VERIFY] PASS: $*"; }
req(){ [[ -f "$1" ]] || fail "missing $1"; }

LAUNCH="$NAV/launch/autonomous.launch.py"
PAGES="$NAV/python/navigation_gui/pages.py"
CANVAS="$NAV/python/navigation_gui/map_canvas.py"
BRIDGE="$NAV/python/navigation_gui/ros_bridge.py"
MAIN="$NAV/python/navigation_gui/main_window.py"
CHECKER="$NAV/scripts/autonomous_runtime_check.py"
PARAMS="$NAV/config/nav2_ackermann.yaml"
GOAL="$NAV/src/goal_pose_nav2_bridge.cpp"
for f in "$LAUNCH" "$PAGES" "$CANVAS" "$BRIDGE" "$MAIN" "$CHECKER" "$PARAMS" "$GOAL"; do req "$f"; done

grep -q 'DeclareLaunchArgument("camera_device", default_value="auto")' "$LAUNCH" || fail 'camera auto-discovery not enabled'
grep -q 'start_pre_amcl_after_sensor_topics = _success_only_exit' "$LAUNCH" || fail 'PRE-AMCL still not directly chained from sensor topics'
grep -q 'start_perception_after_cleanup = _success_only_exit' "$LAUNCH" || fail 'perception independent cleanup branch missing'
if grep -q 'start_pre_amcl_after_post_perception_sensors' "$LAUNCH"; then fail 'old camera->PRE-AMCL deadlock handler still active'; fi
if grep -q 'start_perception_after_camera_resolver,' "$LAUNCH"; then fail 'old camera resolver gate still in active LaunchDescription'; fi

grep -q 'def draw_static_inflation' "$CANVAS" || fail 'saved-map inflation renderer missing'
grep -q 'Saved-map Costmap + Inflation' "$PAGES" || fail 'saved-map inflation layer toggle missing'
grep -q 'inflation_radius=0.45' "$PAGES" || fail 'GUI inflation preview not aligned with Nav2 0.45 m'
[[ "$(grep -Ec 'inflation_radius:[[:space:]]+0\.45' "$PARAMS")" -eq 2 ]] || fail 'global/local Nav2 inflation radius mismatch'
[[ "$(grep -Ec 'cost_scaling_factor:[[:space:]]+12\.0' "$PARAMS")" -eq 2 ]] || fail 'global/local cost scaling mismatch'

grep -q '"/amcl_pose", self._amcl_cb, transient' "$BRIDGE" || fail 'latched AMCL GUI subscription missing'
grep -q '"/amcl_pose", self._amcl_cb, best_effort' "$BRIDGE" || fail 'AMCL compatibility fallback missing'
grep -q '"/transformed_global_plan", self._path_cb("/transformed_global_plan"), best_effort' "$BRIDGE" || fail 'MPPI transformed-plan adaptive QoS missing'
grep -q '"/trajectories", self._trajectories_cb, best_effort' "$BRIDGE" || fail 'MPPI trajectory adaptive QoS missing'
grep -q '/obstacle_detection/performance' "$BRIDGE" || fail 'perception performance telemetry missing'
grep -q 'fallback_topics=\["/camera/color/image_raw"\]' "$MAIN" || fail 'perception raw-camera fallback missing'

grep -q 'plan_topic.*"/smac_plan"' "$GOAL" || fail 'Smac preview publisher missing'
grep -q 'compute_path_action_name.*"/compute_path_to_pose"' "$GOAL" || fail 'ComputePathToPose bridge missing'
grep -q "'/smac_plan': 0" "$CHECKER" || fail 'live Smac acceptance check missing'
grep -q "ACTION /compute_path_to_pose" "$CHECKER" || fail 'live planner action acceptance check missing'

python3 -m py_compile "$LAUNCH" "$PAGES" "$CANVAS" "$BRIDGE" "$MAIN" "$CHECKER"
bash -n "$NAV/tools/run_v58.sh"
pass 'launch graph, AMCL, perception, saved-map inflation, Smac/MPPI GUI QoS and acceptance checker'
