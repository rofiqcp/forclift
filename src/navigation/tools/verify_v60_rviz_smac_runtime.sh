#!/usr/bin/env bash
set -Eeuo pipefail
SRC="${AGV_WS:-$HOME/ros}/src"
NAV="$SRC/navigation"
YOLO="$SRC/yolo_obstacle_detection_ros2"
fail(){ echo "[V60-VERIFY] FAIL: $*" >&2; exit 2; }
pass(){ echo "[V60-VERIFY] PASS: $*"; }
req(){ [[ -f "$1" ]] || fail "missing $1"; }

LAUNCH="$NAV/launch/autonomous.launch.py"
BRIDGE="$NAV/src/goal_pose_nav2_bridge.cpp"
RVIZGATE="$NAV/tools/rviz_robot_model_ready_gate.py"
MAPTF="$NAV/scripts/autonomous_map_tf_ready_gate.py"
RUNNER="$NAV/tools/run_v60.sh"
AUTOCHECK="$NAV/scripts/autonomous_runtime_check.py"
for f in "$LAUNCH" "$BRIDGE" "$RVIZGATE" "$MAPTF" "$RUNNER" "$AUTOCHECK"; do req "$f"; done

# 1) Navigation foundation must not be coupled to perception cleanup.
grep -q 'start_foundation_after_preflight = TimerAction(period=0.30, actions=\[' "$LAUNCH" || \
  fail 'foundation is not an independent TimerAction'
if grep -q 'start_foundation_after_preflight = _success_only_exit' "$LAUNCH"; then
  fail 'foundation is still exit-code gated by perception preflight'
fi
grep -q 'start_perception_after_cleanup = TimerAction(period=0.80, actions=\[' "$LAUNCH" || \
  fail 'perception branch is not independently timer-started'

# 2) RViz must always become available for diagnostics even before Initial Pose.
grep -A22 'rviz_model_gate = Node' "$LAUNCH" | grep -q '"fail_open": True' || \
  fail 'RViz RobotModel gate is not fail-open'
grep -A22 'rviz_model_gate = Node' "$LAUNCH" | grep -q '"timeout_s": 6.0' || \
  fail 'RViz bounded timeout missing'
grep -q 'declare_parameter("fail_open", True)' "$RVIZGATE" || \
  fail 'RViz gate source default is not fail-open'

# 3) Exactly-once lifecycle ownership for AMCL/local/aux; planner lifecycle only
# after the non-terminal map TF prerequisite.
[[ $(grep -c 'actions=\[localization_lifecycle\]' "$LAUNCH") -eq 1 ]] || \
  fail 'localization lifecycle manager must be executed exactly once'
[[ $(grep -c 'actions=\[lifecycle_navigation_local\]' "$LAUNCH") -eq 1 ]] || \
  fail 'local navigation lifecycle manager must be executed exactly once'
[[ $(grep -c 'actions=\[lifecycle_navigation_aux\]' "$LAUNCH") -eq 1 ]] || \
  fail 'aux navigation lifecycle manager must be executed exactly once'
[[ $(grep -c '\[lifecycle_navigation_global, TimerAction' "$LAUNCH") -eq 1 ]] || \
  fail 'global planner lifecycle manager must have exactly one activation path'
grep -A20 'map_tf_ready_gate = Node' "$LAUNCH" | grep -q '"fail_on_timeout": False' || \
  fail 'map TF gate can still terminally suppress PlannerServer'
grep -q "self.declare_parameter('fail_on_timeout', False)" "$MAPTF" || \
  fail 'map TF gate source lacks non-terminal mode'
grep -q 'STILL WAITING (non-terminal)' "$MAPTF" || \
  fail 'map TF gate does not retry after diagnostic timeout'

# 4) Goal bridge must retain Goal, accept AMCL QoS variants, use operator Initial
# Pose as a temporary planner start, and never throw the Goal away after a
# startup retry burst.
grep -q 'amcl_sub_transient_' "$BRIDGE" || fail 'AMCL transient-local subscription missing'
grep -q 'amcl_sub_fallback_' "$BRIDGE" || fail 'AMCL best-effort fallback subscription missing'
grep -q 'initial_pose_topic' "$BRIDGE" || fail 'Initial Pose fallback subscription missing'
grep -q 'WAIT_LOCALIZATION' "$BRIDGE" || fail 'missing localization wait state'
grep -q 'WAIT_REPLAN' "$BRIDGE" || fail 'Goal-retaining replan state missing'
if grep -A12 'plan_retries_ >= max_plan_retries_' "$BRIDGE" | grep -q 'pending_goal_\.reset'; then
  fail 'Goal is still discarded at Smac retry limit'
fi
grep -A28 'goal_pose_bridge = Node' "$LAUNCH" | grep -q '"initial_pose_topic": "/initialpose_safe"' || \
  fail 'launch does not configure Initial Pose fallback'
grep -A35 'goal_pose_bridge = Node' "$LAUNCH" | grep -q '"max_plan_retries": 20' || \
  fail 'robust planner retry budget missing'

# 5) Smac/MPPI production wiring must still be present.
grep -q 'nav2_smac_planner/SmacPlannerHybrid' "$NAV/config/nav2_ackermann.yaml" || \
  fail 'SmacPlannerHybrid plugin missing'
grep -q 'planner_id", "GridBased"' "$BRIDGE" || \
  fail 'Goal bridge default planner id is not GridBased'
grep -q '"plan_topic": "/smac_plan"' "$LAUNCH" || \
  fail '/smac_plan preview output missing'
grep -q '"compute_path_action_name": "/compute_path_to_pose"' "$LAUNCH" || \
  fail 'ComputePathToPose action wiring missing'
grep -q '"navigate_action_name": "/navigate_to_pose"' "$LAUNCH" || \
  fail 'NavigateToPose action wiring missing'
grep -q '/transformed_global_plan' "$AUTOCHECK" || fail 'MPPI transformed-plan acceptance missing'
grep -q '/trajectories' "$AUTOCHECK" || fail 'MPPI trajectories acceptance missing'

# 6) Source syntax. C++ is compiled by colcon on Jetson; here check balanced
# structural tokens and all Python/Bash files touched by V60.
python3 -m py_compile "$LAUNCH" "$RVIZGATE" "$MAPTF" "$AUTOCHECK"
bash -n "$RUNNER"
for f in "$SRC/RUN_AUTONOMOUS.sh" "$SRC/RUN_GUI.sh" "$SRC/RUN_MAPPING.sh" \
         "$SRC/RUN_CHECK.sh" "$SRC/RUN_ACCEPTANCE_V60.sh" \
         "$SRC/RUN_GOAL_ACCEPTANCE_V60.sh"; do
  req "$f"; bash -n "$f"
done
python3 - "$BRIDGE" <<'PY'
from pathlib import Path
import sys
s=Path(sys.argv[1]).read_text()
# Strip // comments and quoted strings only enough for a structural brace check.
# This is intentionally not a replacement for the Jetson colcon compile.
stack=[]
in_str=False
esc=False
for ch in s:
    if in_str:
        if esc: esc=False
        elif ch=='\\': esc=True
        elif ch=='"': in_str=False
        continue
    if ch=='"': in_str=True; continue
    if ch in '({[': stack.append(ch)
    elif ch in ')}]':
        if not stack: raise SystemExit('unbalanced closing token')
        o=stack.pop()
        if (o,ch) not in [('(',')'),('{','}'),('[',']')]:
            raise SystemExit(f'mismatched {o} {ch}')
if stack: raise SystemExit('unbalanced opening tokens')
print('[V60-VERIFY] C++ structural token check PASS')
PY

pass 'independent foundation + bounded RViz + non-terminal map TF + resilient Smac Goal pipeline verified'
