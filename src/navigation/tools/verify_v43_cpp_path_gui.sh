#!/usr/bin/env bash
set -uo pipefail
ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PASS=0
FAIL=0
pass(){ printf '[PASS] %s\n' "$1"; PASS=$((PASS+1)); }
fail(){ printf '[FAIL] %s\n' "$1"; FAIL=$((FAIL+1)); }
need_file(){ [[ -f "$ROOT/$1" ]] && pass "$1 present" || fail "$1 missing"; }
need_grep(){ local pat="$1" file="$2" msg="$3"; grep -Eq "$pat" "$ROOT/$file" 2>/dev/null && pass "$msg" || fail "$msg"; }
no_grep(){ local pat="$1" file="$2" msg="$3"; ! grep -Eq "$pat" "$ROOT/$file" 2>/dev/null && pass "$msg" || fail "$msg"; }

printf '=== V43 C++ GUI / GoalPose / Smac verifier ===\n'
for f in \
  src/agv_gui.cpp src/mapping_gui.cpp src/goal_pose_nav2_bridge.cpp \
  src/scan_self_filter.cpp src/runtime_validator.cpp \
  launch/gui.launch.py launch/map.launch.py launch/autonomous.launch.py launch/lidar.launch.py \
  config/nav2_ackermann.yaml config/lidar.yaml config/slam_toolbox.yaml CMakeLists.txt package.xml; do
  need_file "$f"
done

for f in scripts/agv_gui.py scripts/mapping_gui.py scripts/goal_pose_nav2_bridge.py scripts/scan_self_filter.py; do
  [[ ! -e "$ROOT/$f" ]] && pass "legacy production Python removed: $f" || fail "legacy production Python still exists: $f"
done
[[ ! -d "$ROOT/python/navigation_gui" ]] && pass 'legacy Python navigation_gui package removed' || fail 'legacy Python navigation_gui package still exists'

need_grep 'add_executable\(agv_gui_cpp[[:space:]]+src/agv_gui\.cpp\)' CMakeLists.txt 'CMake builds native main GUI'
need_grep 'add_executable\(mapping_gui_cpp[[:space:]]+src/mapping_gui\.cpp\)' CMakeLists.txt 'CMake builds native mapping GUI'
need_grep 'add_executable\(goal_pose_nav2_bridge[[:space:]]+src/goal_pose_nav2_bridge\.cpp\)' CMakeLists.txt 'CMake builds native GoalPose bridge'
need_grep 'add_executable\(scan_self_filter[[:space:]]+src/scan_self_filter\.cpp\)' CMakeLists.txt 'CMake builds native scan filter'
need_grep 'add_executable\(navigation_runtime_validator[[:space:]]+src/runtime_validator\.cpp\)' CMakeLists.txt 'CMake builds native live validator'

need_grep "executable='agv_gui_cpp'" launch/gui.launch.py 'GUI launch uses C++ executable'
need_grep "executable='mapping_gui_cpp'" launch/map.launch.py 'mapping launch uses C++ executable'
need_grep 'executable="goal_pose_nav2_bridge"' launch/autonomous.launch.py 'autonomous launch uses C++ GoalPose bridge'
need_grep "executable='scan_self_filter'" launch/lidar.launch.py 'LiDAR launch uses C++ filters'
no_grep 'agv_gui\.py|mapping_gui\.py|goal_pose_nav2_bridge\.py|scan_self_filter\.py' launch/gui.launch.py 'GUI launch has no legacy Python runtime entry'
no_grep 'goal_pose_nav2_bridge\.py' launch/autonomous.launch.py 'autonomous launch has no Python GoalPose bridge'
no_grep 'scan_self_filter\.py' launch/lidar.launch.py 'LiDAR launch has no Python scan filter'

need_grep 'nav2_msgs/action/compute_path_to_pose\.hpp' src/goal_pose_nav2_bridge.cpp 'bridge uses ComputePathToPose action'
need_grep 'create_client<ComputePath>' src/goal_pose_nav2_bridge.cpp 'bridge owns Smac path action client'
need_grep 'planner_id_' src/goal_pose_nav2_bridge.cpp 'bridge sets planner ID'
need_grep 'plan_pub_->publish\(path\)' src/goal_pose_nav2_bridge.cpp 'bridge publishes preview path'
need_grep 'create_client<Navigate>' src/goal_pose_nav2_bridge.cpp 'bridge sends NavigateToPose after preview'
need_grep 'plan_topic.*smac_plan' launch/autonomous.launch.py 'preview topic is /smac_plan'
need_grep 'planner_id.*GridBased' launch/autonomous.launch.py 'bridge explicitly selects GridBased planner'

need_grep 'plugin:[[:space:]]+nav2_smac_planner/SmacPlannerHybrid' config/nav2_ackermann.yaml 'GridBased is Smac Hybrid-A*'
need_grep 'motion_model_for_search:[[:space:]]+REEDS_SHEPP' config/nav2_ackermann.yaml 'Smac uses Reeds-Shepp for forward/reverse Ackermann search'
need_grep 'minimum_turning_radius:[[:space:]]+2\.0' config/nav2_ackermann.yaml 'Smac preserves physical 2.0 m minimum turning radius'
need_grep 'plugin:[[:space:]]+nav2_mppi_controller::MPPIController' config/nav2_ackermann.yaml 'controller is MPPI'
need_grep 'motion_model:[[:space:]]+Ackermann' config/nav2_ackermann.yaml 'MPPI motion model is Ackermann'
COUNT_INFLATION=$(grep -Ec 'inflation_radius:[[:space:]]+0\.45' "$ROOT/config/nav2_ackermann.yaml" || true)
[[ "$COUNT_INFLATION" -eq 2 ]] && pass 'global/local inflation radius both 0.45 m' || fail "expected two 0.45 inflation radii, got $COUNT_INFLATION"
COUNT_FOOTPRINT=$(grep -Fc "footprint: '[[0.65,0.40],[0.65,-0.40],[-0.65,-0.40],[-0.65,0.40]]'" "$ROOT/config/nav2_ackermann.yaml" || true)
[[ "$COUNT_FOOTPRINT" -eq 2 ]] && pass 'global/local full 1.30 x 0.80 m footprint preserved' || fail "full footprint mismatch ($COUNT_FOOTPRINT)"

need_grep 'strict_checksum:[[:space:]]+true' config/lidar.yaml 'package LiDAR strict checksum enabled'
need_grep 'declare_parameter\("strict_checksum",[[:space:]]*true\)' src/lidar_node.cpp 'LiDAR C++ default strict checksum enabled'
need_grep "DeclareLaunchArgument\('strict_checksum',[[:space:]]*default_value='true'\)" launch/lidar.launch.py 'launch defaults strict checksum ON'
need_grep "max_output_range': 5\.5" launch/lidar.launch.py 'navigation scan capped at 5.5 m'
need_grep 'previous_ranges_[[:space:]]*=[[:space:]]*out\.ranges' src/scan_self_filter.cpp 'temporal history uses filtered scan (no self-validating starburst)'
need_grep 'max_laser_range:[[:space:]]+5\.5' config/slam_toolbox.yaml 'SLAM consumes bounded LiDAR range'
need_grep 'laser_max_range:[[:space:]]+5\.5' config/nav2_ackermann.yaml 'AMCL range matches navigation scan cap'

need_grep '<depend>rclcpp_action</depend>' package.xml 'rclcpp_action dependency declared'
need_grep '<depend>nav2_msgs</depend>' package.xml 'nav2_msgs dependency declared'
need_grep '<depend>qtbase5-dev</depend>' package.xml 'Qt5 build dependency declared'
no_grep 'python3-pyqt5|python3-pyqtgraph' package.xml 'PyQt/PyQtGraph runtime dependency removed'

printf '\nV43 STATIC RESULT: %d PASS / %d FAIL\n' "$PASS" "$FAIL"
[[ "$FAIL" -eq 0 ]]
