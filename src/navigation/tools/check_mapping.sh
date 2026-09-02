#!/usr/bin/env bash
# Master-aligned LiDAR mapping + IMU-yaw RobotModel health check.
set -u
PASS=0
FAIL=0
WARN=0
pass(){ echo "[PASS] $*"; PASS=$((PASS+1)); }
fail(){ echo "[FAIL] $*"; FAIL=$((FAIL+1)); }
warn(){ echo "[WARN] $*"; WARN=$((WARN+1)); }

check_topic_once(){
  local topic="$1" label="$2"
  if timeout 6s ros2 topic echo "$topic" --once >/tmp/master_map_topic.out 2>/tmp/master_map_topic.err; then
    pass "$label ($topic)"
  else
    fail "$label ($topic)"
    tail -n 2 /tmp/master_map_topic.err 2>/dev/null || true
  fi
}
check_tf(){
  local parent="$1" child="$2" label="$3"
  : >/tmp/master_map_tf.out
  timeout 6s ros2 run tf2_ros tf2_echo "$parent" "$child" >/tmp/master_map_tf.out 2>/tmp/master_map_tf.err || true
  if grep -q 'Translation:' /tmp/master_map_tf.out; then
    pass "$label ($parent -> $child)"
  else
    fail "$label ($parent -> $child)"
  fi
}
check_node(){
  local node="$1" label="$2"
  if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
    pass "$label ($node)"
  else
    fail "$label ($node)"
  fi
}
assert_node_absent(){
  local node="$1" label="$2"
  if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
    fail "$label unexpectedly running ($node)"
  else
    pass "$label not running"
  fi
}

echo '=== MASTER-ALIGNED MAPPING + IMU ROBOTMODEL HEALTH CHECK ==='
check_node /lidar_node 'LiDAR driver'
check_node /hector_slam_node 'Master LiDAR odometry'
check_node /slam_toolbox 'SLAM Toolbox'
if pgrep -af '[a]sync_slam_toolbox_node' >/tmp/master_map_slam_proc.out 2>/dev/null; then
  pass 'SLAM Toolbox executable is ASYNC (master mode)'
else
  fail 'async_slam_toolbox_node is not running; sync mode will lag behind /scan'
  pgrep -af 'slam_toolbox_node' 2>/dev/null || true
fi
check_node /data_imu_node 'IMU driver'
check_node /imu_visual_tf_node 'Anti-jitter IMU visual TF bridge'

# The old visual-fusion chain must not run in mapping mode anymore.
assert_node_absent /imu_ekf_preprocessor_node 'Old IMU visual preprocessor'
assert_node_absent /visual_lidar_yaw_guard_node 'Old LiDAR-yaw visual guard'
assert_node_absent /visual_ekf_filter_node 'Old visual EKF'
assert_node_absent /ekf_filter_node 'Autonomous EKF'

check_topic_once /scan 'LiDAR LaserScan'
check_topic_once /lidar/odom 'LiDAR scan-matching odometry'
check_topic_once /imu/data 'IMU AHRS stream used for RobotModel yaw'
check_topic_once /map 'SLAM occupancy map'
check_topic_once /joint_states 'Robot movable-joint states'

check_tf base_footprint lidar_link 'LiDAR mounting TF'
check_tf odom base_footprint 'EKF mapping TF'
check_tf map odom 'SLAM global TF'
check_tf odom visual_/base_footprint 'IMU-yaw RobotModel visual TF'
check_tf visual_/base_footprint visual_/base_link 'Prefixed RobotModel root TF'
check_tf visual_/base_link visual_/front_left_wheel_link 'Prefixed movable-link TF'

echo '--- expected rates ---'
echo '[target] /scan ~= 10 Hz'
timeout 7s ros2 topic hz /scan 2>/dev/null | tail -n 5 || true
echo '[target] /lidar/odom ~= 9-10 Hz'
timeout 7s ros2 topic hz /lidar/odom 2>/dev/null | tail -n 5 || true
echo '[target] /imu/data ~= 50 Hz'
timeout 5s ros2 topic hz /imu/data 2>/dev/null | tail -n 5 || true
echo '[target] /map responsive; ASYNC master config map_update_interval=0.1 s'
timeout 7s ros2 topic hz /map 2>/dev/null | tail -n 5 || true

echo '--- map validity ---'
: >/tmp/master_map_valid.out
timeout 6s ros2 topic echo /mapping/map_valid std_msgs/msg/Bool --once --qos-durability transient_local >/tmp/master_map_valid.out 2>/dev/null || true
if grep -q 'data: true' /tmp/master_map_valid.out; then
  pass 'OccupancyGrid has known/occupied cells'
else
  warn 'Map exists but map_monitor has not yet declared it valid'
fi

echo
echo "Result: PASS=${PASS} WARN=${WARN} FAIL=${FAIL}"
if [ "$FAIL" -ne 0 ]; then
  echo 'Mapping/RobotModel chain is NOT ready.'
  exit 1
fi
echo 'Master-aligned mapping + IMU-yaw RobotModel ready.'
echo 'Mapping ownership : Hector -> /lidar/odom only; EKF -> odom->base_footprint; slam_toolbox -> map->odom + /map.'
echo 'RobotModel visual : X/Y follows /lidar/odom only while chassis moves; stationary LiDAR wander is held. Yaw follows /imu/data only while gyro moves; stationary AHRS jitter is held.'
echo 'IMU acceleration is not integrated into X/Y.'
