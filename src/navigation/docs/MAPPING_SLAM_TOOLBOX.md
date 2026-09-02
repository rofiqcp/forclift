# AGV Mapping with SLAM Toolbox

Runtime chain:

`ESC /esc/odom + IMU /imu/data -> robot_localization EKF -> odom->base_footprint`

`LiDAR /scan + TF base_footprint->lidar_link + odom->base_footprint -> slam_toolbox -> map->odom + /map`

## Launch

```bash
cd /home/otomasi2/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch navigation map.launch.py
```

The mapping launch uses protocol-aware IMU auto-detection, delays LiDAR probing to avoid serial-port races, and starts SLAM after the local odometry/TF chain has had time to initialize.

The current YDLIDAR parser is configured for the T-mini Plus-compatible intensity packet stream: each point is `I(1 byte) + D(2 bytes)` and packet checksum is validated using the documented YDLIDAR XOR layout. Override only if the physical LiDAR really uses a non-intensity stream:

```bash
ros2 launch navigation map.launch.py lidar_intensity:=false
```

Checksum is **diagnostic-only by default** for this T-mini Plus deployment. The
driver still rejects malformed AA55 framing, invalid sample count, invalid angle
check bits/ranges, and short sample blocks; it also computes and counts checksum
failures. This avoids the previous failure mode where every structurally valid
packet was discarded and `/scan` never existed. If the exact LiDAR firmware has
been verified to match the documented checksum byte-for-byte, strict mode can be
enabled with `lidar_strict_checksum:=true`.

## Runtime health check

In another terminal:

```bash
cd /home/otomasi2/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/navigation/tools/check_mapping.sh
```

The required mapping chain is healthy only when `/robot_description`, `/joint_states`,
`/imu/data`, `/esc/odom`, `/odometry/filtered`, `/scan`, `odom->base_footprint`,
`base_footprint->imu_link`, `base_footprint->lidar_link`, `/map`, and `map->odom` are present.

## ESC steering safety gate

The supplied Ackermann profile intentionally has `steering_calibrated: false` and `require_steering_calibration_for_motion: true`. Do not set `steering_calibrated: true` until `steering_zero_ticks` and `steering_ticks_per_rad` have been measured on the physical steering mechanism. The sensor/SLAM chain can be validated while stationary; commanded Ackermann motion must remain blocked until calibration is valid.

## Save map

```bash
mkdir -p /home/otomasi2/ros/maps
bash src/navigation/tools/save_map.sh /home/otomasi2/ros/maps/kampus
```

This requests SLAM Toolbox to save the occupancy map using the supplied absolute name.


## YOLO model directory

The detector defaults to:

```text
/home/otomasi2/ros/models/yolov8n_agv_forklift.onnx
```

`yolo_model:=...` remains available as an override.

## Expected RViz layers

`mapping.rviz` is preconfigured to show:

- `Map` from `/map` (SLAM Toolbox)
- `LaserScan` from `/scan`
- `RobotModel` from `robot_description`
- `TF` for the full frame tree
- `IMU_Frame` axes at `imu_link`
- `LiDAR_Frame` axes at `lidar_link`
- `ESC_Odometry` arrows from `/esc/odom`
- `EKF_Odometry` arrows from `/odometry/filtered`
- SLAM pose and pose-graph markers

The ESC driver publishes raw physical joint feedback on `/esc/joint_states` during
mapping. `joint_state_publisher` merges those measurements with safe zero/default
positions for movable URDF joints that have not reported yet, then publishes the
canonical `/joint_states` consumed by `robot_state_publisher`.

The LiDAR driver has a scan-stream watchdog. If AA55 serial transport remains
connected but no complete physical revolution is published for longer than the
recovery timeout, the reader/scan command is restarted without replaying stale
LaserScan data.
