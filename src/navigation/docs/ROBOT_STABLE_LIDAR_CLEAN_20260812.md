# RobotModel stationary lock + LiDAR speckle cleanup — 2026-08-12

## Runtime evidence

`log(20260812-122951)` reached 1738 processed scans with 1004 rejected LiDAR-odometry jumps. The RobotModel visual bridge also copied `/lidar/odom` X/Y and `/imu/data` yaw directly, so small scan-matcher wander or AHRS yaw jitter appeared immediately as RViz RobotModel vibration.

## Fix 1 — RobotModel zero-motion lock

`imu_visual_tf_node` now separates motion from stationary behavior:

- Yaw still belongs to the IMU.
- When IMU gyro magnitude stays below 0.012 rad/s for 0.30 s, visual yaw is held exactly.
- While held, AHRS drift is absorbed into a yaw offset, so releasing the hold does not create a jump.
- X/Y still follow `/lidar/odom` while the chassis is moving.
- `/esc/odom` is used only as a visual stationary gate. When ESC linear/angular speed are approximately zero, LiDAR X/Y wander is absorbed into an offset instead of moving the URDF.
- If ESC odometry is unavailable, the bridge falls back to quiet IMU + quiet LiDAR twist.
- Mapping TF (`map -> odom -> base_footprint`) is unchanged.

## Fix 2 — Clean `/scan` before SLAM Toolbox

The LiDAR driver now removes isolated angular speckles before publishing `/scan`:

- 2-bin spatial neighborhood.
- A point survives with at least one nearby range-consistent neighbor.
- A one-bin/thin return can also survive when it is temporally consistent with the previous raw scan.
- `range_min/range_max` are aligned with the mapping chain at 0.15–8.0 m.
- Runtime logs report `clean_bins/raw_bins` and percentage removed.

This filter has no extra scan-frame latency; it operates on the completed revolution already being published.

## Fix 3 — Correct stationary detection in LiDAR odometry

The old `detect_motion()` compared compacted point-cloud array indices. Because invalid LaserScan bins are removed, index `i` in consecutive scans can represent different angles. The new logic rasterizes both scans back into 360 bearing bins before comparing ranges, then uses robust median difference + changed-beam ratio.

This prevents missing bins/speckles from being interpreted as chassis motion while still allowing slow AGV movement to be detected.

## Test

```bash
cd ~/ros
source /opt/ros/humble/setup.bash
rm -rf build/navigation install/navigation
colcon build --symlink-install --packages-select navigation
source install/setup.bash
ros2 launch navigation map.launch.py
```

Useful runtime checks:

```bash
ros2 topic hz /scan
ros2 topic hz /map
ros2 run tf2_ros tf2_echo odom visual_/base_footprint
```

When stationary, `visual_/base_footprint` should remain fixed. The LiDAR log should show `LiDAR cleanup: raw=... clean=... removed=...%`, and Hector jump count should increase much more slowly than in the supplied runtime log.
