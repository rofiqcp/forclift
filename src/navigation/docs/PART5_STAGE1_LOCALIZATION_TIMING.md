# PART-5 Stage-1 — Localization & Timing Integrity

## Production localization contract

```text
/esc/odom ─┐
           ├── robot_localization EKF ──> odom -> base_footprint
/imu/data ─┘

/scan_nav ─────────> AMCL ──────────────> map -> odom

/scan_nav ─────────> hector_slam_node ──> /lidar/odom (DIAGNOSTIC ONLY)
```

`/lidar/odom` is intentionally excluded from both autonomous and mapping EKF
profiles so the same LiDAR measurement is not counted once through local scan
matching and again through AMCL/SLAM global localization.

## AMCL initialization

The safe default is `OPERATOR` in `config/localization_startup.yaml`. AMCL never
assumes `(0,0,0)`. In RViz use **2D Pose Estimate** (published through
`/initialpose_safe` and the timestamp relay) or use the same action from the GUI.

`KNOWN_POSE` is available only for a physically validated start/dock pose. It is
bound to the exact saved-map YAML+image pair by SHA256. Compute that hash on the
robot with:

```bash
python3 src/navigation/tools/map_pair_sha256.py /path/to/map.yaml
```

Copy the resulting 64-hex digest to `known_pose_map_sha256`, set the validated
pose, then set `initial_pose_mode: KNOWN_POSE`. A hash mismatch blocks startup;
it never falls back to origin.

## Convergence gate

AMCL readiness requires all of the following before Nav2 can continue:

- map available;
- AMCL lifecycle ACTIVE;
- an actual AMCL pose received;
- configured number of pose samples;
- covariance X/Y/yaw below thresholds;
- pose spread below translation/yaw stability thresholds;
- `map -> odom` and `map -> base_footprint` available.

For a stationary robot the gate requests AMCL no-motion updates after an initial
pose has actually been supplied.

## Pre-AMCL gate

The pre-AMCL gate now requires independent local-localization inputs:

- `/esc/odom`;
- `/imu/data`;
- `/scan_nav`;
- `/lidar/safety_healthy == true`;
- `/odometry/filtered`;
- static base-to-LiDAR and base-to-IMU TF;
- dynamic `odom -> base_footprint` TF.

It no longer subscribes to or waits for `/lidar/odom`.

## Persistent runtime migration

`navigation_runtime/localization_config.py` migrates persistent runtime YAML with
backup/readback. It removes every `odom1*` key from the production EKF profiles,
disables AMCL default initial pose, preserves unrelated MPPI/Smac tuning, and
seeds `localization_startup.yaml`.

Backups are kept under:

```text
$AGV_RUNTIME_CONFIG_ROOT/navigation/.stage5_localization_backups/
```

## Timing diagnostics

`localization_timing_monitor.py` publishes:

```text
/localization/timing_health
```

for `/scan_nav`, `/imu/data`, `/esc/odom`, `/odometry/filtered` and `/amcl_pose`.
The JSON contains header-stamp age, source period, receive period and jitter over
a rolling window. This is diagnostic-only and is recorded by the experiment
pipeline.

The Astra camera timestamp path was also hardened:

- V4L2 uses kernel capture-buffer timestamps mapped to ROS time;
- GStreamer uses `GstBuffer` PTS deltas mapped to ROS time;
- callback/publish time is only a fallback when a source timestamp is absent.

## Required Jetson validation

```bash
cd /home/otomasi2/ros
colcon build --symlink-install --packages-select navigation esc yolo_obstacle_detection_ros2
source install/setup.bash

python3 src/navigation/tools/verify_part5_stage1_localization.py
ros2 launch navigation autonomous.launch.py
```

In the default OPERATOR mode autonomous bringup must stop at AMCL readiness and
print that it is waiting for a 2D Pose Estimate. After providing a plausible
pose, verify AMCL convergence and:

```bash
ros2 topic echo /localization/timing_health --once
ros2 topic hz /esc/odom
ros2 topic hz /imu/data
ros2 topic hz /odometry/filtered
ros2 topic hz /scan_nav
```

Also verify `/lidar/odom` may stop/fail without preventing pre-AMCL readiness,
while loss of ESC odometry, IMU, LiDAR safety health or EKF correctly blocks
startup.
