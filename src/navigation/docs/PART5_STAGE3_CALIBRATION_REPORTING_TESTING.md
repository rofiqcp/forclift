# PART-5 Stage-3 — Calibration, Reporting, Runtime Schema, Regression, Perception Composition

## Scope

Stage-3 builds on `PART5_STAGE2_mapping_planning_manual_safety` without changing the production localization and safety contracts established in previous stages. It adds engineering calibration workflows, corrects report semantics, makes persistent configuration upgrades auditable, adds automated/replay tests, and reduces camera-to-YOLO inter-process overhead on Jetson.

## 1. Guided calibration workflows

The GUI now contains data-backed calibration panels instead of only calibration instructions.

### IMU stationary calibration
1. Put the AGV completely stationary on a rigid surface.
2. If the IMU/base is physically level with +Z up, keep **Level +Z gravity** enabled.
3. Collect the requested samples.
4. The wizard rejects excessive gyro/acceleration variation.
5. Candidate residual bias is added to the current configured bias because `/imu/data` is already bias-corrected.
6. YAML is staged, `fsync`ed, atomically replaced, then read back and compared.
7. Restart `imu_node` and repeat the stationary test to verify residual bias.

### Steering encoder calibration
The ESC publishes `/esc/steering_raw_ticks`. The wizard deliberately uses raw encoder feedback rather than the already calibrated steering-angle topic.

1. Physically center steering and capture CENTER.
2. Put steering at a physically measured left angle and capture LEFT.
3. Put steering at a physically measured right angle and capture RIGHT.
4. Fit `ticks = zero + sign * ticks_per_rad * angle`.
5. Reject excessive residual error or implausible encoder span.
6. Update `steering_zero_ticks`, `steering_ticks_per_rad`, `steering_sign`, symmetric safe max steering, and canonical vehicle geometry.
7. Recompute a conservative Ackermann minimum turning radius `R >= L/tan(delta_max)` and synchronize derived Nav2/ESC geometry.

Do not enter nominal/guessed angles. They must be physically measured.

### Wheel radius / distance scale
1. Measure a straight physical test distance.
2. Capture START from `/esc/odom`.
3. Drive straight at commissioning speed and stop.
4. Capture END.
5. The wizard rejects large yaw change and implausible scale changes.
6. Effective loaded wheel radius is scaled by `physical_distance / odom_distance` and synchronized into canonical geometry.
7. Repeat the same measured-distance test after restart as independent validation.

### Wheelbase + REP-103 physical confirmation
The wizard never promotes CAD dimensions to field truth. Enter measured axle-center-to-axle-center wheelbase and explicitly confirm in RViz that physical front is +X, left is +Y, and up is +Z in `base_footprint`. Only then are the corresponding validation flags set.

### LiDAR safety quality baseline
The baseline uses `/scan_safety`. Candidate quality thresholds are not allowed to loosen existing safety thresholds; they may only remain equal or become stricter. This does **not** replace thin-obstacle and stopping-distance tests.

## 2. Reporting semantics corrected

- Cross-track error still uses nearest path segment distance.
- Heading error now interpolates the **orientation stored in Smac path poses**. It does not infer heading from segment travel direction, so Reeds-Shepp reverse segments no longer look like ~180-degree heading errors.
- Navigation result comes from the action goal UUID and terminal event published by `goal_pose_nav2_bridge.py`, not the last entry of a generic `GoalStatusArray`.
- `first_plan_latency_s` means action-goal acceptance to the first observed PlannerServer `/plan`. It is **not** labeled PlannerServer compute time.
- Unstamped `Twist` timing is reported as `stage_arrival_gap_ms`, not causal end-to-end latency. Use `ros2_tracing` if actual callback/executor/algorithm latency is required for the thesis.
- Experiment snapshots include the active `runtime_manifest.yaml` in addition to parameter YAML and SHA256 hashes.

## 3. Universal persistent runtime schema

`navigation/config/runtime_schema.yaml` defines the current software contract and managed file revisions across navigation, ESC and perception.

At autonomous/mapping launch:
1. historical owned-key migrations run;
2. geometry schema is migrated/synchronized;
3. missing runtime files are seeded;
4. missing keys introduced by new package defaults are recursively merged;
5. existing calibration/tuning values always win;
6. changed files are backed up under `.runtime_schema_backups`;
7. every managed file is parsed and hashed;
8. `$AGV_RUNTIME_CONFIG_ROOT/runtime_manifest.yaml` is atomically written.

Manual migration/inspection:

```bash
ros2 run navigation migrate_runtime_schema.py
```

## 4. Transactional tuning profiles

Profile restore now uses an isolated staging tree. Every staged YAML is parsed and semantically validated before active files are touched. All replacement temp files are prepared before the first `os.replace`. If replacement or post-activation validation fails, old files are automatically restored and files introduced only by the failed profile are removed. Hidden backup/staging trees are excluded from profile contents, preventing backup-of-backup growth.

## 5. Automated tests

Build-time PyTest tests cover pure engineering math and transactions:

```bash
colcon test --packages-select navigation
colcon test-result --verbose
```

Direct source-tree equivalent:

```bash
PYTHONPATH=src/navigation/python python3 -m pytest -q src/navigation/test
```

Covered contracts include stationary IMU bias, negative steering encoder direction, wheel radius scaling, safety-baseline non-loosening, reverse Reeds-Shepp heading, wrapped yaw interpolation, runtime default-key merge preserving tuning, and profile rollback.

## 6. Sensor-only rosbag regression

A replay harness is included for regression on a running stack:

```bash
ros2 run navigation run_sensor_rosbag_regression.sh /path/to/bag 30
```

It replays only sensor/hardware-observation topics such as `/scan_nav`, `/scan_safety`, `/imu/data`, `/esc/odom`, `/esc/ready`, static TF and operator initial pose. It intentionally does **not** replay `/odometry/filtered`, `/amcl_pose` or `/smac_plan`; those outputs must be recomputed by the software under test.

For reproducible thesis regression, keep a fixed sensor-only bag and map pair with recorded SHA256.

## 7. Jetson camera-to-YOLO composition

`AstraRGBNode` and `ObstacleDetectorNode` are now also built as `rclcpp_components`. Production autonomous, mapping and all-system launch files use `perception_all.launch.py`, which defaults to a multithreaded component container and `use_intra_process_comms=true` for camera and YOLO. Standalone executables remain available with `use_composition:=false`.

This removes DDS serialization between camera and YOLO in the production topology. It is **not** full NVMM zero-copy: the current camera node still materializes a CPU `sensor_msgs/Image`, and TensorRT can still upload/preprocess from that representation. A later NVMM/CUDA-native transport would be required for true end-to-end zero-copy.

The CMake path explicitly restores GStreamer/PkgConfig dependencies and checks/enables the CUDA language before adding the optional GPU preprocess kernel. TensorRT/OpenCV-CUDA fallback behavior from PART-3 remains intact.

## 8. Required Jetson/hardware validation

Static verification cannot replace these tests:

1. Build `navigation`, `esc`, and `yolo_obstacle_detection_ros2` on ROS 2 Humble/Jetson.
2. Run all PART verifiers and `colcon test`.
3. Verify `/esc/steering_raw_ticks` is stable at each physical steering position.
4. Run IMU calibration and residual stationary test.
5. Run steering calibration using independently measured angles, then target-vs-actual verification.
6. Run wheel-radius calibration and a second independent measured-distance verification.
7. Confirm measured wheelbase and REP-103 axes before setting validation flags.
8. Run thin-obstacle, sparse-scan, LiDAR disconnect, manual gate, autonomy gate and E-stop fault tests from previous stages.
9. Record an experiment and verify goal UUID, reverse heading, first-plan latency, config manifest/hash and rosbag outputs.
10. Compare perception with `use_composition:=true` and `false` using `/obstacle_detection/performance`, tegrastats and system metrics.

Do not enable full-speed autonomous operation solely because the static verifier passes.
