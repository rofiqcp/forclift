# PART 1 — Core Navigation / Localization / Mapping Revision

This revision intentionally focuses on runtime correctness and safety. GUI schema/tuning UX
and TensorRT optimization belong to PART 2/3.

## Implemented

1. **Scan-synchronous LiDAR odometry**
   - `/lidar/odom` is now stamped with the LaserScan measurement timestamp.
   - Twist is computed between successive scan measurements, not a 30 Hz wall timer.
   - Rejected/held scan matches publish weak covariance instead of pretending high confidence.
   - Quality-dependent pose/twist covariance is emitted for accepted scan matches.
   - Fixed duplicate C++ declarations that could stop compilation.

2. **Single TF ownership**
   - EKF owns `odom -> base_footprint` in mapping and autonomous.
   - SLAM Toolbox owns `map -> odom` during mapping.
   - AMCL owns `map -> odom` during autonomous localization.
   - Hector publishes `/lidar/odom` only and never competes for production TF.

3. **Wheel + IMU primary local odometry**
   - `/esc/odom` forward/non-holonomic velocity is the primary translational input.
   - `/imu/data` gyro-Z is the high-rate rotational input.
   - `/lidar/odom` is a conservative planar pose correction/fallback.
   - EKF runs at 30 Hz, faster than the 5 Hz MPPI controller.

4. **Separate autonomous LiDAR-odom profile**
   - `hector_autonomous.yaml` supports the Nav2 speed envelope without the old 0.38 m/s rejection mismatch.
   - Mapping keeps a slower/conservative scan matcher profile.

5. **IMU calibration is actually applied**
   - `accel_bias_mps2`, `gyro_bias_rps`, `mag_bias_t`, `mag_soft_iron` are loaded from `imu.yaml` and applied in `IMUFilter`.
   - Composite `/imu/data` is withheld when accel/gyro components are stale.
   - Duplicate composite samples are not synthesized from unrelated packet types.
   - Stale AHRS orientation is marked unavailable rather than reported as current.

6. **Mapping load reduction**
   - SLAM map raster update changed to 0.5 s.
   - Non-zero travel/heading thresholds prevent duplicate zero-motion scan processing.

7. **Navigation safety**
   - Collision Monitor source timeout reduced from 8.0 s to 0.45 s.
   - Added rear stop/slow polygons because REEDS_SHEPP + MPPI permit reverse motion.
   - Final command guard now requires LiDAR + IMU freshness + `/esc/ready=true` + fresh command.
   - Reverse MPPI/smoother limit aligned with actuator mux at 0.30 m/s.

8. **Single planning authority**
   - Goal bridge no longer computes a separate preview path before NavigateToPose.
   - BT Navigator / PlannerServer computes the real path exactly once.
   - Canonical `/plan` is relayed to `/smac_plan` for RViz/reporting.

9. **Canonical geometry + verifier**
   - Added `config/vehicle_geometry.yaml`.
   - Added `tools/verify_part1_core.py` to detect Smac/MPPI/ESC/footprint/safety drift.
   - Current 0.70 m wheelbase and 0.34 rad max steering imply ~1.979 m Ackermann radius, consistent with the 2.0 m Nav2 radius.

## Deliberate fail-safe remaining

`esc/config/ackermann_1_board.yaml` still has `steering_calibrated: false`.
Do not set it true until physical steering zero, sign, ticks/rad and end limits are measured.
The static verifier reports this as a warning, not a failure.

## Required on-Jetson validation

```bash
cd /home/otomasi2/forclift
colcon build --symlink-install --packages-select navigation esc
source install/setup.bash
python3 src/navigation/tools/verify_part1_core.py
ros2 launch navigation mapping_runtime.launch.py
# then run mapping checks while physically moving the AGV
ros2 launch navigation autonomous.launch.py
python3 install/navigation/lib/navigation/autonomous_runtime_check.py
```

A static check cannot validate serial hardware, TF timing under load, TensorRT, or real steering geometry.
