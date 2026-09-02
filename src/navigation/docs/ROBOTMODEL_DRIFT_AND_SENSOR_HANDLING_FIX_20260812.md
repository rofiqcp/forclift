# RobotModel drift & sensor-handling fix — 2026-08-12

Runtime evidence from `log(20260812-093030)` showed two independent causes:

1. The previous visual EKF fused IMU `ax/ay`. Small bias is integrated into velocity and then position, so moving only the IMU can produce large RobotModel drift.
2. The previous visual EKF fused LiDAR absolute `x/y/yaw`. The LiDAR odometry log jumped from roughly `(-0.11,-0.05,5 deg)` to around `(-1.18,-0.51,6.5 deg)` and later about `(-0.78,-1.14,56 deg)` while the sensor was handled. Because LiDAR X/Y was a direct RobotModel measurement, the model followed the sensor.

## Final visual ownership

- Mapping: unchanged, LiDAR-only.
- RobotModel translation: real `/esc/odom` body velocity (`vx`) plus `vy=0` non-holonomic constraint.
- RobotModel absolute yaw: LiDAR yaw only, through `visual_lidar_yaw_guard_node`.
- IMU accel/gyro/magnetometer: still measured, low-pass filtered, bias-conditioned, and used to validate whether a LiDAR yaw change is consistent with chassis motion.
- IMU acceleration is **not integrated into position**.
- EKF `publish_tf=false`; mapping TF remains untouched.

## Bench-test behavior

- Move IMU only: RobotModel should not translate/drift.
- Move LiDAR only while IMU is stationary: RobotModel should not translate; LiDAR yaw changes are held by the guard.
- Drive wheels: `/esc/odom` velocity moves RobotModel.
- Rotate chassis so IMU gyro detects motion and LiDAR yaw changes: yaw is accepted from LiDAR.

This intentionally models sensors as rigidly mounted to the chassis during real operation while rejecting isolated sensor handling during bench testing.
