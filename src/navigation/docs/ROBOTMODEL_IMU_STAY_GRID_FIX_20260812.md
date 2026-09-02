# RobotModel IMU stay-on-grid fix

Mapping remains unchanged and LiDAR-only.

The visual RobotModel uses:

- X/Y: `/lidar/odom`
- Z: locked to `0.0`
- roll: `0`
- pitch: `0`
- yaw: relative `/imu/data` yaw, aligned to the initial LiDAR heading

This prevents a full-URDF roll/pitch rotation from making the AGV appear to slide or leave the 2-D RViz grid.
