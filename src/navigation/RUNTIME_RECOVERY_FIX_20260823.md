# Runtime Recovery Fix — 2026-08-23

Fixes based on log(20260823-132130):

- IMU and LiDAR USB role resolution are independent; one missing sensor no longer blocks the other driver.
- USB resolver keeps validated physical-port matching but falls back when a CP210x has moved to another hub socket.
- Ambiguous topology may use one-time protocol probing (WT901 vs YDLIDAR) before alias handoff.
- serial_transport_ready_gate supports per-sensor readiness.
- ROS Python runtime scripts are pinned to /usr/bin/python3 (Python 3.10 for ROS Humble) instead of inheriting uv Python 3.11.
- GUI main window is shown maximized before the ROS bridge begins emitting high-rate callbacks.
- Mapping consumers remain gated on both /imu/data and /scan_nav, so SLAM never starts from a partial sensor state.
- Autonomous AMCL/Nav2 remains fail-closed until both sensor topics, ESC odometry, EKF and TF are ready.
