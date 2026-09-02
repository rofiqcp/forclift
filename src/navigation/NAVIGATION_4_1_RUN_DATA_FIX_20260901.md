# Navigation 4.1 + Run Data Fix — 2026-09-01

- Added Navigation **4.1 Ringkasan Kelompok Pengujian** and **Tabel 4.1** from the supplied BAB II–IV DOCX.
- Navigation menu now defaults to 4.1.
- 4.1 is a report overview and therefore cannot start a ROS acquisition run.
- Every recorded raw row now stores subsystem, section, table, variation, condition, and all active experiment/YAML/ground-truth parameter fields.
- Navigation raw CSV now includes the complete scalar telemetry already emitted by the GUI bridge for LiDAR, IMU, odometry/EKF, SLAM, AMCL, commands, path, goal, derived metrics, and host resources.
- Navigation 4.4.3 speed-response metrics now use odometry velocity + drive target instead of steering feedback.
- Fixed malformed duplicate list fragments in `config/nav2_ackermann.yaml`.
