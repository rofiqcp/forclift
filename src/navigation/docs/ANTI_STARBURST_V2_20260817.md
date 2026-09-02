# Anti-Starburst V2 — 2026-08-17

Scope: mapping stability + RViz LaserScan only. USB physical roles, LiDAR packet decoder, IMU, ESC, URDF and sensor launch order are unchanged.

Fixes:
- RViz mapping/allsystem LaserScan reads `/scan` directly (Best Effort, decay 0.2 s).
- Hector median filter no longer fills raw `+inf` beams with neighbour ranges.
- Motion detector re-bins compacted points to 360 fixed bearings before comparing scans.
- Low-information scans (<60 valid points) hold odometry.
- Motion must be present for 2 consecutive scans before pose can change.
- Scan gaps >0.30 s hold pose and re-seed matcher.
- Match quality alone can no longer move the pose.
- Map score counts only observed map cells.
- Equal-score search candidates prefer the smallest motion instead of the first negative-edge candidate.
- Hector search and SLAM Toolbox angular search are narrowed; graph travel thresholds reject residual stationary jitter.
- SLAM scan queue increased from 1 to 3 to reduce queue-full drops.

Motion median threshold is 0.020 m so slow AGV movement remains detectable after the 2-scan confirmation gate.
