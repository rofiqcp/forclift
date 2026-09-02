# Mapping V7 — obstacle refresh + anti-starburst

V7 is based on V6 and the runtime `log(20260818-111513)`.

Key runtime evidence:
- final scan stream remained in VERIFIED_2BYTE mode;
- physical revolutions were about 6 Hz;
- map-monitor known/free/occupied counts often stayed unchanged for long stretches despite continuous `/scan` (one unchanged-count stretch exceeded 7 minutes);
- occupied cells reached a peak then dropped while known/free cells continued to grow;
- Hector jump rejects rose sharply late in the run, and the IMU stream became unavailable mid-run.

Changes:
1. Cartesian endpoint-continuity LiDAR gate for oblique walls/real obstacles.
2. Persistent short far-shadow runs are rejected even when they repeat in RAW history.
3. Tighter Hector physical translation/yaw limits and 6 cm search window.
4. SLAM Toolbox consumes every fresh physical scan (zero travel thresholds), queue size 1.
5. Huber loss for robust scan graph optimization.
6. Faster obstacle evidence: min_pass_through=2, occupancy_threshold=0.20.
7. Correlation search narrowed to 0.30 m to reduce false radial registration.

The mapping architecture remains `/scan -> Hector /lidar/odom -> async slam_toolbox -> /map`; IMU is not integrated into X/Y and no new map-persistence layer is added.
