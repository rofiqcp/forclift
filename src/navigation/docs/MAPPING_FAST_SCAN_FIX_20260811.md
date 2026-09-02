# Fast scan-driven mapping fix — 2026-08-11

Runtime log diagnosis:

- Physical LiDAR `/scan` is healthy at ~10 Hz.
- The old `/scan` publisher used BEST_EFFORT, while at least one runtime subscriber requested RELIABLE QoS.
- `slam_toolbox` reported a startup message-filter queue overflow.
- LiDAR scan-matching odometry rejected a very large fraction of deltas; the search window allowed false matches far larger than plausible 10 Hz AGV motion.
- Motion detection compared compacted Cartesian points after invalid beams were removed, so indexes did not reliably correspond to equal LiDAR angles.
- The scan matcher score denominator incremented even for unobserved map cells because of missing braces.
- `slam_toolbox` only processed scans after non-zero odometric travel thresholds; when LiDAR odometry froze, map growth also froze.

Fixes:

1. `/scan` publisher changed to RELIABLE/VOLATILE KeepLast(10), compatible with reliable and best-effort local subscribers.
2. Hector motion detection now compares the original LaserScan bins by angle.
3. Hector map-score observed-cell counting fixed.
4. 10 Hz scan-matching search window and physical gates tightened; response smoothing made faster.
5. Hector internal map update thresholds reduced.
6. `slam_toolbox` minimum travel distance/heading set to zero so every valid scan is eligible for scan matching; `minimum_time_interval=0.05` still limits processing load.
7. SLAM startup delayed by 0.5 s so odom/TF is primed before its scan message filter starts.

Expected behavior: `/map` can update at the configured 0.1 s period when new 10 Hz scans arrive, rather than waiting for a large odometric displacement.
