# LiDAR-only obstacle persistence fix — 2026-08-12

Runtime log `20260812_134357.txt` showed a valid fast map, but occupied cells peaked at 2377 and then fell continuously to 1595 while free cells increased. The LiDAR itself remained ~10 Hz with zero invalid/checksum packets.

## Changes

1. SLAM Toolbox `occupancy_threshold` reduced from 0.10 to 0.04. Karto decides occupied/free from hit-count divided by pass-through-count. This gives static obstacle endpoints more hysteresis against a small number of contradictory free rays.
2. Hector helper map free update reduced 0.15 -> 0.05 and occupied update increased 0.35 -> 0.45. This helper grid is only for LiDAR odometry; `/map` is still produced by SLAM Toolbox.
3. Fixed `min_scan_match_quality` semantics. It now compares the actual relative score improvement against zero-motion instead of comparing the 1+improvement telemetry value.
4. Near-equal scan-match scores now prefer the lower-motion candidate. This suppresses quantisation-driven edge-of-window jumps which can ray-trace through previously occupied cells.
5. Map monitor now reports `occupied-retention` and warns if occupied cells fall more than 20% from their peak.

No IMU, EKF, camera or wheel odometry is added to the mapping chain.
