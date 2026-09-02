# Map Error Fix — 2026-08-12

Runtime log `20260812_132030.txt` proved that the LiDAR was healthy (~10 Hz, zero invalid/checksum packets), and SLAM Toolbox initially published a valid OccupancyGrid. Immediately afterwards `sync_slam_toolbox_node` terminated with:

`Mapper FATAL ERROR - unable to get pointer in probability search!`

The failure appeared after the local Karto correlation search geometry had been changed from the stable 0.50 m / 0.01 m combination to 0.35 m / 0.01 m. This package restores the stable matcher geometry and the upstream-style angle search values while retaining the fast LiDAR-only odometry path.

Mapping ownership remains:

`/scan -> hector_slam_node -> odom -> base_footprint`

`/scan + LiDAR-derived odom -> sync_slam_toolbox_node -> map -> odom + /map`

IMU, EKF, IMU visual fusion and wheel odometry are not inputs to mapping.

RViz mapping configuration was also cleaned so it no longer requests `visual_/...` IMU frames when running LiDAR-only mapping.
