# Mapping Async Master Fix — 2026-08-12

## Symptom
Live `/scan` runs at approximately 10 Hz, but `/map` appears to trail the scan in RViz.

## Runtime evidence
The affected runtime launched `sync_slam_toolbox_node`. The supplied master launches
`async_slam_toolbox_node` and uses `slam_toolbox_async.yaml` values equivalent to the
current `navigation/config/slam_toolbox.yaml`.

## Fix
- Keep the master SLAM parameters unchanged:
  - `throttle_scans: 1`
  - `map_update_interval: 0.1`
  - `minimum_time_interval: 0.05`
  - `minimum_travel_distance: 0.01`
  - `minimum_travel_heading: 0.0087`
  - `resolution: 0.05`
- Replace synchronous SLAM Toolbox runtime with lifecycle-managed
  `async_slam_toolbox_node`.
- Preserve Hector LiDAR odometry, IMU-driven visual yaw, URDF and TF ownership from
  the previous master-aligned fix.

## Expected runtime signature
`ps` / health check must show `async_slam_toolbox_node` and must not show
`sync_slam_toolbox_node` for mapping.
