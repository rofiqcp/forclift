# Fast SLAM Toolbox map update fix — 2026-08-12

Target: `/map` follows the live 10 Hz `/scan` quickly while preserving stable scan matching.

## Changes

1. `slam_toolbox.yaml`: `scan_queue_size: 1` for `async_slam_toolbox_node`, so stale scans do not build up behind the newest scan.
2. `slam_toolbox.yaml`: pose-graph insertion thresholds reduced to 5 mm and 0.25 deg. This makes slow AGV motion visible sooner without forcing unlimited stationary nodes.
3. `/map` remains `map_update_interval: 0.1` (10 Hz). Publishing faster than the 10 Hz lidar would only rasterize the same data more often.
4. Interactive SLAM mode disabled during normal live mapping to reduce unnecessary overhead.
5. `hector.yaml`: local scan-filter window reduced 5 -> 3 and internal map update gate reduced 50 mm/2 deg -> 15 mm/0.5 deg so LiDAR odometry reacts sooner.
6. Fixed `compute_map_score()` so unknown/unobserved map cells are not counted in the score denominator. The previous code produced `-Wmisleading-indentation` during build and diluted scan-match quality.

## Runtime checks

```bash
ros2 topic hz /scan
ros2 topic hz /map
ros2 topic hz /lidar/odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 param get /slam_toolbox scan_queue_size
ros2 param get /slam_toolbox minimum_travel_distance
ros2 param get /slam_toolbox minimum_travel_heading
ros2 param get /slam_toolbox map_update_interval
```

Expected: `/scan` ~10 Hz, `/map` can publish close to 10 Hz when the pose graph/map changes, and no steadily growing stale scan queue.
