# V21 — Latest Map + Direct Hardware IMU RobotModel

## Root causes from the 2026-08-20 autonomous run

1. `map:=auto` was not actually latest. `_resolve_map_yaml()` returned
   `map_20260819_003151.yaml` first whenever that file existed, before comparing
   any newer maps.
2. Mapping output lived inside `src/navigation/maps`, so clean source replacement
   could move the newest map into a backup source tree.
3. Autonomous RobotModel used an identity static bridge from real
   `base_footprint` to `visual_/base_footprint`. The visual model therefore
   followed EKF/LiDAR heading rather than direct hardware IMU yaw.
4. The captured run also contained avoidable Jetson load/configuration warnings:
   invalid `controller_server.verbose`, planner/controller deadline misses and
   startup EKF update-rate misses.

## V21 architecture

### Map

- New mapping saves to `/home/otomasi2/forclift/maps`.
- Installer migrates valid YAML + image pairs from current and `src_backup*`
  trees into the persistent map directory.
- `map:=auto` ranks `map_YYYYMMDD_HHMMSS` by embedded timestamp, validates the
  YAML/image pair, and searches persistent + legacy + backup directories.
- No hard-coded preferred map remains.

### RobotModel

Navigation TF remains exactly:

`map -> odom -> base_footprint`

Visual-only TF becomes:

`odom -> visual_/base_footprint`

where X/Y comes from `/odometry/filtered` and yaw comes directly from
`/imu/data` with a one-time heading alignment. This TF is isolated from Nav2 and
cannot create a second owner for `odom -> base_footprint`.

### Runtime load

- controller: 5 Hz
- local costmap: 5 Hz update / 3 Hz publish
- velocity smoother: 10 Hz
- MPPI: 400 trajectories x 20 steps
- Smac expected planner rate: 0.20 Hz; BT replanning: 0.20 Hz
- autonomous camera: 15 FPS; YOLO max inference: 8 FPS
- EKF: 4 Hz with queued 10 Hz sensor input
- invalid `controller_server.verbose` parameter removed

## Validation after deployment

```bash
bash src/navigation/tools/verify_v21_map_imu.sh
```

The checker compares the map loaded by `map_server` against the newest valid map
on disk, checks `/imu/data`, `/scan`, `/odometry/filtered`, visual and navigation
TF chains, lifecycle states, Smac action and MPPI FollowPath action.
