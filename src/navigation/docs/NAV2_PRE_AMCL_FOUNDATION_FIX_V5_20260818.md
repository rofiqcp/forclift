# Nav2 Pre-AMCL Foundation Fix V5 — 2026-08-18

## Why the fix starts before AMCL

V4 started new Map Server, AMCL, EKF, Robot State Publisher and Nav2 processes
at launch time, while the stale-owner preflight ran in parallel. An old mapping
runtime could therefore still publish `/map` / `map->odom` during the first AMCL
startup. In addition, the old sensor gate proved topics only, not the TF links
AMCL actually needs.

## Deterministic startup

0. **Preflight only**: stop stale mapping/autonomous/Nav2/localization children.
1. Start RSP, drivers, LiDAR odometry and EKF; create lifecycle nodes unconfigured.
2. `autonomous_pre_amcl_gate.py` requires fresh `/imu/data`, `/scan`,
   `/lidar/odom`, `/odometry/filtered`, correct frame IDs, static sensor TF, and
   dynamic `odom -> base_footprint`.
3. Only then start localization lifecycle: `map_server` activates first, `amcl`
   second.
4. AMCL must publish `/amcl_pose` before navigation lifecycle is started.
5. Planner/Controller then activate and publish global/local costmaps.

No fake `map -> odom` is introduced. AMCL remains its sole owner; EKF remains
sole owner of `odom -> base_footprint`.

## AMCL completion gate

After the pre-AMCL foundation is ready, `lifecycle_manager_localization`
activates Map Server then AMCL. `autonomous_amcl_ready_gate.py` then requires
`/map`, AMCL lifecycle `active`, `/amcl_pose`, `map -> odom`, and
`map -> base_footprint`. Only its successful exit starts the navigation lifecycle.
