# V58 complete autonomous runtime repair

V58 removes a startup dependency that coupled AMCL/Nav2 to camera discovery.
The authoritative PRE-AMCL gate now begins when `/imu/data` and `/scan_nav`
are proven live and independently validates ESC odometry, EKF odometry,
LiDAR safety and TF. Camera/YOLO starts after perception stale-owner cleanup
using the camera driver's native `device=auto` rediscovery path.

The GUI now subscribes to AMCL with the canonical transient-local QoS plus a
best-effort compatibility reader, and MPPI debug topics with a compatible
best-effort reader. Perception pages expose performance status and raw-camera
fallback. Each Map 1/2/3 snapshot renders a map-local static cost/inflation
preview using the production 0.45 m / 12.0 inflation parameters; live Nav2
costmaps remain bound to the one map actually active in map_server.

`autonomous_runtime_check.py` additionally verifies `/compute_path_to_pose`,
`/navigate_to_pose`, `/smac_plan`, MPPI debug output, perception, both
costmaps, lifecycle state and TF. `RUN_GOAL_ACCEPTANCE_V58.sh` enables the
Smac/MPPI-output requirement and must be run while a valid Goal Pose is active.
