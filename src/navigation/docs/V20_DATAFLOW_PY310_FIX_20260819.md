# V20 — ROS Humble Python 3.10 + Full Dataflow Recovery

## Root cause confirmed from 2026-08-19 23:42 runtime

The physical sensors were alive:
- IMU opened `/dev/ttyUSB0` and `/imu/data` was observed by the PRE-AMCL gate.
- LiDAR opened `/dev/ttyUSB1` and the C++ driver kept completing/publishing scan revolutions.

The navigation dataflow was broken between `/scan_raw` and `/scan` because
`scan_self_filter.py` used `#!/usr/bin/env python3`. The user's PATH resolved that
interpreter to uv CPython 3.11, while the apt-installed ROS 2 Humble rclpy binary is
for Ubuntu's Python 3.10. `initialpose_stamp_relay.py` had the same defect.

Consequences were deterministic:
`/scan=0` -> PRE-AMCL never READY -> no AMCL activation -> no `map->odom` -> RViz
RobotModel/message filters fail -> costmaps do not activate -> no Smac/MPPI -> camera
was also delayed by the Nav2 gate.

## V20 fixes

1. `scan_self_filter.py` and `initialpose_stamp_relay.py` use `/usr/bin/python3`
   explicitly; uv/virtualenv PATH can no longer select Python 3.11.
2. Both helper nodes respawn if they ever terminate unexpectedly.
3. Camera is launched after hardware preflight, independently of Nav2 readiness, so
   RViz camera diagnostics are never hidden by a navigation gate.
4. YOLO/hole-alignment remain after local-costmap/MPPI readiness to protect startup CPU.
5. The V19 `start_yolo_independent` ordering bug is removed: all start actions are
   defined before the event handler references them.
6. V20 installer verifies `/usr/bin/python3` is Python 3.10 and can import rclpy,
   verifies the installed shebangs, installs/tests CP210x software recovery, removes
   stale build/install products, and rebuilds all affected packages.
7. `verify_v20_dataflow.sh` validates the real chain from IMU/LiDAR through TF,
   localization, PlannerServer/Smac and ControllerServer/MPPI.

## Important configuration note

Nav2 Humble declares local costmap `width` and `height` as integer parameters.
The existing `width: 8` and `height: 8` are therefore intentionally retained; changing
them to `8.0` would be the type error, not the fix.
