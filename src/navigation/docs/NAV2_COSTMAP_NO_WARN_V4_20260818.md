# Nav2 Costmap No-Warn V4 — 2026-08-18

This revision removes the remaining autonomous Nav2 startup deadlock.

## Root cause fixed

`lifecycle_manager_navigation` was started only after the `/amcl_pose` readiness
gate exited. If AMCL had not published a pose yet, PlannerServer and
ControllerServer stayed unconfigured forever; therefore neither
`/global_costmap/costmap` nor `/local_costmap/costmap` could exist and RViz
showed both displays in warning state.

## New startup rule

Nav2 servers are created first, then `lifecycle_manager_navigation` starts
independently after a short deterministic delay. `/amcl_pose` remains a
diagnostic readiness gate only. This matches the normal Nav2 lifecycle model:
server lifecycle is not conditional on an application-level pose topic.

Costmap TF tolerance is raised to 1.0 s for real Jetson/USB scheduling jitter.
The frames remain unchanged: global costmap=`map`, local costmap=`odom`, robot
base=`base_footprint`.
