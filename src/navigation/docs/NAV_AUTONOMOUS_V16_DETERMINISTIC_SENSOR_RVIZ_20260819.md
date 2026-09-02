# Autonomous V16 — deterministic sensor startup and RViz

V16 fixes three independent startup races: serial drivers no longer race USB-role resolution, the Nav2 service gate now terminates after READY so `OnProcessExit` can advance, and RViz/map startup is independent from sensor/Nav2 readiness.

Serial order is now:

`preflight -> USB topology resolver -> serial transport readiness -> IMU -> LiDAR -> LiDAR odometry -> pre-AMCL gate`.

The transport readiness gate is protocol-blind and sends no sensor bytes. It waits for stable, distinct canonical tty endpoints and performs one successful open/close check on each endpoint. If USB is electrically unavailable, the gate keeps waiting while RViz and the saved map remain available.

The saved map servers use their own lifecycle manager and RViz starts one second after foundation startup. AMCL retains its separate lifecycle gate and is activated only after `/imu/data`, `/scan`, `/lidar/odom`, filtered odometry and required TF are valid.

MPPI remains the Nav2 `FollowPath` controller with Ackermann motion model, while Smac Hybrid-A* remains the global planner.
