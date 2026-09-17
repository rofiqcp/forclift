# V35 Global Costmap, Goal, MPPI, and RViz Fix — 2026-08-25

## Evidence from the supplied run

- AMCL reported localized and `map->odom` was valid.
- Local costmap, PlannerServer process, ControllerServer/MPPI process, LiDAR,
  IMU, EKF, and the complete TF chain were present.
- Global costmap remained `NODE UP / waiting for topic`.
- The GUI published a goal from Map 1 while Map 3 was the map loaded by
  `map_server`.
- MPPI was configured with `visualize: false`, so its visualization topics
  could not publish even when ControllerServer was running.
- RViz did not explicitly disable Odometry covariance and did not bound the
  retained Odometry marker count/arrow size.

The supplied ZIP contains build logs, not the launch console.  Therefore the
source fixes below remove the deterministic startup deadlocks visible in the
launch graph; the real Jetson run remains the authoritative runtime proof.

## Changes

1. `autonomous_map_tf_ready_gate.py` separates the minimal PlannerServer
   prerequisite from the stricter AMCL motion-confidence decision.  It requires
   `/map`, AMCL `ACTIVE`, `map->odom`, and `map->base_footprint` for three stable
   cycles.
2. The global StaticLayer uses the lifecycle-proven `/map`.  The optional
   derived `/nav_map` cannot block production global-costmap startup.
3. `goal_pose_nav2_bridge.py` starts immediately with the autonomous foundation,
   queues the latest valid goal, and sends it when `/navigate_to_pose` becomes
   ready.
4. The GUI rejects Goal and Initial Pose clicks from a map slot other than the
   map currently loaded by `map_server`.
5. MPPI visualization is enabled with bounded down-sampling:
   `trajectory_step=20`, `time_step=3`.
6. RViz disables Odometry covariance rendering, keeps one compact marker, and
   reduces costmap overlay opacity.
7. Persistent runtime migration enforces the critical `/map` and MPPI settings,
   so an old file under `config/runtime/navigation` cannot silently undo V35.

## Clean deployment

```bash
cd /home/otomasi2/forclift
bash src/navigation/tools/rebuild_navigation_clean.sh /home/otomasi2/forclift
source /opt/ros/humble/setup.bash
source /home/otomasi2/forclift/install/setup.bash
python3 src/navigation/tools/verify_v35_global_goal_mppi.py
ros2 launch navigation autonomous.launch.py map:=auto
```

Select Goal only on the map slot shown as `MAP NAV` in the GUI.  MPPI
`/trajectories` and `/transformed_global_plan` are expected only while a valid
NavigateToPose goal is actively executing.

```bash
AUTONOMOUS_CHECK_SECONDS=20 \
AUTONOMOUS_CHECK_REQUIRE_PERCEPTION=0 \
AUTONOMOUS_CHECK_REQUIRE_MPPI_OUTPUT=1 \
ros2 run navigation autonomous_runtime_check.py
```
