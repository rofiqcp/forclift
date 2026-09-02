# Nav2 Costmap + Lifecycle Final Fix — 2026-08-18

This revision is based on the latest `ros 18-08-2026 21.55.zip` / `src(20260818-145412).zip`.

Fixes:

1. `autonomous_sensor_preflight.sh` is now actually executed before new sensor/camera nodes.
2. Nav2 server processes are created early, while `lifecycle_manager_navigation` activates them only after live EKF and AMCL localization are proven.
3. A dedicated costmap gate verifies `/global_costmap/costmap` and `/local_costmap/costmap`.
4. The readiness gate supports `nav_msgs/OccupancyGrid` with Reliable + Transient Local QoS.
5. RViz Global/Local Costmap displays use matching Reliable + Transient Local QoS.
6. Global StaticLayer explicitly subscribes to `/map`.
7. Costmap publish rates are increased for responsive RViz display.
8. `nav2_behavior_tree` is an explicit runtime dependency and launch prerequisite.
9. The existing `/goal_pose -> /navigate_to_pose` bridge remains enabled.

Expected autonomous chain:

`map_server -> AMCL(map->odom) -> EKF(odom->base_footprint) -> lifecycle activation -> PlannerServer/GlobalCostmap + ControllerServer/LocalCostmap -> Smac Hybrid-A* -> MPPI Ackermann -> velocity smoother -> collision monitor -> ESC`.
