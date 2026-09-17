# Full Nav2 Autonomous Bringup — 2026-08-18

Run:

```bash
ros2 launch navigation autonomous.launch.py map:=/absolute/path/to/map.yaml
```

`map:=auto` or the historical literal `/path/to/map.yaml` selects the newest YAML in `/home/otomasi2/forclift/src/navigation/maps`.

Active stack:

- `nav2_map_server` → `/map`
- `nav2_amcl` → `map -> odom`
- `planner_server` → `nav2_smac_planner/SmacPlannerHybrid` + global costmap
- `controller_server` → MPPI Ackermann + local costmap
- `behavior_server`
- `bt_navigator` → `/navigate_to_pose`
- `velocity_smoother`
- `collision_monitor`
- localization + navigation lifecycle managers
- RViz `/goal_pose` bridge to `/navigate_to_pose`

Global costmap layers: `static_layer + obstacle_layer + inflation_layer`.
Local costmap layers: `obstacle_layer + inflation_layer`.
The inflation layer is part of each costmap; it is not a separate occupancy-map source.

Runtime validation:

```bash
ros2 run navigation autonomous_runtime_check.py
```

The checker verifies `/map`, both costmaps, `/navigate_to_pose`, lifecycle `active` states, AMCL and TF.
