# V12 - RViz Goal Pose -> Smac Hybrid-A* visible path + smaller inflation

## Goal-path behavior

RViz `2D Goal Pose` publishes `/goal_pose`.

`goal_pose_nav2_bridge.py` now performs this deterministic sequence:

1. Receive `/goal_pose`.
2. Validate the selected point against the planning map `/nav_map`.
3. Reject occupied/unknown/out-of-map cells.
4. Call PlannerServer action `/compute_path_to_pose` with `planner_id=GridBased`.
5. `GridBased` is configured as `nav2_smac_planner/SmacPlannerHybrid`.
6. Publish the returned path as `/smac_plan` using Reliable + Transient Local QoS.
7. RViz displays `/smac_plan` as `SmacHybridAStarPath`.
8. Only after the Smac path is valid is `/navigate_to_pose` sent to BT Navigator.

Expected bridge log after a valid RViz click:

```
FREE-SPACE goal accepted: ... Requesting Smac Hybrid-A*...
SMAC HYBRID-A* PATH READY: points=... length=... m topic=/smac_plan
NavigateToPose accepted AFTER Smac path preview was published
```

## Inflation V12

Physical footprint is unchanged: `1.30 m x 0.80 m`.

- Global inflation: `0.25 m`, scaling `25.0`
- Local inflation: `0.35 m`, scaling `15.0`
- Global footprint padding: `0.00 m`
- Local footprint padding: `0.05 m`
- Local `/scan` obstacle layer remains enabled

The smaller inflation values are soft-cost margins only. Smac Hybrid-A* still uses the full polygon footprint for collision checking.

## Runtime verification

After autonomous launch is READY, in another terminal:

```bash
cd /home/otomasi2/forclift
source install/setup.bash
ros2 run navigation verify_smac_goal_path.py
```

Then click `2D Goal Pose` on known white/free space in RViz.

Expected final line:

```
PASS: PlannerServer + BT Navigator are ready and Smac path is visible.
```

Path topic:

```bash
ros2 topic echo /smac_plan --once
```

Planner plugin:

```bash
ros2 param get /planner_server GridBased.plugin
```

Expected:

```
String value is: nav2_smac_planner/SmacPlannerHybrid
```
