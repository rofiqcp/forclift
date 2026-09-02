# V18 Mapping Runtime + Costmap + Smac Hybrid-A* fix — 2026-08-19

Root causes from `log(20260819-145939).zip`:

1. Mapping START self-terminated because `autonomous_sensor_preflight.sh` used a broad `pkill -f mapping_runtime.launch.py` from inside the current mapping runtime.
2. PlannerServer configured two full Smac Hybrid-A* plugins before activation. The log reached `Created global planner plugin ... SmacPlannerHybrid` but `/global_costmap/costmap` stayed at `0/1`, so PlannerServer never became operational during the captured run.
3. `goal_pose_nav2_bridge` was chained after the local costmap, so `/smac_plan` could not exist while global planner startup was stalled.

V18 fixes:

- ancestor-safe stale-runtime cleanup; current mapping runtime is never killed;
- one canonical `GridBased` Smac Hybrid-A* plugin;
- explicit 10 m Reeds-Shepp lookup table (compact map, faster initialization);
- Smac RViz bridge starts immediately after real global costmap publication;
- normal action-server waiting logs are INFO rather than WARN;
- V17 software CP210x replug remains in place.
