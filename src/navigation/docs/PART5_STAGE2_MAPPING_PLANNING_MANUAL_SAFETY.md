# PART 5 Stage 2 — Mapping, Planning, and Manual Safety

## Production changes

1. `/nav_map` is immutable during an autonomous session. `live_nav_map_footprint_cleaner.py` is no longer launched.
2. One-time deterministic `prepare_nav_map.py` cleanup may still create the derived saved planning map before `nav_map_server` starts.
3. Global costmap preserves unknown cells (`track_unknown_space: true`) and Smac rejects unknown (`allow_unknown: false`).
4. Global costmap includes a live `/scan_nav` ObstacleLayer so blocked corridors can cause global replanning. The observation persistence is intentionally short.
5. Mapping and all-system process handoffs are success-only. A non-zero/crashed motion-critical gate cannot start the next stage.
6. Mapping manual commands are fail-closed through `/system/manual_motion_allowed`, which requires fresh LiDAR safety health and `/esc/ready`.
7. ESC mux applies an additional commissioning-speed cap to manual sources when not in explicit standalone/service mode.
8. TTY keyboard fallback is disabled when physical dead-man is required. Global evdev `RIGHT CTRL` remains the supported keyboard dead-man.

## Runtime contract

```text
/scan_safety -> lidar_safety_health ----+
                                         +-> manual_motion_health
/esc/ready -----------------------------+       |
                                                 +-> /system/manual_motion_allowed

keyboard / joystick / remote / GUI
              |
              v
          ESC command mux <---- manual gate freshness
              |
              +---- blocked => zero output
              v
       /cmd_vel/actuator
```

Autonomous motion continues to use the independent `/system/autonomy_motion_allowed` gate.

## Required stationary/raised-wheel validation

Before driving on the floor:

```bash
python3 src/navigation/tools/verify_part5_stage2_mapping_manual.py
ros2 launch navigation mapping_runtime.launch.py
```

Verify:

```bash
ros2 topic echo /system/manual_motion_allowed
ros2 topic echo /system/manual_motion_health
ros2 topic echo /esc/mux/status
ros2 topic echo /cmd_vel/actuator
```

Expected startup state is `manual_motion_allowed=false` until LiDAR safety is healthy and ESC feedback is ready. After both remain healthy for the configured recovery interval, the gate may become true.

### Fault injection

While holding keyboard/joystick dead-man at very low commissioning speed:

- unplug/stop LiDAR -> gate false -> actuator command zero;
- make `/lidar/safety_healthy=false` -> gate false -> actuator command zero;
- stop ESC telemetry -> `/esc/ready` stale/false -> gate false -> actuator command zero;
- kill `manual_motion_health` -> mux gate freshness expires -> actuator command zero;
- release keyboard/joystick dead-man -> command zero independent of gate;
- engage E-stop -> command zero independent of every other gate.

`/esc/mux/status` must show a source such as `keyboard_blocked` / `joystick_blocked` when a manual source is requesting ownership but the manual gate is not valid.

## Thin-obstacle and self-filter validation

Stage 2 LiDAR safety from the previous release remains mandatory. Test a thin pole / pallet leg in front and rear while moving at the lowest possible commissioning speed. `/scan_safety` must retain the obstacle and the collision/safety path must stop the command.

## Global reroute validation

Use a saved map with at least two possible routes around a corridor. Place a large stationary obstacle/pallet in the originally preferred route.

Expected behavior:

1. `/scan_nav` marks the obstacle in `global_costmap`.
2. the obstacle is not written into the immutable `/nav_map` static map;
3. Smac replanning chooses an alternate valid known-space route when available;
4. after the obstacle is physically removed and observed by clearing rays, the dynamic obstacle layer clears it;
5. unknown map cells remain unknown and are not traversed by Smac.

Do not tune `observation_persistence` upward merely to make visualization look stable. Validate it against real scan rate and replan behavior.

## Map integrity

A new 2D Pose Estimate must never modify `/nav_map`. If a start pose is lethal because the original saved map contains robot artifacts, fix/regenerate the derived planning map rather than erasing cells continuously around AMCL pose.

## Manual speed caps

The Stage-2 defaults are commissioning limits, not certified stopping speeds:

- forward <= 0.30 m/s;
- reverse <= 0.15 m/s;
- yaw rate <= 0.35 rad/s.

These values must be revalidated with actual stopping-distance measurements before any increase.
