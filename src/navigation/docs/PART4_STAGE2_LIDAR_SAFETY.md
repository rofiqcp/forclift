# PART-4 Stage-2 — Split LiDAR Safety / Navigation Pipeline

## Purpose

This stage prevents navigation-oriented scan cleanup from weakening collision safety.
A single physical LiDAR revolution now produces two explicitly different products:

```text
physical revolution
       |
       +--> scan_safety (before anti-starburst) --> self-mask --> /scan_safety
       |                                                |
       |                                                +--> lidar_safety_health
       |                                                +--> Collision Monitor
       |
       +--> anti-starburst --> scan --> self-mask --> /scan_nav
                                                        |
                                                        +--> AMCL
                                                        +--> SLAM Toolbox
                                                        +--> LiDAR odometry
                                                        +--> Nav2 obstacle layers
```

There is intentionally **no production `/scan` alias**. A consumer must choose the
safety or navigation contract explicitly.

## Safety invariants

1. `/scan_safety` is emitted before anti-starburst removal.
2. A sparse completed physical revolution is still published on `/scan_safety` so
   the health gate can hold motion immediately; that revolution is withheld from
   `/scan_nav` when below the navigation hard-density requirement.
3. Both paths remove only robot-self returns. The mask comes from the canonical
   Stage-1 vehicle footprint and must not extend beyond it.
4. Collision Monitor consumes only `/scan_safety`.
5. FrontStop and RearStop use `max_points: 0`. In the Nav2 Humble stop/slow code,
   the action triggers when points inside are greater than `max_points`, therefore
   a single in-zone beam is sufficient for STOP.
6. Collision Monitor source timeout is not treated as the sole fail-stop mechanism.
   `autonomous_sensor_cmd_guard` independently requires fresh `/scan_safety` and a
   fresh/true `/lidar/safety_healthy` state.
7. LiDAR safety health checks scan age, usable-beam density, scan rate, driver/motor
   state, and parser/checksum counter degradation. Recovery requires consecutive
   good scans.
8. Persistent runtime YAML is migrated in-place with backups. Existing unrelated
   Nav2/MPPI tuning is preserved.

## Runtime topics

| Topic | Contract | Main consumers |
|---|---|---|
| `/scan_safety_raw` | driver output before anti-starburst | safety self-mask only |
| `/scan_safety` | minimally filtered, robot-self masked | Collision Monitor, LiDAR health, command guard freshness |
| `/scan_nav_raw` | anti-starburst navigation product | nav self-mask only |
| `/scan_nav` | filtered, robot-self masked | AMCL, SLAM, LiDAR odometry, costmaps |
| `/lidar/safety_healthy` | latched quality Bool | final autonomous sensor command guard |
| `/lidar/safety_health` | JSON diagnostics | GUI, experiment logger, operator diagnostics |

## Required physical tests on Jetson / AGV

Do these at low speed with the vehicle restrained or with a spotter and an
accessible hardware E-stop. Static source validation cannot prove physical stop
performance.

### A. Stream separation

```bash
ros2 topic hz /scan_safety
ros2 topic hz /scan_nav
ros2 topic echo /lidar/safety_health --once
```

Expected: both scan streams are live; health becomes true only after parser/status
warm-up and consecutive good scans.

### B. Thin-obstacle test

Place a thin pole / pallet leg in each stop polygon. Verify that even one retained
safety beam causes Collision Monitor output to zero while the obstacle can still be
visible if anti-starburst rejects it from `/scan_nav`.

### C. Sparse-data hold

Temporarily obstruct/degrade the LiDAR so a complete scan has very few usable
returns. `/lidar/safety_healthy` must go false without waiting for `/scan_safety`
to become stale, and `/cmd_vel/actuator` must be zero through the sensor guard.

### D. Disconnect / motor-stop test

Disconnect the LiDAR or stop its motor. The health state must become false and
motion must remain blocked. Reconnection must require consecutive good scans before
health returns true.

### E. Self-mask boundary test

Put an obstacle immediately outside every physical chassis side. It must remain in
`/scan_safety`; only returns from the robot footprint itself may be masked.

### F. Navigation regression

Map/localize using `/scan_nav` and verify AMCL, SLAM Toolbox, LiDAR odometry and
costmaps receive no `/scan_safety` input.

## Verification

```bash
python3 src/navigation/tools/verify_part4_stage2_lidar_safety.py
```

This is a source/config/migration verifier, not a substitute for the physical tests
above.
