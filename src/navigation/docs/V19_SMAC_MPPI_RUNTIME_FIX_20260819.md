# V19 Smac Hybrid-A* + MPPI Runtime Fix

Based on `log(20260819-155149).zip`.

Observed runtime:
- SmacPlannerHybrid configured and ACTIVE; `/smac_plan` was generated (63 poses, 8.31 m).
- MPPIController configured and ACTIVE.
- Runtime failure began in MPPI CostCritic with invalid effective inflation, followed by `Optimizer fail to compute path`.
- CollisionMonitor stopped immediately in PolygonStop, whose old polygon overlapped the physical chassis footprint.

V19 changes:
1. LiDAR driver now publishes `/scan_raw`; `scan_self_filter.py` removes robot-self returns and publishes canonical `/scan`.
2. Local/global inflation fields are 0.82 m radius, scaling 8.0. For the 1.30 x 0.80 m rectangular footprint, circumscribed radius is ~0.763 m, so Humble CostCritic receives non-zero inflation cost at that radius.
3. CollisionMonitor front zones start at x=0.70 m, outside the x=0.65 m chassis front.
4. MPPI batch reduced to 800 and horizon to 24 x 0.1 s to cut Jetson controller load.
5. Real Humble MPPI visualization enabled with heavy downsampling; RViz shows `/trajectories` and `/transformed_global_plan`.
6. RViz initial pose passes through a 150 ms back-date relay to avoid AMCL future-TF warning.
7. Sensor guard startup HOLD is INFO; a real post-healthy dropout remains WARN/fail-safe.
8. EKF autonomous frequency reduced from 30 Hz to 10 Hz to fit observed Jetson processing time and still match the 10 Hz controller.

No fake plan or fake MPPI trajectory publisher is introduced.
9. Camera/YOLO startup is moved after local Nav2/MPPI readiness so CPU-heavy perception no longer competes with EKF and controller lifecycle bringup.
10. The stale fallback BT now uses the canonical `GridBased` planner; it no longer references removed `GridBasedFallback`.
