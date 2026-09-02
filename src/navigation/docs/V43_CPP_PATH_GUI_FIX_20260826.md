# V43 C++ Goal-Path + GUI Integration

## Runtime converted to C++

The production GUI and the Goal Pose / LiDAR-filter navigation path no longer use the old PyQt/Python nodes:

- `agv_gui_cpp` — native Qt5/C++ system dashboard
- `mapping_gui_cpp` — native Qt5/C++ mapping control
- `goal_pose_nav2_bridge` — C++ `ComputePathToPose` + `NavigateToPose` bridge
- `scan_self_filter` — C++ chassis mask + anti-starburst scan filter
- `navigation_runtime_validator` — C++ live validation utility

Removed production Python entry points:

- `scripts/agv_gui.py`
- `scripts/mapping_gui.py`
- `scripts/goal_pose_nav2_bridge.py`
- `scripts/scan_self_filter.py`
- `python/navigation_gui/`

ROS 2 `.launch.py` files and Python configuration/diagnostic helpers remain where the Humble bringup still depends on them. They are not the GUI, planner, controller, LiDAR driver, IMU driver, goal bridge, or scan filter. The supplied `src_NAV2_FIXED_COMPLETE` reference also contains Python launch/support files.

## Goal Pose dataflow

`/goal_pose` -> `goal_pose_nav2_bridge` -> `/compute_path_to_pose` (`GridBased`) -> Smac Hybrid-A* -> `/smac_plan` -> `/navigate_to_pose` -> BT Navigator -> MPPI.

The bridge keeps a goal submitted while PlannerServer is activating, validates the full physical footprint, optionally snaps an invalid click to nearby free space, retries planner requests, and uses a watchdog so a lost action response cannot leave the GUI waiting forever.

## LiDAR / map noise hardening

- LiDAR protocol checksum is strict by default.
- malformed packet/ring progression checks stay in the C++ driver.
- navigation scan is capped at 5.5 m.
- C++ scan filter applies chassis masking plus spatial/temporal isolated-return rejection.
- temporal history stores the already-filtered frame so a repeated starburst cannot validate itself.
- SLAM/AMCL ranges are synchronized to the navigation scan.

Existing maps that already contain radial/starburst artifacts are not rewritten automatically. Rebuild the affected map after installing V43.

## Geometry

- Full collision footprint remains 1.30 x 0.80 m.
- Ackermann minimum turning radius remains 2.0 m because `wheelbase=0.70 m` and `max steering=0.34 rad` imply about 1.98 m.
- Inflation is 0.45 m for both global and local costmaps; this removes the previous 0.77 m excessive halo without falsifying the physical footprint.
