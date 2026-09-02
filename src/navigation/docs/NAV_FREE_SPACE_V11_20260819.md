# SUPERSEDED BY V12

This V11 note is retained for history. The active autonomous configuration is V12: global inflation 0.25 m, local inflation 0.35 m, with exact polygon footprint collision checking and explicit Smac goal-path preview on `/smac_plan`. See `NAV_SMAC_GOAL_PATH_V12_20260819.md`.

# V11 Autonomous Global Costmap — Free-Space Fix (2026-08-19)

## Problem reproduced from the uploaded runtime

The latest autonomous map is only 125 x 123 cells at 0.05 m/cell and contains many
small isolated occupied components. With the previous 0.50 m global inflation radius,
every isolated speckle generates a large inflated island, so the islands overlap and
leave very little visually clear planning space.

## Fix used in V11

1. `/map` remains the raw saved map used by AMCL. It is never modified.
2. `prepare_nav_map.py` creates a planning-only copy on startup and removes only tiny
   isolated occupied connected-components. Output is written under
   `/tmp/navigation_autonomous_navmap` and published as `/nav_map`.
3. The GLOBAL costmap StaticLayer subscribes to `/nav_map` instead of `/map`.
4. The GLOBAL footprint remains the real AGV rectangle 1.30 x 0.80 m, but padding is
   0.00 m and inflation is 0.40 m (the physical half-width / inscribed clearance).
5. The LOCAL costmap stays conservative and live: `/scan` ObstacleLayer + 0.50 m
   inflation + 0.05 m footprint padding.
6. `verify_autonomous_freespace.py` checks the actual runtime topics and returns PASS
   only when `/map`, `/nav_map`, and the global costmap are present and the global
   costmap contains usable free space.

This split is deliberate: the planner is no longer dominated by old mapping speckles,
while the local controller continues to protect against real/current LiDAR obstacles.

## Build and run

```bash
cd /home/otomasi2/ros
rm -rf build/navigation install/navigation
colcon build --packages-select navigation --symlink-install
source install/setup.bash
ros2 launch navigation autonomous.launch.py
```

The launch should print a line similar to:

```text
[AUTONOMOUS-NAV-MAP] raw=/map; planning=/nav_map; removed_cells=...; removed_components=...
```

## Runtime verification

In a second terminal:

```bash
cd /home/otomasi2/ros
source install/setup.bash
ros2 run navigation verify_autonomous_freespace.py
```

Expected final line:

```text
PASS: /nav_map exists and global costmap contains usable free space.
```

Also verify the loaded parameters:

```bash
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /global_costmap/global_costmap footprint_padding
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap footprint_padding
```

Expected values:

```text
global inflation = 0.40
global padding   = 0.0
local inflation  = 0.50
local padding    = 0.05
```

## Launch-time tuning knobs

Default values are selected for this uploaded map. If a future map is already clean,
the planning filter can be disabled without changing AMCL:

```bash
ros2 launch navigation autonomous.launch.py enable_nav_map_filter:=false
```

The tiny-component filter can also be made more/less aggressive:

```bash
ros2 launch navigation autonomous.launch.py nav_map_filter_max_cells:=3
```

V11 historical note: the old recommendation was not to reduce GLOBAL inflation below 0.40 m unless the physical footprint and
collision-checking assumptions are intentionally revalidated on the real AGV.
