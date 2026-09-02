# V13 — Smac Goal Path + Humble no-error fixes

Fixes derived from `starburst_fix_20260819_172258.txt`:

- ROS Python runtime forced to `/usr/bin/python3` (Ubuntu 22.04 / ROS Humble Python 3.10).
- `local_costmap.width` / `height` are integers, as required by Humble.
- Global/local inflation radius is 0.40 m, exactly the 0.40 m inscribed radius of the 1.30 x 0.80 m footprint.
- `footprint_padding=0.0`; high cost scaling keeps the halo compact.
- Full Humble BT plugin library list restored, including `nav2_remove_passed_goals_action_bt_node`.
- BT Navigator direct `goal_pose` subscriber remapped away from RViz. `/goal_pose` is owned only by the Smac preview bridge.
- `allsystem_gate` sanitizes hyphenated gate names before constructing ROS topic names.
- Python-based readiness gates and goal bridge no longer inherit an incompatible Python 3.11 interpreter.
- Persistent workspace sourcing + log-directory helper scripts added under `navigation/tools/`.
