# Final bringup fix 2026-08-24

Changes in this revision:
- autonomous LiDAR/IMU drivers start independently after preflight and keep reconnecting; resolver/transport gates are diagnostic rather than one-shot publication blockers.
- mapping STOP+SAVE stores a SHA256-bound `map_N.autopose.json`; autonomous reuses it only for the exact saved map, otherwise remains in safe operator initial-pose mode.
- controller/behavior/BT navigator/velocity smoother/collision monitor processes are created early and lifecycle activation remains gated by AMCL/costmap readiness.
- manual motion health is present in autonomous bringup.
- autonomous/mapping perception uses standalone camera/YOLO processes; camera and detector respawn independently.
- Mapping GUI starts `mapping_runtime.launch.py enable_rviz:=false`; `map.launch.py` is the single RViz owner.
- map saver writes explicit PGM/trinary/free/occupied options.
- GUI TF status distinguishes local TF healthy but AMCL initial pose pending.
