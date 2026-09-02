# Autonomous map selection

`autonomous.launch.py` uses this runtime directory on the AGV:

`/home/otomasi2/ros/src/navigation/maps`

- `map:=auto` selects the newest `*.yaml` file.
- The historical placeholder `map:=/path/to/map.yaml` is treated as `auto`.
- An explicit existing YAML path is still accepted.
- The associated `.pgm` path is read from the selected YAML by `nav2_map_server`.

## Autonomous localization-only behavior

`ros2 launch navigation autonomous.launch.py map:=<saved_map.yaml>` is a localization-only launch.

- `nav2_map_server` reads the saved YAML/PGM map.
- `nav2_amcl` localizes the robot against that saved map.
- LiDAR stays active and publishes `/scan`; AMCL requires this LaserScan input.
- `/scan` is not a new map and does not modify the saved YAML/PGM.
- Hector SLAM, SLAM Toolbox, map saver, `map.launch.py`, and `mapping_runtime.launch.py` are not started.
- IMU remains active on `/imu/data`.
- `robot_state_publisher` remains active, so the URDF/RobotModel remains available in RViz.
- The LaserScan display is disabled by default in `autonomous.rviz`; the topic remains active for AMCL.
