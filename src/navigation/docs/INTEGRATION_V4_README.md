# AGV V4 integration (GNSS-free)

## Architecture
`/esc/odom + /imu/data -> robot_localization EKF -> odom->base_footprint`

Autonomous: `/scan + saved map -> AMCL -> map->odom -> SmacPlannerHybrid -> MPPI -> /cmd_vel_nav_raw -> velocity_smoother -> /cmd_vel_nav_smoothed -> collision_monitor(/scan) -> /cmd_vel -> esc_command_mux -> /cmd_vel/actuator -> ESC driver`

Mapping: `/scan + /odometry/filtered -> SLAM Toolbox -> /map + map->odom`.

Astra color image feeds YOLOv8 C++ detector. Astra depth + detections feed the C++ pallet pose estimator, which publishes `/pallet_detection/poses`.

## Build
```bash
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Mapping
```bash
ros2 launch navigation map.launch.py
```
Save:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/absolute/path/agv_map'}}"
```

## Autonomous
```bash
ros2 launch navigation autonomous.launch.py map:=/absolute/path/agv_map.yaml yolo_model:=/absolute/path/yolov8n_agv_forklift.onnx
```
After AMCL is active, set the initial pose in RViz, then send a Nav2 Goal.

## Key topics
- `/scan`: LiDAR C++
- `/imu/data`: IMU C++ @ 115200 baud
- `/odometry/filtered`: EKF
- `/amcl_pose`: AMCL pose
- `/plan`: Nav2 global path
- `/cmd_vel_nav_raw`: MPPI raw command
- `/cmd_vel_nav_smoothed`: smoothed command
- `/cmd_vel`: collision-monitored command entering ESC mux
- `/cmd_vel/actuator`: final ESC driver command
- `/obstacle_detection/obstacles`: YOLOv8 detections
- `/pallet_detection/poses`: pallet pose C++

## YOLO model
The uploaded YOLO archive contains code and message definitions but no `.onnx`, `.engine`, or `.pt` weight file. The launch therefore exposes `yolo_model` and `yolo_engine` arguments. Provide the exact trained model used on the AGV.


## Autonomous localization startup (2026-08-18)
`autonomous.launch.py` starts `map_server` + `nav2_amcl` immediately under a dedicated localization lifecycle manager. Planner/controller startup waits for `/amcl_pose`. RViz shows only `/obstacle_detection/visualization`; `/camera/color/image_raw` remains an internal detector input and is not displayed.

If AMCL/Nav2 binaries are missing on ROS 2 Humble, install the official Nav2 binary packages:

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
```
