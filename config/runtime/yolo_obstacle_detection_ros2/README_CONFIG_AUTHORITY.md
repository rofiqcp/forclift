# Perception Runtime Configuration Authority

Canonical runtime directory:
`/home/otomasi2/forclift/config/runtime/yolo_obstacle_detection_ros2/`

Files:
- `camera_v4l2.yaml`
- `yolo_detection.yaml`
- `alignment_realtime.yaml`

`autonomous.launch.py`, `perception_all.launch.py`, and `camera.launch.py` resolve this directory first. Package-share/source YAML files are fallback templates only and are not the tuning authority.

P0.1 unified on 20260917_040914
