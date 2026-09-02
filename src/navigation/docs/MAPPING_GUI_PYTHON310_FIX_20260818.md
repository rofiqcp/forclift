# Mapping GUI Python 3.10 launcher fix — 2026-08-18

Observed failure from runtime log:

- `mapping_gui.py` starts and immediately dies importing `rclpy`.
- Python standard library is loaded from uv CPython 3.11.
- ROS 2 Humble `rclpy` belongs to Ubuntu system Python 3.10.
- Consequently `mapping_runtime.launch.py` is never started because the GUI owns START/STOP.

Fix scope is intentionally limited to GUI startup:

1. `mapping_gui.py` shebang pinned to `/usr/bin/python3`.
2. Added `mapping_gui_launcher.sh` to force `/usr/bin/python3` and filter uv/Python 3.11 environment contamination.
3. `map.launch.py` launches the wrapper.
4. CMake installs the wrapper.

No LiDAR, IMU, Hector, SLAM Toolbox, scan generation, USB role, TF, EKF, RViz config, or mapping-runtime algorithm was changed.
