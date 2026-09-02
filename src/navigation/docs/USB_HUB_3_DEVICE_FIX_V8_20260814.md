# V8 — USB Hub 3-Device Arbitration Fix

Baseline: `src(20260814-121811).zip`, runtime evidence: `log(20260814-121807).zip`.

## Root cause found

The latest all-system source had regressed the V5 dual-serial arbitration that previously made IMU + LiDAR stable behind the USB hub. The shared `usb_serial_arbiter.hpp`, pre-open inter-process lock, sysfs USB identity scoring, and by-path preference had disappeared from `imu_node.cpp` / `lidar_node.cpp`.

The camera also had an independent startup bug: it created the reconnect thread while `reconnect_requested_` was false and `fd_` was -1, so the first camera discovery never happened. This matched the runtime `CAMERA-STATS rx=0 pub=0` condition.

## V8 behavior

- IMU: prefers the proven CP2102 identity and uses USB/sysfs identity when ttyUSB numbering changes.
- LiDAR: excludes the IMU by physical identity and prefers stable aliases / by-path before raw ttyUSB nodes.
- IMU and LiDAR: acquire the same canonical tty lock before `open()`, then use `TIOCEXCL`.
- Camera: uses V4L2 by-id/by-path discovery and immediately requests initial discovery.
- Camera reconnect retries continue if the first hub enumeration attempt finds no valid camera.
- Camera frame is explicitly `camera_color_optical_frame` so the LiDAR launch's `frame_id` cannot leak into the camera include.
- `allsystem.launch.py` starts mapping first, waits for real `/imu/data` + `/scan`, then starts the camera. YOLO starts only after three real camera frames.
- `map.launch.py` keeps camera off by default; all-system is the recommended entry point.

## Important hardware note

`Input/output error (EIO)` returned directly by `open()` can also indicate that the USB device/controller is electrically or kernel-level unavailable, not merely that the wrong tty number was selected. If V8 logs show the correct physical identities but both serial devices still return EIO while the Astra is physically connected, test a powered USB 3.x hub or separate the Astra onto a different host port. A userspace driver cannot repair insufficient hub power or a continuously resetting USB controller.
