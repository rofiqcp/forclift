# V22 Camera + Local Costmap Fix — 2026-08-20

## Runtime evidence from log(20260819-185534).zip

- Astra hardware was healthy: `/camera/color/image_raw` reached camera gate READY and frames continued publishing.
- RViz displayed `/obstacle_detection/visualization`, but YOLO was only started after `LOCAL-INFLATION` READY. Therefore controller failure prevented the processed image publisher from ever starting.
- `controller_server` failed during configure with: `Controller period more then model dt, set it equal to model dt`.
- V21 configured `controller_frequency: 5.0` (0.20 s period) with MPPI `model_dt: 0.1`, which Nav2 rejected.
- Local costmap was additionally hidden in RViz because `LocalCostmap + Inflation` had `Enabled: false`.

## V22 changes

1. Keep controller at 5 Hz and set MPPI `model_dt: 0.2`; use `time_steps: 15` for a 3.0 s horizon.
2. Set autonomous EKF to 5 Hz so odometry cadence matches the controller cadence.
3. Start YOLO and hole-alignment after the real camera gate exits READY, independent of ControllerServer/local costmap.
4. Enable `LocalCostmap + Inflation` in `autonomous.rviz`.
5. Add `verify_v22_camera_local_costmap.sh` to verify raw camera, processed camera, controller lifecycle, local costmap, footprint, and installed configuration.
