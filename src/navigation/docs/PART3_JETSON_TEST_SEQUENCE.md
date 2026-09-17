# PART 3 Jetson Test Sequence

Use this order after installing the revised source tree on the AGV.

## A. Static gates

```bash
cd /home/otomasi2/forclift
python3 src/navigation/tools/verify_part1_core.py
python3 src/navigation/tools/verify_part2_gui_config.py
python3 src/navigation/tools/verify_part3_gpu_reporting.py
```

Expected: PART 1 = 0 failures (steering calibration may intentionally warn), PART 2 = 0 failures, PART 3 = 0 failures.

## B. Build

```bash
colcon build --symlink-install --packages-select navigation esc yolo_obstacle_detection_ros2
source install/setup.bash
```

## C. TensorRT engine

```bash
bash src/yolo_obstacle_detection_ros2/tools/build_yolo_tensorrt_engine.sh \
  "$AGV_WS/models/yolov8n_agv_forklift.onnx" \
  "$AGV_WS/models/yolov8n_agv_forklift.engine"
```

Check that the build and benchmark finish successfully and that `.sha256` and `.buildinfo.txt` are produced.

## D. Perception runtime

Start the normal perception launch and verify:

```bash
ros2 topic echo /obstacle_detection/status --once
ros2 topic hz /obstacle_detection/performance
ros2 topic echo /obstacle_detection/performance --once
```

The performance line should identify the backend (`TensorRT_FP16` or fallback), FPS, latency, frame age, and frame counters.

## E. GUI experiment

1. Start the GUI.
2. Open **Experiments**.
3. Keep **Record rosbag2** and **Record Jetson CPU/RAM/GPU/temperature/power** enabled.
4. Select subsystem and map.
5. Start recording.
6. Execute the tuning route/test.
7. Stop with the correct result (PASS/FAIL/ABORTED/COMPLETED).
8. Open **Reports** and inspect the generated report.

## F. Navigation metrics

For an MPPI route, verify that `metrics.yaml` receives values for:

- `navigation.cross_track_error_m`
- `navigation.cross_track_rmse_m`
- `navigation.heading_error_rad`
- `navigation.heading_error_rmse_rad`
- `navigation.final_goal_error_m`
- `control.drive_speed_rmse_mps`
- `control.steering_rmse_rad`
- `control.command_stage_latency_ms`

For perception, verify FPS/latency and Jetson GPU statistics are populated.

## G. Physical safety gate

Do not bypass `steering_calibrated=false`. Perform real steering zero/direction/ticks-per-radian calibration first, save it through the calibration workflow, re-run the PART 1 verifier, and only then allow physical autonomous motion.
