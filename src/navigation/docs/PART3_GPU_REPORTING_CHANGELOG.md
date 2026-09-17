# PART 3 — Jetson GPU, Experiment Recorder, Metrics, and Reporting

Date: 2026-08-22

PART 3 is applied on top of the validated PART 2 tree. It intentionally does not replace the PART 1 localization/TF safety architecture or the PART 2 canonical runtime-configuration workflow.

## 1. TensorRT / CUDA hardening

- TensorRT detector input/output tensor names and shapes are discovered from the serialized engine at runtime.
- Removed the fixed `images`, `output0`, and `8400` binding/grid contract.
- The application class contract remains explicit: five AGV classes, therefore a detector head with `4 + 5 = 9` channels is required. Incompatible exports are rejected and fall back safely.
- TensorRT named-I/O path is enabled only for TensorRT >= 8.5. Older installations compile the OpenCV CUDA fallback rather than failing at build time.
- Persistent device and page-locked host buffers replace large per-frame stack buffers.
- Optional CUDA preprocessing performs BGR -> RGB, letterbox resize, normalization, and NCHW conversion on the GPU when nvcc is available.
- If nvcc is unavailable, TensorRT remains usable with persistent pinned CPU preprocessing.
- CUDA API errors and `enqueueV3()` failures are checked.
- Partially initialized TensorRT/CUDA resources are released before falling back to OpenCV CUDA.
- Perception publishes `/obstacle_detection/performance` with backend, FPS, stage latency, frame age, detection count, and frame receive/process/drop counters.

## 2. Target-side engine build

Installed tool:

```bash
yolo_obstacle_detection_ros2/tools/build_yolo_tensorrt_engine.sh
```

Build the `.engine` on the target Jetson, not on another GPU machine:

```bash
export AGV_WS=/home/otomasi2/forclift
bash "$AGV_WS/src/yolo_obstacle_detection_ros2/tools/build_yolo_tensorrt_engine.sh" \
  "$AGV_WS/models/yolov8n_agv_forklift.onnx" \
  "$AGV_WS/models/yolov8n_agv_forklift.engine"
```

The script builds FP16, benchmarks with `trtexec`, and writes SHA256/build metadata next to the engine.

## 3. Experiment recorder

`Experiments -> Start Recording` can now record:

- selected ROS 2 raw evidence through rosbag2;
- exact runtime YAML snapshots and SHA256 hashes;
- ROS timestamp, wall clock, and monotonic timestamps;
- GUI-decimated telemetry by subsystem;
- critical topic health/rate/age traces;
- Jetson CPU/RAM/load/thermal metrics;
- Jetson `tegrastats` GPU utilization, GPU temperature, and input power when available.

A completed session contains approximately:

```text
<session>/
├── metadata.yaml
├── metrics.yaml
├── config_snapshot/
├── raw/
│   ├── rosbag2/
│   ├── rosbag2_console.log
│   └── tegrastats.log
├── csv/
│   ├── raw_data.csv
│   ├── esc.csv
│   ├── navigation.csv
│   ├── yolo_performance.csv
│   ├── system.csv
│   └── topic_health.csv
├── plots/
└── report/report.html
```

## 4. Automatic metrics

Current report metrics include, when the corresponding data exists:

- LiDAR valid-range ratio and scan-match quality;
- EKF and AMCL covariance summaries;
- planning time, path length, curvature, reverse segments;
- distance-to-goal and final goal error;
- path cross-track error and RMSE;
- path-heading error and RMSE;
- command-pipeline stage latency;
- target-vs-actual drive-speed RMSE;
- target-vs-actual steering RMSE;
- safety guard / motion permission / E-stop traces;
- perception FPS and preprocess/inference/postprocess/total latency;
- camera-to-inference frame age;
- CPU, GPU, RAM, thermal, and input-power summaries;
- critical topic observed ratio, rate, and age.

No arbitrary academic PASS/FAIL thresholds are invented by the software. The operator records the test result, while quantitative metrics remain available for thesis acceptance criteria.

## 5. Report UI

The GUI now has a real Reports page to:

- browse recorded sessions;
- inspect key metrics;
- regenerate `report.html` from saved CSV data;
- open the selected HTML report.

The HTML report contains the final test result, start/end information, rosbag status, operator note, calculated metrics, plots, and config snapshot SHA256 values.

## 6. Validation

Run on the project source tree:

```bash
python3 src/navigation/tools/verify_part1_core.py
python3 src/navigation/tools/verify_part2_gui_config.py
python3 src/navigation/tools/verify_part3_gpu_reporting.py
```

Then build on the Jetson:

```bash
cd /home/otomasi2/forclift
colcon build --symlink-install --packages-select navigation esc yolo_obstacle_detection_ros2
source install/setup.bash
```

PART 3 cannot replace target-side validation of ROS 2 Humble, the installed JetPack/TensorRT/CUDA stack, camera transport, LiDAR/IMU/ESC serial hardware, or physical vehicle motion.
