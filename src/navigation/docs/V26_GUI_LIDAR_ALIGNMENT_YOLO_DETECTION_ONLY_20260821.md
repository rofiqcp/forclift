# V26 — GUI plot, LiDAR continuity, Hole-Block image, YOLO detection-only

## Scope

V26 addresses the runtime symptoms observed on 2026-08-21 without reducing ROS sensor/control publish rates.

### GUI plot
- Realtime plots now use the actual oldest available sample during warm-up so new data fills the canvas instead of appearing only on the right side.
- Ring buffers hold up to 7200 samples per signal (60 s at 120 Hz) and remain time-pruned.
- QPainter uses one `drawPolyline()` per signal with display-only decimation.
- Mixed-unit plots automatically use per-signal vertical scaling when one unit would otherwise flatten the others.
- `/imu/gyro` and `/imu/accel` still participate in full-rate topic health, but no longer duplicate the accel/gyro values already provided by `/imu/data` into the plot buffer.

### LiDAR
- SerialPort ownership is protected across reader, watchdog and reconnect worker.
- Connector lifecycle transitions are serialized, preventing concurrent reopen/replacement races.
- Recovery callbacks no longer destroy/join their own reconnect worker.
- A completed real scan changes the motor diagnostic state back to `RUNNING`.
- Moderately sparse valid revolutions are published with `+inf` for missing beams instead of making `/scan` disappear.
- Almost-empty/corrupt revolutions remain fail-closed: they are not published and the published-scan watchdog triggers recovery.
- Watchdog evaluates both physical-revolution freshness and `/scan` publication freshness.

### Hole-Block Alignment
- `gui.launch.py` now enables the alignment node by default, matching `autonomous.launch.py`.
- The GUI treats Fork Alignment as a required subsystem rather than silently reporting `DISABLED` when it is missing.
- `AlignmentState.state`, `state_text` and `data_valid` are populated correctly on every processed camera frame.
- `/fork_alignment/image` remains an annotated preview generated from real camera frames.

### YOLOv8
- YOLO visualization is object-detection-only: bounding box, class, confidence and compact detector status.
- Removed the old center docking box, path-lane overlay, alignment arrows and `/docking/state` publisher from `obstacle_detector_node`.
- Object metadata `/obstacle_detection/obstacles` is retained because Hole-Block Alignment and perception/safety logic consume detections.
- CUDA/TensorRT enforcement from V25.1 remains active.

## Runtime verification

After a clean build and launch:

```bash
ros2 run navigation verify_gui_perception_v26.py
```

The verifier checks NVIDIA camera decode, CUDA/TensorRT YOLO, live camera/IMU/LiDAR/EKF/YOLO topics, live Hole-Block image/state, LiDAR scan continuity, required nodes, and confirms there is no `/docking/state` publisher.
