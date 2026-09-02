# Camera / Perception FPS Optimization — 2026-09-01

Scope: GUI camera preview and RGB perception only. Navigation, AMCL, Smac, MPPI,
LiDAR, IMU, ESC, and winch behavior are unchanged.

## Runtime profile

- Camera capture target: 1280x720 MJPG @ 30 FPS.
- NVIDIA `nvv4l2decoder` remains preferred for MJPG decode.
- YOLO inference ceiling: 30 FPS. The existing latest-frame-only inference queue
  prevents backlog when the backend is slower than the camera.
- YOLO annotated visualization ceiling: 20 FPS. This intentionally leaves GPU/CPU
  headroom for inference and navigation instead of spending all resources drawing
  and republishing 1280x720 HUD frames.
- GUI image conversion/render ceiling: 30 FPS, latest frame only.
- Qt 5.15 `Format_BGR888` is used when available so the Astra `bgr8` stream does
  not require a full-frame `rgbSwapped()` operation before display.

## TensorRT selection fix

Autonomous launch now prefers an auto-selected ONNX model that has a matching
`.engine` file when `use_tensorrt:=true`. This avoids silently selecting
`yolov8n_agv_forklift_opencv.onnx` while an already-built
`yolov8n_agv_forklift.engine` is available.

The package intentionally does not build a TensorRT engine automatically during
launch. Engines are target-Jetson/TensorRT/CUDA specific. Use the supplied
`build_yolo_tensorrt_engine.sh` on the AGV Jetson if the engine is absent.

## Expected interpretation

- Camera page can approach 30 FPS when the UVC device negotiates 1280x720@30 MJPG.
- YOLO/Perception preview is intentionally bounded at 20 FPS.
- Detection rate can only approach 30 FPS when the selected inference backend is
  fast enough. If the status reports `OpenCV DNN CUDA`, the actual detection rate
  may be below the ceiling; `TensorRT FP16` is the preferred high-throughput path.
