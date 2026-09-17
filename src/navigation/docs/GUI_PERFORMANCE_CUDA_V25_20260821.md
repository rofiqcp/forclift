# GUI Performance + NVIDIA/CUDA Runtime V25 — 2026-08-21

## Root cause confirmed from `log(6).zip`

`agv_gui_startup.log` repeatedly raised:

`ValueError: The truth value of an array with more than one element is ambiguous.`

The hot callback was the CameraInfo path evaluating a fixed ROS array as a Python boolean.  The bridge survived, but the repeated exception/traceback added avoidable executor and GUI load.

## Performance policy

Sensor and control ROS rates are **not reduced**.  Optimization is only applied to GUI work that does not need to execute for every source message:

- Hidden camera/YOLO/fork tabs do not convert full 1280x720 ROS images to QImage.
- The active image preview is coalesced for display only; the ROS image topics keep their native rates.
- Hidden plots stop repaint timers while their buffers continue receiving samples.
- Hidden map pages keep latest live data but do not mutate QGraphicsScene.
- LiDAR scan rendering uses one batched QPainterPath instead of hundreds/thousands of QGraphicsItems per scan.
- Subsystem tables update at a bounded UI cadence rather than rebuilding on every IMU/camera callback.
- High-bandwidth GUI image subscriptions use BEST_EFFORT/KEEP_LAST(1), so the GUI drops stale display frames instead of building a DDS backlog.
- Hole/block alignment processes every unique camera frame exactly once and publishes one annotated frame per unique camera frame in autonomous mode; no ROS topic is frequency-capped by this optimization.

## Status semantics

- `/map` and map-server output are treated as latched/static, not falsely marked stale.
- AMCL is healthy when `map->odom` TF is valid even if a stationary robot is not continuously publishing pose.
- Event-driven planner/controller topics show IDLE before a goal rather than NO DATA/error.
- Sensor Guard starts in the foundation stage instead of appearing NOT RUNNING while Nav2 is still staging.
- Hole/Block Alignment is enabled by default in autonomous launch.

## Camera + YOLO GPU policy

Camera MJPEG decode is strict NVIDIA hardware decode:

`v4l2src -> nvv4l2decoder mjpeg=1 -> nvvidconv -> appsink`

`require_gpu_decode:=true` disables CPU decode fallback and launch preflight requires `nvv4l2decoder` + `nvvidconv`.  `/camera/color/status` must contain `backend=CUDA-NVV4L2` (the label denotes the NVIDIA accelerated path; final ROS Image memory is necessarily CPU-addressable).

YOLO uses TensorRT FP16 when a valid engine is available; otherwise OpenCV DNN CUDA FP16 is selected and a real warm-up forward pass is executed. `require_cuda:=true` prevents CPU inference fallback. `/obstacle_detection/status` reports the proven backend.

## Verification after build

```bash
source /opt/ros/humble/setup.bash
source /home/otomasi2/forclift/install/setup.bash
ros2 run navigation verify_gui_cuda_v25.py
```

Expected result ends with:

`RESULT: PASS — live topics + strict NVIDIA/CUDA backends confirmed`
