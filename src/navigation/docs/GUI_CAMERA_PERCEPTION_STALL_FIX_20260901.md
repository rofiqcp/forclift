# GUI Camera / Perception Stall Fix — 2026-09-01

Scope: GUI image preview path only. Navigation, AMCL, Smac, MPPI, LiDAR, IMU, ESC, winch, and perception algorithms are unchanged.

## Changes

1. Image frame counters now advance only when a new ROS image reaches the GUI, not on every Qt render-timer tick.
2. The preview reports `IMAGE STALE` when the most recently received GUI image is older than 1.5 seconds.
3. Only one full-resolution image topic is subscribed by the GUI for the active perception page:
   - Camera / Camera-Fork Calibration: `/camera/color/image_raw`
   - YOLO Detection / Perception Validation: `/obstacle_detection/visualization`
   - Hole-Block Alignment: `/fork_alignment/image`
4. `sensor_msgs/Image -> QImage` conversion is moved out of the rclpy callback thread into a dedicated latest-only worker (`agv-gui-image`) capped to about 15 FPS.
5. The worker drops superseded queued frames instead of building an image backlog.

## Expected behavior

If an upstream camera or perception publisher actually stops, the visible frame counter stops and the page changes to `IMAGE STALE` instead of repeatedly redrawing the old frame as though it were live.

If the upstream publisher is healthy, image conversion work no longer blocks the ROS bridge callback loop, reducing GUI-side pressure on TF, telemetry, camera, and perception callbacks.
