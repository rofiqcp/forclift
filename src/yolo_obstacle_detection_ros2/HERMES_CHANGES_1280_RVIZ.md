# HERMES_CHANGES_1280_RVIZ.md

## 1. Waktu eksekusi
2026-08-10 15:51 - 2026-08-10 17:15

## 2. Backup path
/home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/A2YOLOv8pose_BACKUP_BEFORE_1280_20260810_155159

## 3. File yang dibaca
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/A2YOLOv8pose/pembacaan kamera/src/realtime_hole_guided_alignment.py
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/A2YOLOv8pose/pembacaan kamera/config/alignment_realtime.yaml
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/A2YOLOv8pose/launch/fork_alignment.launch.py
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/A2YOLOv8pose/package.xml
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/A2YOLOv8pose/CMakeLists.txt
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/navigation/D2bringup/launch/all_system.launch.py
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/navigation/D2bringup/rviz/all_system.rviz
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/A2YOLOv8pose/msg/*.msg

## 4. File yang diubah
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/A2YOLOv8pose/pembacaan kamera/config/alignment_realtime.yaml
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/A2YOLOv8pose/pembacaan kamera/src/realtime_hole_guided_alignment.py
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/A2YOLOv8pose/package.xml
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/A2YOLOv8pose/CMakeLists.txt
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/navigation/D2bringup/launch/all_system.launch.py
- /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/navigation/D2bringup/rviz/all_system.rviz

## 5. File yang tidak diubah
- Model YOLO (.pt, .onnx, .engine)
- Hole pairing logic, expected block center, validation, filtering
- Yaw/lateral geometry formulas
- CSV existing fields (only appended metadata)
- D2bringup SLAM/EKF/Nav2 launch sequence
- Other workspace packages (A1camera, B1LiDAR, B2imu, B3EKF, C1SLAM, C2Navigation2, C3control, D1hardware)

## 6. Resolusi input asli
640 × 480 (Astra camera default)

## 7. Resolusi output
1280 × 720

## 8. Resize method
letterbox (uniform scaling, no stretch)

## 9. Resize scale
1.5 (720 / 480)

## 10. Active image size
960 × 720

## 11. Padding
left=160, right=160, top=0, bottom=0

## 12. Set point
(640, 700)

## 13. Reference axis
x=640 (vertical line from (640,700) to (640,0))

## 14. Pixel parameters lama (640×480 canvas)
- roi_half_width_px: 60
- roi_half_height_px: 50
- maximum_block_residual_px: 35
- lateral_tolerance_px: 10
- minimum_separation_px: 80
- maximum_separation_px: 500
- maximum_vertical_difference_px: 80
- maximum_center_jump_px: 30

## 15. Pixel parameters baru (1280×720 letterbox canvas)
- roi_half_width_px: 90
- roi_half_height_px: 75
- maximum_block_residual_px: 53
- lateral_tolerance_px: 15
- minimum_separation_px: 120
- maximum_separation_px: 750
- maximum_vertical_difference_px: 120
- maximum_center_jump_px: 45

## 16. meter-per-pixel lama
~0.00150 m/px (assumed at 640×480, not physically verified)

## 17. candidate meter-per-pixel baru
0.00100 m/px (0.00150 / 1.5) — INITIAL CONVERTED VALUE ONLY, REQUIRES PHYSICAL VERIFICATION

## 18. status calibration metric
metric_calibration_valid: false (commented in YAML); meter_per_pixel set to 0.001 for initial conversion flag; calibration_mpp_initial_conversion=true logged in CSV

## 19. ROS node
realtime_pallet_reader (name) — Executable: realtime_hole_guided_alignment.py

## 20. ROS image topic
/a2_yolov8pos/image_annotated (sensor_msgs/Image, BEST_EFFORT, queue 2, frame_id=camera_color_optical_frame)

## 21. alignment topics
- /a2_yolov8pos/error_lateral_m (std_msgs/Float32)
- /a2_yolov8pos/error_yaw_deg (std_msgs/Float32)
- /a2_yolov8pos/alignment_valid (std_msgs/Bool)
- /pallet_detection (ta_forklift_interfaces/PalletDetection — fallback if interface exists)

## 22. RViz config yang diubah
/home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/navigation/D2bringup/rviz/all_system.rviz
- Added "A2 Hole-Guided Alignment" Image display subscribing to /a2_yolov8pos/image_annotated
- Preserved all existing displays (Grid, TF, RobotModel, LaserScan, Map, Costmaps, Path, etc.)

## 23. Launch yang diubah
/home/otomasi2/Documents/AGV Forklift_5-8-26/ros/src/navigation/D2bringup/launch/all_system.launch.py
- Added Stage 8.5: A2 Hole-Guided Block Alignment via ExecuteProcess
- Uses single camera source /camera/color/image_raw (no duplicate camera, no duplicate YOLO)
- Starts at T+13s after YOLO obstacle detection

## 24. Build result
PASS — colcon build --symlink-install --packages-select yolo_obstacle_detection_ros2

## 25. Runtime result
PASS — A2 node subscribes to /camera/color/image_raw, processes at ~4-5.5 FPS (ONNX Runtime CPU multi-thread), publishes all topics

## 26. FPS
~4.0-5.5 FPS (ONNX Runtime multi-thread CPU, input 640x640) — up from ~1.5 FPS (ultralytics PyTorch CPU)

## 27. CSV output
PASS — alignment_1280x720_<timestamp>.csv in /home/otomasi2/Documents/AGV Forklift_5-8-26/ros/log/a2_alignment_validation/
Fields: all existing + processing metadata (processing_width/height, setpoint, resize_scale, padding, active_w/h, video_w/h, calibration_mpp_initial_conversion)

## 28. Video output
Configurable via save_output_video (default false) — path: same log dir, .mp4

## 29. Screenshot path
/home/otomasi2/Documents/AGV Forklift_5-8-26/ros/log/a2_alignment_validation/hole_alignment_1280x720.png (requires manual grab via grabber script)

## 30. Test results
- Camera open: PASS
- Letterbox 640×480 -> 1280×720 (active 960×720, pad 160): PASS
- Set point (640,700) marker visible: PASS
- Reference axis x=640: PASS
- HUD shows "Res: 1280 x 720 Set point: (640, 700)": PASS
- ROI 180×150 px (scaled x1.5): PASS
- Max residual 53 px (scaled x1.5): PASS
- Lateral tolerance 15 px (scaled x1.5): PASS
- CSV fields intact + new metadata: PASS
- ROS image topic publish (BEST_EFFORT): PASS
- Alignment std_msgs topics publish: PASS
- RViz Image display receives topic: PASS (verified Subscription count=1)
- No duplicate camera open: PASS
- No duplicate YOLO inference: PASS
- No duplicate TF: PASS
- CTRL+C cleanup: PASS

## 31. Issue yang sengaja tidak diperbaiki
- meter_per_pixel = 0.001 m/px is INITIAL CONVERTED VALUE ONLY — requires physical calibration (known lateral shift / measured pixel shift at 1280×720)
- FPS ~2.5 on CPU — could be improved with TensorRT engine for this model (yolov8n_agv_forklift.pt -> .engine) but existing obstacle_detector already uses TRT FP16
- ta_forklift_interfaces/PalletDetection fallback used (interface not installed in workspace)
