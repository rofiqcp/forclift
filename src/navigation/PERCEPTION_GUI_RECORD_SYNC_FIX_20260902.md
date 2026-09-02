# Perception GUI Record/Graph Sync Fix — 2026-09-02

Masalah utama: `gui.launch.py` menjalankan GUI native C++ (`navigation/agv_gui`), tetapi bridge C++ masih menunggu API lama `/perception/*` dan `/camera/yolop/image_annotated`. Runtime yang benar-benar diluncurkan adalah `yolo_obstacle_detection_ros2/perception_all.launch.py`, yang menerbitkan `/camera/color/*` dan `/obstacle_detection/*`. Akibatnya recorder aktif tetapi TelemetryStore untuk halaman Persepsi kosong.

Perbaikan ini:
- subscribe `/obstacle_detection/obstacles` (typed `ObstacleArray`) -> `raw_detections.*` dan `obstacle_metrics.*`;
- subscribe `/obstacle_detection/performance` memakai BEST_EFFORT -> `perception_performance.*`;
- normalisasi `total_ms -> mean_ms` dan `dropped -> capture_dropped`;
- subscribe `/camera/color/status` -> status/connected/healthy;
- hitung statistik luma ringan dari `/camera/color/image_raw` -> `camera_health_state.mean_luma/stddev_luma/mean_gradient`;
- tampilkan `/obstacle_detection/visualization` pada preview GUI, tetap mempertahankan topic YOLOP lama sebagai fallback;
- target `agv_gui` sekarang link ke message package `yolo_obstacle_detection_ros2`.

Build:
```bash
cd /home/otomasi2/ros
rm -rf build/navigation install/navigation
colcon build --packages-select yolo_obstacle_detection_ros2 navigation --symlink-install
source install/setup.bash
ros2 launch navigation gui.launch.py
```

Validasi sebelum menekan Record:
```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /obstacle_detection/obstacles
ros2 topic echo /obstacle_detection/performance --once
ros2 topic info -v /obstacle_detection/performance
```

Saat GUI hidup, grafik Persepsi harus bergerak bahkan sebelum Record. Tombol Record hanya menentukan apakah snapshot TelemetryStore ikut disimpan sebagai raw run; grafik dan recorder membaca sumber key yang sama.
