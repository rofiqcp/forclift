# Mapping Responsive Sync Fix — 2026-08-12

Perubahan utama:

- `slam_toolbox` diubah dari `async_slam_toolbox_node` menjadi `sync_slam_toolbox_node` agar scan valid tidak dibuang hanya karena callback sedang sibuk.
- `scan_queue_size=5`, `throttle_scans=1`, `minimum_time_interval=0.0`.
- Node pose-graph dibuat lebih rapat: 5 mm / 0.25 derajat.
- `/map` dirasterisasi 5 Hz (`map_update_interval=0.2`) supaya CPU tidak habis untuk OccupancyGrid/RViz; scan tetap diproses 10 Hz.
- Interactive scan cache dimatikan untuk mengurangi overhead.
- Gate revolusi LiDAR dibuat toleran terhadap kehilangan sektor kecil: 180 point, 300 derajat, 60 valid bin. Natural wrap tetap wajib.
- Timestamp LaserScan memakai midpoint revolusi untuk mengurangi latency pose sekitar setengah periode scan.
- `hector_slam_node` menerbitkan `odom->base_footprint` dan `/lidar/odom` tepat pada timestamp scan, bukan menunggu timer 30 Hz. Ini mengurangi TF message-filter waiting/drop pada SLAM Toolbox.
- Internal local-odometry map diperbarui lebih rapat (1.5 cm / 0.5 derajat) dan median filter diperkecil menjadi window 3.
- Warning `-Wmisleading-indentation` pada scan score diperbaiki.

Target runtime yang diharapkan pada LiDAR 10 Hz:

- `/scan`: mendekati 10 Hz stabil.
- `/map`: sekitar 5 Hz.
- SLAM tidak mengejar backlog lama; queue hanya menahan spike singkat CPU.
- Gerakan lambat lebih cepat menjadi node pose-graph/map.
