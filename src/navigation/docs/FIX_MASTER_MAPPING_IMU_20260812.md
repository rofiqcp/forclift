# Fix master mapping + IMU RobotModel — 2026-08-12

## Masalah runtime yang ditemukan

Log `20260812_164838.txt` menunjukkan RobotModel terbaru bukan memakai yaw IMU. Runtime secara eksplisit menulis bahwa IMU hanya dipakai sebagai motion validation, sedangkan yaw RobotModel berasal dari LiDAR. Pada saat yang sama pose Hector berubah dari sekitar `(0.046, -0.027, -2.2 deg)` menjadi `(0.341, -0.678, 48.2 deg)` lalu `(0.532, -1.455, 78.5 deg)`, sehingga scan matching mengalami false-motion/drift besar dan map menjadi bertumpuk.

## Perbaikan

1. `hector_slam_node.cpp/.hpp` dikembalikan ke implementasi master yang memiliki stationary scan detection, deadband, EMA, quality gate, dan rejection logic yang sudah tervalidasi pada master.
2. `hector.yaml` dikembalikan persis ke parameter master (grid internal 0.025 m, filter window 5, min scan-match quality 0.30, physical limits dan anti-drift master).
3. `slam_toolbox.yaml` dikembalikan ke parameter mapping master, termasuk `map_update_interval: 0.1`, `minimum_travel_distance: 0.01`, dan matcher geometry master.
4. Jalur RobotModel visual dikembalikan ke konsep master: X/Y dari `/lidar/odom`, tetapi yaw langsung dari `/imu/data`. Offset heading awal dihitung satu kali agar heading IMU dan heading awal LiDAR sejajar.
5. TF visual dikembalikan ke prefix master `visual_/`; RViz RobotModel menggunakan `TF Prefix: visual_`.
6. Jalur visual EKF lama (`imu_ekf_preprocessor_node`, `visual_lidar_yaw_guard_node`, `visual_ekf_filter_node`) dihapus dari mapping karena jalur itu justru membuat yaw RobotModel tetap dimiliki LiDAR.

## Arsitektur setelah fix

```text
MAPPING:
/scan -> hector_slam_node -> /lidar/odom + odom->base_footprint
/scan + odom->base_footprint -> slam_toolbox -> /map + map->odom

ROBOTMODEL VISUAL:
/lidar/odom X,Y ----\
                     -> imu_visual_tf_node -> odom->visual_/base_footprint
/imu/data yaw ------/

visual_/base_footprint -> visual_/base_link -> seluruh URDF/CAD
```

Catatan: IMU tidak dipakai untuk menghasilkan X/Y melalui double integration karena itu akan menghasilkan drift posisi jauh lebih besar. Yang dibuat benar-benar mengikuti IMU adalah orientasi/yaw RobotModel, sedangkan translasi tetap mengambil odometri LiDAR seperti master.
