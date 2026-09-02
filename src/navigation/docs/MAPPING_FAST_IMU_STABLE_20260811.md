# Mapping Fast + RobotModel IMU Stabilized

## Tujuan

Patch ini mengembalikan jalur mapping ke master referensi `src(4).zip` yang sudah terbukti bagus, lalu menurunkan pengaruh IMU pada visualisasi RobotModel.

## Perubahan mapping

- `/scan` kembali memakai `SensorDataQoS` (`BEST_EFFORT`, `VOLATILE`) agar scan terbaru tidak tertahan retransmit/queue reliable.
- `hector_slam_node.cpp`, header, dan `hector.yaml` dikembalikan ke algoritma/parameter master referensi.
- `slam_toolbox.yaml` dikembalikan ke master: `map_update_interval=0.1 s`, `minimum_time_interval=0.05 s`, `minimum_travel_distance=0.01 m`, `minimum_travel_heading=0.0087 rad`.
- Startup SLAM menjadi 5.0 s seperti master.
- TF mapping tetap: LiDAR odometry memiliki `odom -> base_footprint`; SLAM Toolbox memiliki `map -> odom`; EKF tidak publish TF.

## Perubahan IMU / Kalman gain visual

RobotModel RViz pada paket ini menggunakan TF `visual_/base_footprint`. Sebelumnya yaw visual memakai orientasi AHRS IMU mentah 1:1, sehingga menurunkan covariance EKF saja tidak dapat menghilangkan jitter visual.

Sekarang visual yaw memakai scalar Kalman fusion:

- Prediksi yaw: delta yaw `/lidar/odom`.
- Koreksi: absolute yaw `/imu/data`.
- IMU measurement variance `R = 0.20 rad^2`.
- Process variance `Q = 0.0004 rad^2`.
- Kalman gain IMU dibatasi maksimum `K = 0.05`.
- Innovation IMU dibatasi `12 deg` agar spike magnetometer/AHRS tidak menendang RobotModel.

IMU covariance pada `/imu/data` juga dibuat lebih konservatif:

- orientation covariance Z: `0.20`
- angular velocity covariance Z: `0.08`

Artinya EKF dan visualisasi sama-sama tidak terlalu percaya IMU.

## Build

```bash
cd ~/ros
source /opt/ros/humble/setup.bash
rm -rf build/navigation install/navigation
colcon build --symlink-install --packages-select navigation
source install/setup.bash
```

Jika workspace Anda bukan `~/ros`, ganti path sesuai workspace.

## Launch mapping

```bash
ros2 launch navigation map.launch.py
```

## Verifikasi cepat

```bash
ros2 topic hz /scan
ros2 topic hz /lidar/odom
ros2 topic hz /map
ros2 topic echo /odometry/filtered --once
```

Target `/scan` sekitar 10 Hz. `/map` tidak harus berubah isi saat robot benar-benar diam, tetapi ketika robot bergerak known/free/occupied cells harus bertambah tanpa freeze panjang.

Untuk melihat statistik:

```bash
ros2 topic echo /mapping/map_stats
```

Untuk memeriksa TF owner:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo odom visual_/base_footprint
```

## Jika visual masih sedikit jitter

Turunkan `max_imu_kalman_gain` pada `launch/map.launch.py` dari `0.05` ke `0.03`. Jangan langsung dibuat nol karena IMU masih berguna sebagai koreksi heading jangka panjang.
