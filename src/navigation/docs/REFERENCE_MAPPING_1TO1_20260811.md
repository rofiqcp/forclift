# Reference 1:1 Mapping — LiDAR map + IMU RobotModel

Implementasi ini mengganti jalur mapping eksperimental sebelumnya dengan arsitektur yang sama dengan source referensi yang diberikan.

## Jalur data

```text
YDLIDAR T-mini Plus
  /scan (~10 Hz)
      |
      +--> hector_slam_node (LiDAR scan matching, odometry-only)
      |       +--> /lidar/odom
      |       +--> odom -> base_footprint     [REAL mapping TF]
      |
      +--> slam_toolbox async
              +--> /map
              +--> map -> odom

WT901 / WitMotion AHRS
  /imu/data (~50 Hz, absolute orientation)
      |
      +--> imu_visual_tf_node
             ^
             | x/y from /lidar/odom
             |
             +--> odom -> visual_/base_footprint
                    yaw = IMU yaw relative to startup alignment
                    x/y = LiDAR odometry

visual_ robot_state_publisher
  visual_/base_footprint -> visual_/base_link -> visual_/...
      |
      +--> RViz RobotModel, TF Prefix = visual_
```

## Pemisahan yang wajib

- `hector_slam_node` adalah satu-satunya publisher `odom -> base_footprint` untuk mapping.
- `slam_toolbox` adalah satu-satunya publisher `map -> odom` dan `/map`.
- EKF mapping berjalan dengan `publish_tf=false`, jadi tidak bisa berebut TF.
- `imu_visual_tf_node` hanya menerbitkan `odom -> visual_/base_footprint`; frame `visual_` tidak dipakai SLAM.
- ESC boleh aktif untuk menggerakkan AGV tetapi `/esc/odom` tidak dipakai oleh mapping.

## Parameter yang disamakan dengan referensi

SLAM Toolbox:
- `map_update_interval: 0.1`
- `minimum_time_interval: 0.05`
- `minimum_travel_distance: 0.01`
- `minimum_travel_heading: 0.0087`
- `scan_buffer_size: 10`
- `correlation_search_space_dimension: 0.5`
- `correlation_search_space_resolution: 0.01`
- `correlation_search_space_smear_deviation: 0.1`

LiDAR odometry (`hector.yaml`) adalah file yang disalin langsung dari referensi C1SLAM. Source `hector_slam_node.cpp/.hpp` dan `imu_visual_tf_node.cpp` juga disalin dari referensi.

Frame mounting dikembalikan seperti referensi:
- `base_footprint_joint yaw = 0`
- `lidar_joint yaw = 0`

Driver LiDAR yang sudah terbukti pada log sebelumnya tetap dipakai karena sudah menghasilkan scan fisik sekitar 10 Hz. Transport checksum T-mini tetap soft agar mismatch firmware tidak mematikan scan.

## Build bersih

```bash
cd /home/otomasi2/forclift
rm -rf build/navigation install/navigation
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select navigation esc
source install/setup.bash
```

## Mapping

```bash
ros2 launch navigation map.launch.py
```

## Preflight

```bash
bash /home/otomasi2/forclift/src/navigation/tools/check_mapping.sh
```

## Tes pemisahan IMU/LiDAR

1. Robot diam. Putar chassis/IMU bersama robot: `visual_/base_footprint` yaw harus mengikuti IMU.
2. Jangan menjadikan `visual_` sebagai input SLAM. SLAM selalu memakai `base_footprint` asli yang berasal dari LiDAR odometry.
3. `/map` harus mengikuti geometri `/scan`; IMU tidak mengubah occupancy grid secara langsung.
4. RViz `Fixed Frame = map`, `RobotModel TF Prefix = visual_`.

Perlu diingat: pada reference, posisi X/Y RobotModel visual memang berasal dari LiDAR odometry; IMU hanya menjadi sumber orientation/yaw visual. Mengintegrasikan accelerometer IMU menjadi X/Y tidak dilakukan karena drift-nya besar.
