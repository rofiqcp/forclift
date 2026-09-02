# V38 autonomous sensor, costmap, scan-noise, dan visual-TF fix

## Temuan dari log(6)

- Mapping membuktikan `/imu/data` dan `/scan_nav` mencapai gate dengan data nyata.
- Hampir semua paket T-mini Plus tercatat `checksum_fail`, tetapi tetap diterima karena mode checksum longgar.
- Filter navigasi sempat meneruskan scan mentah sebelum TF statis `base_footprint <- lidar_link` tersedia.
- EKF mapping 30 Hz beberapa kali melewati deadline saat Ceres/RViz mulai bekerja.
- Penghentian sesi mapping lama belum secara eksplisit membersihkan parent `map.launch.py` sebelum autonomous mengambil port serial.

## Perubahan

- Preflight autonomous membersihkan sesi `map.launch.py` lama sebelum membuat alias dan driver serial baru.
- Checksum YDLIDAR menerima format `[CT | LSN]` lengkap sesuai protokol SDK serta varian lama yang hanya memakai bit tipe paket.
- `/scan_nav` ditahan sampai TF statis tersedia; scan mentah tidak lagi masuk ke SLAM/costmap saat startup.
- Filter anti-starburst navigasi diperketat. `/scan_safety` tetap terpisah dan tidak memakai filter tersebut.
- Visual TF mapping dan autonomous memakai hysteresis/deadband lebih besar serta tidak menampilkan yaw IMU mentah secara langsung.
- EKF mapping diturunkan menjadi 20 Hz untuk menghindari deadline miss di Jetson.
- Local dan global inflation menjadi `0.35 m`; `cost_scaling_factor=18.0`.
- Footprint fisik tetap `1.30 x 0.80 m`, MPPI tetap `consider_footprint=true`, Smac Hybrid-A* tetap aktif, dan Collision Monitor tetap memakai `/scan_safety`.
- RViz autonomous dibuka setelah kedua costmap benar-benar menerbitkan grid, sehingga status startup sementara tidak tampil sebagai warning.

## Build dan uji perangkat

```bash
cd /home/otomasi2/ros
bash src/navigation/tools/rebuild_navigation_clean.sh
source install/setup.bash
python3 install/navigation/lib/navigation/verify_v38_autonomous_sensor_costmap_stability.py
bash src/navigation/tools/run_autonomous.sh auto
```

Di terminal kedua, setelah autonomous siap:

```bash
source /home/otomasi2/ros/install/setup.bash
AUTONOMOUS_CHECK_REQUIRE_PERCEPTION=0 \
  python3 /home/otomasi2/ros/install/navigation/lib/navigation/autonomous_runtime_check.py
```

PASS perangkat keras memerlukan `/imu/data >= 20 Hz`, `/scan_nav >= 3 Hz`, kedua costmap, seluruh lifecycle Nav2, dan rantai TF termasuk `map -> visual_/base_footprint`.
