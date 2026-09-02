# Forclift ROS 2 Workspace

Repository source code untuk sistem autonomous forklift berbasis ROS 2.

Branch utama/default repository ini adalah `v1`.

## Struktur Repository

```text
forclift/
├── src/
│   ├── esc/
│   ├── navigation/
│   └── yolo_obstacle_detection_ros2/
├── .gitignore
└── README.md
```

Repository ini sengaja hanya menyimpan source code. Direktori hasil build ROS 2 seperti `build/`, `install/`, dan `log/` tidak disimpan ke Git.

## Package ROS 2

### `esc`
Package untuk subsistem ESC/winch dan kontrol terkait aktuator.

Launch file utama yang tersedia antara lain:

```bash
ros2 launch esc esc.launch.py
ros2 launch esc winch.launch.py
```

### `navigation`
Package utama navigasi AGV/forklift yang berisi konfigurasi, GUI, sensor, mapping, localization, autonomous navigation, RViz, URDF, dan komponen pendukung lainnya.

Beberapa launch file penting:

```bash
ros2 launch navigation lidar.launch.py
ros2 launch navigation imu.launch.py
ros2 launch navigation map.launch.py
ros2 launch navigation slam_async.launch.py
ros2 launch navigation autonomous.launch.py
ros2 launch navigation gui.launch.py
ros2 launch navigation allsystem.launch.py
```

### `yolo_obstacle_detection_ros2`
Package perception berbasis YOLO untuk deteksi obstacle/objek dan integrasi kamera.

Launch file utama:

```bash
ros2 launch yolo_obstacle_detection_ros2 camera.launch.py
ros2 launch yolo_obstacle_detection_ros2 perception_all.launch.py
```

## Build

Repository ini digunakan sebagai source workspace ROS 2. Contoh penggunaan:

```bash
mkdir -p ~/ros
cd ~/ros
git clone -b v1 https://github.com/rofiqcp/forclift.git src_repo
```

Jika ingin menggunakan struktur workspace langsung dengan isi repository sebagai source, salin atau tautkan package dari `src_repo/src` ke folder `src` workspace ROS 2 Anda.

Untuk workspace yang sudah memiliki repository ini pada root workspace:

```bash
cd /path/to/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Menjalankan Sistem

Setelah build berhasil:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Kemudian jalankan launch file sesuai subsistem yang diperlukan, misalnya:

```bash
ros2 launch navigation autonomous.launch.py
```

atau GUI:

```bash
ros2 launch navigation gui.launch.py
```

## Catatan

- ROS 2 yang digunakan pada pengembangan workspace adalah ROS 2 Humble.
- Pastikan dependency setiap package telah tersedia sebelum melakukan build.
- File hasil `colcon build` tidak di-version-control.
- Folder model di luar `src/` tidak termasuk dalam repository ini.
- Beberapa model atau asset yang berada di dalam package `src/` tetap ikut karena merupakan bagian dari source package tersebut.

## Repository

Owner: `rofiqcp`  
Repository: `forclift`  
Default branch: `v1`
