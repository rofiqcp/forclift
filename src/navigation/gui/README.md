# Autonomous Vehicle Interface — Native C++/Qt5

GUI operator package `navigation` sekarang sepenuhnya native **C++17 + Qt5 + rclcpp**. GUI lama `agv_gui.py` sudah tidak menjadi runtime dan tidak lagi membutuhkan `rclpy` atau `PyQt5`.

## Build

```bash
cd ~/Sistem-Otomasi-Car/Car-MiniPC/ros
rm -rf build/navigation install/navigation
colcon build --packages-select navigation --symlink-install --cmake-clean-cache --event-handlers console_direct+
source install/setup.bash
```

Dependency native yang diperlukan antara lain Qt5 Widgets/Network dan yaml-cpp. Pada Ubuntu 22.04 paket sistem utamanya adalah `qtbase5-dev` dan `libyaml-cpp-dev`. Dengan rosdep:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Jalankan seluruh stack + GUI:

```bash
ros2 launch navigation gui.launch.py
```

GUI saja:

```bash
ros2 run navigation agv_gui
```

## Struktur

- `agv_gui.cpp` — entry translation unit, rclcpp bridge, dan Qt meta-object classes.
- `modules/*.cpp` — modul GUI terpisah untuk YAML/core, reporting, acquisition, steering, ESC/map, kamera, dan main window.
- `agv_gui_specs.hpp` — definisi parameter/configuration pages.
- `agv_experiment_catalog.hpp` — satu-satunya katalog subbab pengujian 4.x untuk Navigasi, Persepsi, dan ESC/FOC.
- `assets/` — aset visual.

ROS dijalankan pada `MultiThreadedExecutor` terpisah sehingga callback sensor tidak memblokir event-loop Qt. Data callback diteruskan ke Qt melalui signal/slot queued connection. Subscription disimpan selama umur bridge sehingga tidak terhapus setelah dibuat.

## Navigasi GUI

Tombol `☰` kiri atas membuka **satu floating menu**. Menu hanya mempunyai tiga tab utama: **Navigasi**, **Persepsi**, dan **ESC**. Di setiap tab, subbab 4.1, 4.2, dst. serta leaf 4.x.x dibangun langsung dari `agv_experiment_catalog.hpp`. Tidak ada lagi panel `Tools lama` dan tidak ada tree BAB IV permanen yang memenuhi sidebar.

Klik leaf 4.x.x merutekan ke workspace akuisisi existing yang sama, bukan membuat widget duplikat. Pilihan subsystem/leaf terakhir disimpan ke `gui.yaml` (`navigation_menu.active_subsystem` dan `navigation_menu.active_leaf`) sehingga session berikutnya kembali ke pengujian terakhir.

Parameter panel kiri tetap memakai YAML sebagai source of truth dengan autosave atomik. GUI juga mempertahankan vehicle-authority synchronization, Config-vs-Runtime audit, MPPI A/B/C sweep, stationary IMU/GNSS capture, COG qualification, GNSS fusion controls, rosbag, CPU/GPU profile, snapshot konfigurasi, CSV/PNG export, dan metadata SHA-256.

## Pelaporan

Setiap CSV menghasilkan sidecar `.meta.json` yang berisi session/experiment metadata, snapshot SHA-256 konfigurasi, serta statistik numerik. Kolom bernama `error` atau `residual` juga mendapatkan RMSE, mean absolute error, dan maximum absolute error.

## Catatan runtime

Perubahan YAML tidak otomatis berarti node yang sudah berjalan telah menerima nilai baru. Gunakan **Verify Runtime** dan restart/live-apply node terkait sebelum merekam pengujian. Tutup GUI akan menghentikan launch pada `gui.launch.py`, sehingga autonomous stack tidak tertinggal berjalan tanpa HMI.
