# AGV Forklift ROS 2 Workspace

Repository utama untuk software AGV forklift berbasis ROS 2 Humble. Branch integrasi utama adalah `v1`.

## Source of truth

- `src/navigation/` — LiDAR, IMU, SLAM, localization, Nav2, mapping, web/desktop GUI.
- `src/esc/` — ESC/BTS/winch ROS integration; `winch_serial_node` adalah bridge resmi STM32F411 ke ROS.
- `src/yolo_obstacle_detection_ros2/` — camera, YOLO/perception, obstacle dan pallet alignment.
- `F4gateway/` — Git submodule firmware STM32F411CEU6, branch produksi `v2`.
- `hoverboard-vesc/` — Git submodule firmware hoverboard/VESC, branch `v1`.
- `config/runtime/` — konfigurasi runtime yang dipakai launch/GUI.
- `maps/` dan `src/navigation/maps/` — map dan metadata navigasi.
- `models/` — model/dataset lokal; artefak besar tertentu tidak di-track Git.
- `scripts/` — environment dan QA workspace.
- `install.sh` — setup dependency/build workspace.

Project standalone lama `f4/` sudah dihapus setelah fitur aktifnya diaudit dan dipindahkan/diverifikasi pada submodule `F4gateway` v2. Jangan membuat copy firmware F411 baru di root; perubahan firmware dilakukan di `F4gateway/`.

## Clone dan submodule

```bash
git clone -b v1 --recurse-submodules https://github.com/rofiqcp/forclift.git
cd forclift
git submodule update --init --recursive
```

Submodule yang dipin:

- `F4gateway`: branch `v2`.
- `hoverboard-vesc`: branch `v1`.

Commit submodule pada repository induk adalah pointer reproducible; setelah update submodule, commit pointer tersebut di root workspace.

## Environment

Target utama Jetson/Ubuntu memakai ROS 2 Humble dan system Python:

```bash
source /opt/ros/humble/setup.bash
source scripts/agv_env.sh
```

`AGV_PYTHON` default adalah `/usr/bin/python3`. Hindari mengganti `/usr/bin/python3` dengan virtualenv atau `update-alternatives`, karena ROS 2 Humble dan package system bergantung pada Python Ubuntu.

## Setup dan build

Setup otomatis:

```bash
./install.sh
```

Build manual ROS workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Direktori `build/`, `install/`, dan `log/` adalah output lokal colcon dan tidak di-track Git.

## Firmware STM32F411

Masuk ke submodule dan ikuti dokumentasi firmware:

```bash
cd F4gateway
pio run -e blackpill_f411ce_v2
pio run -e blackpill_f411ce_v2 -t upload
```

Upload normal memakai resident USB CDC bootloader. ST-Link hanya dibutuhkan untuk provisioning awal/recovery. Detail layout flash, udev, limit switch, BTS7960, HMI, test, dan recovery ada di `F4gateway/README.md`.

## Runtime configuration

Launch/runtime membaca konfigurasi dari `config/runtime/`. Source defaults tetap berada di package masing-masing. Backup editor/web lama tidak disimpan di Git; gunakan commit history untuk rollback.

Map navigasi yang digunakan harus memiliki pasangan `.pgm` + `.yaml` yang konsisten. Metadata `*_structure.json` menyimpan hasil evaluasi geometri map dan ikut version control jika menjadi bagian hasil pengujian.

## QA dasar

Periksa source aktif agar tidak kembali memakai path HOME absolut:

```bash
/usr/bin/python3 scripts/check_portable_paths.py
```

Periksa Python helper baru:

```bash
/usr/bin/python3 -m py_compile \
  scripts/check_portable_paths.py \
  src/navigation/scripts/nav2_param_reload_apply.py \
  src/navigation/tools/calibrate_lidar_imu_8dir.py
```

Firmware F411 memiliki self-check tersendiri di `F4gateway/test/`.

## Workflow Git

Urutan sinkronisasi yang aman:

1. Commit/push perubahan di submodule masing-masing terlebih dahulu.
2. Update pointer submodule di root `forclift`.
3. Pastikan `git diff --check` dan QA relevan lulus.
4. `git pull --rebase origin v1` pada root.
5. Push branch `v1`.

Jangan commit output build, Python cache, editor state, backup sementara, TensorRT engine machine-specific, atau archive lokal hasil development.
