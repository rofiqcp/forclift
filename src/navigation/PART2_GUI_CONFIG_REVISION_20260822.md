# PART 2 — GUI, Runtime YAML, Tuning, dan Experiment Reproducibility

Tanggal: 2026-08-22
Basis: `src_PART1_revised_20260822.zip`

## Tujuan
PART 2 tidak mengubah algoritma core odometry/TF/Nav2 PART 1. Fokusnya adalah memastikan GUI menjadi alat engineering yang aman untuk kalibrasi/tuning, YAML yang diedit benar-benar sama dengan YAML yang dipakai launch, nilai aktif ROS dapat dibaca balik, dan setiap eksperimen menyimpan konfigurasi yang digunakan.

## Perubahan utama

1. **Canonical runtime YAML**
   - Default package config sekarang hanya template/seed.
   - Runtime persistent berada di `$AGV_RUNTIME_CONFIG_ROOT/<package>`.
   - Default root: `$AGV_WS/config/runtime`.
   - GUI, autonomous launch, mapping launch, LiDAR, IMU, SLAM, ESC, winch, camera dan perception memakai runtime config yang sama.
   - Runtime file di-seed sekali dari package config jika belum ada; rebuild tidak menimpa tuning.

2. **YAML editor staged, bukan autosave**
   - Autosave 250 ms dihapus.
   - Alur: Edit -> DIRTY -> Validate -> Diff -> Save YAML -> disk readback.
   - Save atomic (`temp + fsync + os.replace`) dan membuat backup.
   - External modification conflict diblok agar tab/editor lain tidak menimpa file yang lebih baru.
   - Rollback memakai backup terbaru.
   - Disk readback benar-benar membuka dan membandingkan file yang baru ditulis.

3. **Scoped editor**
   - AMCL hanya subtree AMCL.
   - Smac hanya `GridBased`.
   - MPPI hanya `FollowPath`.
   - Local dan Global Costmap terpisah.
   - Velocity Smoother memiliki editor sendiri.
   - Mapping/autonomous EKF profile dipisah.
   - Mapping/autonomous LiDAR odometry profile dipisah.
   - SLAM Toolbox mendapatkan halaman tuning sendiri.
   - Vehicle geometry tersedia sebagai source-of-truth terpisah.

4. **Validation**
   - Ditambahkan `config_manager.py` untuk range/type checks dan cross-file checks.
   - Smac/MPPI turning radius diverifikasi konsisten dengan vehicle geometry.
   - MPPI `model_dt` divalidasi terhadap periode controller.
   - Range LiDAR, rate EKF, SLAM thresholds, collision timeout, dan parameter penting lain memiliki guard.

5. **Saved YAML vs Active ROS**
   - Tombol `Read Active ROS` menggunakan parameter service ROS 2.
   - Mismatch YAML vs runtime ditampilkan di tabel.
   - `Save + Apply Live` hanya mengirim parameter yang berubah.
   - Apply menggunakan `set_parameters_atomically`; satu parameter ditolak => batch live tidak diterapkan parsial.
   - Setelah apply diterima, GUI membaca balik nilai aktif.
   - Parameter yang tidak mendukung dynamic update tetap tersimpan di YAML untuk restart node/lifecycle.

6. **Configuration Profiles**
   - Halaman `Configuration` dapat validasi semua runtime YAML.
   - Current runtime configuration dapat disimpan sebagai named tuning profile.
   - Restore profile membackup runtime config sebelum overwrite.
   - Source/package defaults tidak diubah.

7. **Perception config canonicalization**
   - `alignment_realtime.yaml` aktif disatukan ke runtime config yang sama dengan GUI.
   - Autonomous/perception launch tidak lagi memakai alignment YAML berbeda dari yang ditampilkan GUI.

8. **Experiment reproducibility**
   - `Start Recording` sekarang benar-benar mengirim daftar config files ke `ExperimentManager`.
   - Snapshot navigation/ESC/perception disimpan per eksperimen.
   - Setiap snapshot diberi SHA256 di `metadata.yaml`.
   - Telemetry menyimpan `ros_timestamp`, `wall_timestamp`, dan `monotonic_timestamp`.

9. **Workspace portability**
   - GUI export/log mengikuti `$AGV_WS`.
   - Launcher mengekspor `$AGV_WS` dan `$AGV_RUNTIME_CONFIG_ROOT`.

## Runtime workflow yang disarankan

```bash
export AGV_WS=/home/otomasi2/ros
export AGV_RUNTIME_CONFIG_ROOT=$AGV_WS/config/runtime
cd $AGV_WS
colcon build --symlink-install --packages-select navigation esc yolo_obstacle_detection_ros2
source install/setup.bash
python3 src/navigation/tools/verify_part2_gui_config.py
ros2 launch navigation gui.launch.py
```

Pada GUI:
1. Configuration -> Validate All Runtime YAML.
2. Buka subsystem tuning.
3. Reload -> edit -> Validate -> Diff -> Save YAML.
4. Jika node aktif dan parameter memang dynamic, gunakan Save + Apply Live.
5. Tekan Read Active ROS untuk memastikan nilai runtime sama.
6. Sebelum pengujian, Experiments -> Start Recording supaya config snapshot tersimpan.

## Batas validasi di environment ini
Environment audit tidak menyediakan ROS 2 Humble, PyQt5, Jetson GPU, serial LiDAR/IMU/ESC, atau Nav2 lifecycle runtime. Karena itu PART 2 telah diverifikasi melalui parsing/syntax, file-level transaction tests, config validation, runtime-path structural checks, dan regression lint. Build/runtime hardware tetap harus dilakukan pada Jetson.
