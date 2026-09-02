# Akuisisi Data GUI untuk BAB IV

GUI menyediakan tiga workspace akuisisi yang dipilih melalui satu floating menu `☰` kiri atas. Menu mempunyai tab **Navigasi**, **Persepsi**, dan **ESC**, masing-masing dengan group 4.1, 4.2, dst.:

- **Akuisisi BAB IV — Navigasi**: 54 tabel aktual yang mengikuti Tabel 4.2–4.55 pada dokumen navigasi, 17 format grafik, dan tiga bukti screenshot map untuk rangkaian pengujian LiDAR, yaw IMU, odometri `vx`, tiga hasil map SLAM, AMCL, global costmap, Smac Hybrid-A*, serta validasi end-to-end.
- **Akuisisi BAB IV — Persepsi**: 32 bentuk tabel dan 10 format grafik dari laporan persepsi visual.
- **Akuisisi BAB IV — ESC / FOC**: 15 bentuk tabel dan 20 format grafik dari laporan steering/FOC.

Setiap subpengujian mempunyai kolom tabel sesuai laporan, grafik live atau ringkasan, identitas variasi/kondisi, ground truth opsional, rekaman sampel mentah, dan tombol **Simpan CSV + PNG + Raw**. Hasil disimpan secara default ke `~/.ros/agv_gui_reports` bersama manifest JSON.

## Arti warna sel

- Hijau: nilai dihitung atau diambil otomatis dari topic/source yang tersedia.
- Kuning: nilai perlu diisi dari ground truth, alat ukur eksternal, atau hasil observasi operator.

Nilai berlabel "DATA ESTIMASI" di laporan hanya dipakai untuk menentukan bentuk tabel/grafik. Nilai estimasi tersebut tidak disalin sebagai hasil pengujian aktual.

Field tuning pada subbab AMCL, global costmap, Smac, serta scan frequency menampilkan nilai aktual dari YAML source. Perubahan yang dikonfirmasi pada field tersebut ditulis atomik ke `config/nav2_ackermann.yaml` atau `config/lidar.yaml`; parameter lain pada source tidak disentuh.

## Topic penting

Navigasi memakai `/scan`, `/imu/data`, `/odom`, `/odometry/filtered`, `/map`, `/amcl_pose`, `/particle_cloud`, `/plan`, `/local_plan`, costmap, command chain, dan status goal yang sudah ada. Persepsi memakai detection, obstacle metrics, lane/drivable-space, camera health, near-field, safety, dan performance topic CPU/GPU yang sudah ada.

Steering target/actual dan RPM tersedia dari topic ESC yang sudah ada. Kolom arus/tegangan FOC (`Id`, `Iq`, `Vd`, `Vq`, dan `Vbus`) akan ditandai belum tersedia sampai sistem memublikasikan `std_msgs/msg/String` pada `/esc/foc/telemetry`. GUI sengaja tidak mengarang nilai pengganti.

## Build dan run

```bash
cd /home/otomasi/mobil_stage3_ws
colcon build --packages-select navigation --symlink-install
source install/setup.bash
export QT_QPA_PLATFORM=xcb
ros2 launch navigation gui.launch.py
```

Perbaikan ini tidak mengubah `autonomous.launch.py`, konfigurasi Nav2, maupun konfigurasi RViz. Jalankan autonomous seperti biasa; paket ini hanya menimpa file GUI dan self-check terkait GUI.
