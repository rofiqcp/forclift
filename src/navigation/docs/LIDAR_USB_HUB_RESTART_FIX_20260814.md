# LiDAR USB Hub Restart Fix — 2026-08-14

## Gejala dari log

Pada run pertama LiDAR terverifikasi pada `/dev/ttyUSB1 @ 230400` dan `/scan` kemudian mencapai sekitar 10 Hz.
Pada run berikutnya, setelah USB hub mengubah enumerasi port, auto-detect salah menerima `/dev/ttyUSB0 @ 115200` sebagai LiDAR. Port tersebut adalah jalur CP2102 IMU pada sistem ini. Setelah stream tidak pernah membentuk revolusi valid, recovery berulang dan akhirnya node crash dengan `std::system_error: Resource deadlock avoided`.

## Perbaikan

1. Probe LiDAR tidak lagi mencoba 115200 baud. T-mini Plus sistem ini menggunakan 230400; 128000 dipertahankan sebagai fallback YDLIDAR.
2. Port IMU CP2102, `/dev/imu_yahboom`, dan `/dev/esc` dikecualikan dari probing LiDAR. Perbandingan menggunakan canonical path, jadi perlindungan tetap berlaku ketika USB hub menukar `ttyUSB0/ttyUSB1`.
3. Deteksi LiDAR tidak lagi menerima satu byte-pair `AA55`. Kandidat harus menghasilkan minimal 3 paket triangle YDLIDAR yang lolos validasi struktur LSN/FSA/LSA.
4. Stable alias/by-id dipertahankan jika tersedia. Generic `/dev/ttyUSB*` tidak dibuka ulang secara buta setelah I/O error; driver melakukan full protocol re-detection.
5. Ownership pointer serial dilindungi mutex agar reconnect worker tidak mereset serial saat ScanReader sedang memakai pointer yang sama.
6. Error callback tidak lagi menghentikan/join ScanReader dari worker thread. Callback hanya menaikkan flag; teardown dilakukan oleh ROS monitor timer sehingga self-join `Resource deadlock avoided` hilang.
7. Jika stream restart gagal 3 kali berturut-turut, driver melakukan full teardown + port re-detection.
8. Open port melakukan reset RTS/DTR deterministik untuk membantu kondisi USB-UART yang tertinggal setelah proses sebelumnya ditutup. HUPCL juga dinonaktifkan agar close/open tidak memberi pulse kontrol yang tidak diinginkan.

## Target log setelah fix

Startup yang sehat harus memperlihatkan pola:

```text
[SERIAL-ARBITRATION] Skipping reserved serial port: ...CP2102...
Testing LiDAR candidate /dev/ttyUSBX @ 230400...
[VERIFIED] LiDAR triangle protocol on /dev/ttyUSBX -> /dev/ttyUSBX @ 230400 (... structurally valid packets)
Opening serial port /dev/ttyUSBX @ 230400 baud...
[INFO] LIDAR CONNECTED
[INFO] MOTOR RUNNING ...
LiDAR initialization complete
[ScanReader] ... published=... measured_scan_hz=~10
```

Tidak boleh lagi muncul verifikasi LiDAR pada `115200` atau crash `Resource deadlock avoided`.

## Uji setelah build

```bash
cd ~/Documents/agv_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select navigation --symlink-install
source install/setup.bash

ros2 launch navigation map.launch.py
```

Terminal kedua:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/agv_ws/install/setup.bash
ros2 topic hz /scan
```

Target sesudah stabil: `/scan` mendekati frekuensi fisik LiDAR (sekitar 10 Hz pada konfigurasi sekarang).

Lalu `Ctrl+C`, tunggu proses launch benar-benar selesai, dan jalankan `ros2 launch navigation map.launch.py` lagi tanpa cabut-pasang USB. LiDAR harus kembali berputar dan `/scan` muncul lagi. Ulangi beberapa kali untuk memvalidasi restart melalui USB hub.
