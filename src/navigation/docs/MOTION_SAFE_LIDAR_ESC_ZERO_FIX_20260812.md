# Motion-safe LiDAR + ESC offline-zero fix — 2026-08-12

## Gejala runtime

- Ketika IMU saja digerakkan sistem stabil.
- Ketika IMU dan LiDAR digerakkan bersamaan, cleanup LiDAR dapat menyisakan hanya 30–50 bin dari sekitar 140–160 bin raw.
- Scan kemudian gagal batas minimum, `/scan` berhenti dipublish, watchdog mengira LiDAR macet dan me-restart stream berulang.
- RViz kemudian mengalami antrean/filter TF dan mapping berhenti menerima scan baru.

## Perbaikan

1. Cleanup LiDAR dibuat motion-safe: sebuah beam hanya dibuang jika ada tetangga valid di kiri dan kanan yang saling konsisten, sementara beam pusat menyimpang dari keduanya.
2. Temporal scan reference selalu maju pada setiap revolusi fisik, bahkan bila cleanup tidak memenuhi soft threshold.
3. `minimum_valid_bins` menjadi soft threshold. Jika raw scan masih sehat, driver fallback ke raw scan daripada memutus `/scan`.
4. Watchdog memakai timestamp revolusi fisik (`last_completed_scan_wall_time_`), bukan hanya waktu scan yang berhasil dipublish. Cleanup tidak dapat lagi memicu restart motor palsu.
5. `restamp_tf: true` menjaga publikasi TF `map->odom` tetap current untuk visualisasi saat satu scan sempat dilewati.
6. ESC `offline_zero_output: true`: `/esc/odom`, speed/actual, target, joint state, battery, dan temperature numerik dipublish 0 selama hardware belum valid. `feedback_valid`, `ready`, dan connected tetap false.

## Target tes

- `/scan` tetap sekitar 10 Hz ketika IMU dan LiDAR digerakkan bersama.
- Tidak ada restart LiDAR hanya karena cleanup menipis.
- `/esc/odom` selalu tersedia dengan pose/twist nol sebelum ESC tersambung.
- SLAM Toolbox tetap menjadi pembangun `/map`.
