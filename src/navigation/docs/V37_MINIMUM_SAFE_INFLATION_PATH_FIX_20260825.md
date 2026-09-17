# V37 Minimum-Safe Inflation Path Fix — 2026-08-25

## Target aktif

- Local dan global `inflation_radius`: `0.45 m` menjadi `0.40 m`.
- Local dan global `cost_scaling_factor`: tetap `18.0`.
- Resolusi costmap: `0.05 m`.
- Footprint fisik: tetap `1.30 x 0.80 m`.
- `footprint_padding`: tetap `0.0`.

Target `0.40 m` sama dengan setengah lebar fisik kendaraan dan menghapus satu
sel soft-cost yang masih tersisa pada V36. Ini adalah batas minimum produksi
yang diizinkan konfigurasi ini. Menurunkannya lagi akan membuat inflation lebih
kecil daripada inscribed radius kendaraan dan tidak boleh dilakukan hanya
untuk memaksa planner menghasilkan path.

Smac Hybrid-A* tetap memakai model `REEDS_SHEPP`, turning radius fisik `2.0 m`,
serta full polygon footprint. MPPI `CostCritic.consider_footprint` tetap `true`.
Collision Monitor tetap menggunakan `/scan_safety`; seluruh polygon stop/slow
depan dan belakang tidak diubah.

## Audit clearance map

Audit statis memakai resolusi map `0.05 m`, sel occupied/unknown sebagai
penghalang, dan distance transform pusat robot menghasilkan:

| Map | Sel clearance pada 0.45 m | Sel clearance pada 0.40 m | Perubahan |
| --- | ---: | ---: | ---: |
| Map 1 | 2915 | 3637 | +24.8% |
| Map 2 | 3763 | 4289 | +14.0% |
| Map 3 | 2995 | 3530 | +17.9% |

Ini membuktikan pengurangan radius membuka ruang pencarian tambahan pada semua
map. Angka tersebut bukan jaminan path untuk sembarang start/goal karena Smac
tetap memeriksa polygon footprint, heading, turning radius, dan obstacle live.

## Batas diagnosis dari log(5)

Arsip `log(5)` hanya memuat dua build colcon. Semua `stderr.log` kosong dan
tidak ada log runtime PlannerServer, koordinat start/goal, atau pesan
`no valid path`. Karena itu, V37 memperbaiki sisa satu sel inflation tetapi
tidak mengklaim bahwa map mempunyai koridor yang secara fisik cukup.

Jika path masih gagal pada V37, penyebab berikutnya bukan radius tambahan:

1. start atau goal berada pada sel occupied/unknown;
2. koridor lebih sempit daripada footprint fisik;
3. turning radius `2.0 m` tidak muat pada geometri koridor;
4. obstacle live dari `/scan_nav` menutup koridor.

## Build dan verifikasi

```bash
cd /home/otomasi2/forclift
bash src/navigation/tools/rebuild_navigation_clean.sh /home/otomasi2/forclift
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/navigation/tools/verify_v37_minimum_safe_inflation.py
ros2 launch navigation autonomous.launch.py map:=auto
```

Saat sistem aktif:

```bash
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap footprint
ros2 param get /global_costmap/global_costmap footprint
ros2 run navigation verify_smac_goal_path.py --ros-args -p timeout_sec:=90.0
```

Target radius runtime adalah `0.4` untuk local dan global. Pilih Goal Pose pada
sel putih bebas, bukan pada warna ungu/pink, hitam, atau area unknown.
