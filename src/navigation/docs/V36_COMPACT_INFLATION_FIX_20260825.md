# V36 Compact Inflation Fix — 2026-08-25 (superseded by V37)

V36 menggunakan radius `0.45 m`. Target operasional terbaru adalah `0.40 m`
dan dijelaskan pada `V37_MINIMUM_SAFE_INFLATION_PATH_FIX_20260825.md`.

## Perubahan aktif

- Local dan global `inflation_radius`: `0.55 m` menjadi `0.45 m`.
- Local dan global `cost_scaling_factor`: `12.0` menjadi `18.0`.
- `footprint_padding`: tetap/dipulihkan ke `0.0`.
- Footprint fisik tetap `1.30 x 0.80 m` (`+/-0.65 x +/-0.40 m`).
- MPPI `CostCritic.consider_footprint`: tetap `true`.
- Polygon `FrontStop`, `FrontSlow`, `RearStop`, dan `RearSlow` Collision
  Monitor tidak diubah.

Pada resolusi costmap `0.05 m`, radius fisik terinskripsi kendaraan adalah
`0.40 m`. Target `0.45 m` mempertahankan seluruh radius fisik dan menyisakan
satu sel soft-cost di luarnya. Faktor `18.0` mempercepat penurunan biaya di sel
tersebut. Ini mengecilkan halo perencanaan tanpa mengecilkan kendaraan yang
dipakai untuk pemeriksaan tabrakan.

## Persistensi

`ensure_stage5_planning_runtime()` selalu memigrasikan local dan global
costmap ke target V36. Dengan demikian, YAML lama pada
`/home/otomasi2/ros/config/runtime/navigation/nav2_ackermann.yaml` tidak dapat
mengembalikan radius `0.55 m`, scaling `12.0`, atau padding non-zero setelah
rebuild.

## Build, validasi, dan run

```bash
cd /home/otomasi2/ros
bash src/navigation/tools/rebuild_navigation_clean.sh /home/otomasi2/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/navigation/tools/verify_v36_compact_inflation.py
ros2 launch navigation autonomous.launch.py map:=auto
```

Sesudah costmap aktif, verifikasi parameter runtime:

```bash
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.cost_scaling_factor
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /global_costmap/global_costmap inflation_layer.cost_scaling_factor
ros2 param get /local_costmap/local_costmap footprint_padding
ros2 param get /global_costmap/global_costmap footprint_padding
```

Target berturut-turut: `0.45`, `18.0`, `0.45`, `18.0`, `0.0`, `0.0`.
