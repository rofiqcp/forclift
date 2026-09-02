# V28 Compact Inflation — 2026-08-21 (superseded by V37)

Nilai operasional terbaru ada pada dokumen V37. Verifier lama ini tetap
dipelihara agar memeriksa target aktif V37 dan tidak meloloskan konfigurasi
runtime lama.

## Tujuan

Mengurangi area inflation pada Map 1, Map 2, dan Map 3 agar AGV mempunyai ruang manuver lebih besar tanpa mengecilkan geometri fisik robot.

## Parameter aktif

File: `config/nav2_ackermann.yaml`

- Footprint tetap: `1.30 x 0.80 m` (`+/-0.65 m x +/-0.40 m`).
- `local_costmap.inflation_layer.inflation_radius`: `0.40 m`.
- `local_costmap.inflation_layer.cost_scaling_factor`: `18.0`.
- `global_costmap.inflation_layer.inflation_radius`: `0.40 m`.
- `global_costmap.inflation_layer.cost_scaling_factor`: `18.0`.
- `footprint_padding`: tetap `0.0`.
- MPPI `CostCritic.consider_footprint`: tetap `true`.

Ketiga map menggunakan costmap Nav2 yang sama saat map tersebut menjadi active navigation map, sehingga parameter inflation ini berlaku konsisten pada Map 1, Map 2, dan Map 3.

## Alasan nilai 0.40 m

Lebar fisik AGV adalah 0.80 m sehingga inscribed radius kira-kira 0.40 m. Radius 0.40 m menghapus soft cost band tambahan, tetapi collision tetap diperiksa memakai full rectangular footprint. Nilai tidak boleh diturunkan lagi tanpa pengukuran fisik baru dan audit seluruh collision checker.

`cost_scaling_factor=18.0` membuat biaya inflation turun lebih cepat daripada konfigurasi sebelumnya, sehingga area merah/pink pada costmap lebih kompak dan koridor sempit lebih mungkin dipakai planner/controller.

## Validasi runtime

```bash
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.cost_scaling_factor
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /global_costmap/global_costmap inflation_layer.cost_scaling_factor
```

Target: radius `0.40`, scaling `18.0` untuk local dan global.
