# Perception YOLOPv2 — OFF / CPU / GPU

Runtime memakai satu selector: `perception_mode:=off|cpu|gpu`. Untuk profil Mini-PC, konfigurasi default sekarang `cpu`.

| Mode | Kamera | Model runtime | Backend |
|---|---|---|---|
| `off` | aktif | tidak ada | camera-only C++ |
| `cpu` | aktif | `yolopv2.pt` auto-discovery | direct TorchScript LibTorch CPU |
| `gpu` | aktif | TensorRT `.engine` | CUDA + TensorRT |

CPU **tidak menggunakan ONNX** saat runtime. Model CPU dicari dari `YOLOPV2_PT_PATH`, `<workspace>/models/yolopv2.pt`, `~/ros/models/yolopv2.pt`, `~/models/yolopv2.pt`, lalu jalur Jetson lama sebagai fallback kompatibilitas.

## CPU Mini-PC

Pastikan PyTorch CPU/LibTorch tersedia pada `/usr/bin/python3`, lalu taruh model di `<workspace>/models/yolopv2.pt`.

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select perception navigation --symlink-install
source install/setup.bash
python3 src/perception/tools/perception_backend_preflight.py --workspace . --mode cpu
ros2 launch navigation gui.launch.py perception_mode:=cpu
```

Default stabilitas Mini-PC: `cpu_inference_fps=2.0`, `cpu_threads=0` (auto; menyisakan CPU untuk ROS2/Nav2/GUI). Naikkan FPS hanya setelah mengamati `/perception/performance`.

Model wajib TorchScript. Output yang diterima mengikuti kontrak YOLOPv2: `([pred, anchor_grid], drivable, lane)` dengan 3 prediction head dan 3 anchor head.

## GPU Jetson Orin

```bash
ros2 launch navigation gui.launch.py perception_mode:=gpu
```

GPU runtime tetap membutuhkan TensorRT engine lokal target. Engine jangan dipindahkan sembarang antar arsitektur/versi TensorRT.

## OFF / camera-only

```bash
ros2 launch navigation gui.launch.py perception_mode:=off
```

Kamera tetap membuka V4L2 dan publish preview/health, tetapi model tidak diload dan inference tidak dijalankan.

Lihat juga `MINIPC_CPU_YOLOPV2.md` di root workspace untuk checklist build, runtime, dan topik pengambilan data.
