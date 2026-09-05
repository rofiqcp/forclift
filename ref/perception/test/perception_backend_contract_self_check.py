#!/usr/bin/env python3
"""Static contract for OFF(camera-only)/CPU(.pt direct)/GPU(TensorRT)."""
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT.parent / "navigation"

def require(condition: bool, message: str) -> None:
    if not condition:
        print(f"FAIL: {message}")
        raise SystemExit(1)

cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
cpu = (ROOT / "src/astra_yolop_cpu_pt_node.cpp").read_text(encoding="utf-8")
camera = (ROOT / "src/camera_only_node.cpp").read_text(encoding="utf-8")
gpu = (ROOT / "src/astra_yolop_gpu_node.cpp").read_text(encoding="utf-8")
direct_launch = (ROOT / "launch/astra_yolop.launch.py").read_text(encoding="utf-8")
autonomous = (NAV / "launch/autonomous.launch.py").read_text(encoding="utf-8")
gui = (NAV / "launch/gui.launch.py").read_text(encoding="utf-8")
params = yaml.safe_load((ROOT / "config/astra_yolop_gpu.yaml").read_text())["perception"]["ros__parameters"]

require("project(perception LANGUAGES CXX)" in cmake, "perception project must stay C++")
require("find_package(Torch QUIET)" in cmake and "PERCEPTION_CPU_AVAILABLE" in cmake,
        "CPU direct PT must use optional LibTorch capability detection")
require("torch.utils.cmake_prefix_path" in cmake, "CMake must discover LibTorch from system Python torch")
require("add_executable(perception_cpu_node src/astra_yolop_cpu_pt_node.cpp)" in cmake,
        "direct PT CPU target missing")
require("camera-only tetap dibangun" in cmake, "Torch-missing camera-only fallback missing")
require("add_executable(camera_only_node src/camera_only_node.cpp)" in cmake,
        "camera-only target missing")
require("RENAME perception_cpu_node" not in cmake, "legacy Python converter must not own CPU executable")
require("check_language(CUDA)" in cmake and "PERCEPTION_GPU_AVAILABLE" in cmake,
        "GPU target must remain optional/capability-gated")
require('_PERCEPTION_GPU_DEFAULT ON' in cmake and '_PERCEPTION_GPU_DEFAULT OFF' in cmake and
        'option(PERCEPTION_BUILD_GPU "Build the CUDA/TensorRT backend when dependencies exist" ${_PERCEPTION_GPU_DEFAULT})' in cmake,
        "GPU default must be platform-aware: Jetson ON, x86 Mini-PC OFF")
require('set(CMAKE_MESSAGE_LOG_LEVEL ERROR)' in cmake,
        "TorchConfig optional-warning isolation missing")
require('COMPONENTS core imgproc imgcodecs highgui videoio)' in cmake and 'videoio dnn)' not in cmake,
        "unused OpenCV DNN build dependency must stay removed")

for token in (
    "pt_model_path", "torch::jit::load", "torch::InferenceMode", "forwardTorch",
    "prediction_heads", "anchor_heads", "drivable", "lane",
    "/perception/object_points", "/perception/drivable_boundary_points",
    "/perception/camera_healthy", "/perception/emergency_stop",
    "mixRecenterCommand", "MultiThreadedExecutor", "inference_group_", "control_group_",
):
    require(token in cpu, f"CPU direct PT backend token missing: {token}")
require("readNetFromONNX" not in cpu and "onnx_model_path" not in cpu,
        "CPU backend still contains ONNX inference path")
require('declare_parameter<std::string>("pt_model_path", "/home/sirobo/ros/models/yolopv2.pt")' in cpu, "CPU workspace model default missing")
require("resolveCpuModelPath" in cpu and "YOLOPV2_PT_PATH" in cpu, "CPU portable model discovery missing")
require("resolveCpuThreadCount" in cpu and "torch::set_num_interop_threads(1)" in cpu,
        "CPU anti-oversubscription policy missing")

for token in ("camera_only", "model=off", "/camera/yolop/image_annotated",
              "/camera/astra/image_raw", "/perception/camera_connected"):
    require(token in camera, f"camera-only backend missing: {token}")
require("torch" not in camera.lower() and "onnx" not in camera.lower() and "tensorrt" not in camera.lower(),
        "camera-only backend must not depend on model runtimes")

for source, label in ((direct_launch, "direct"), (autonomous, "autonomous"), (gui, "gui")):
    require("perception_mode" in source and "pt_model_path" in source,
            f"{label} launch missing mode/PT path")
require("camera_only_node" in autonomous and "camera_only_enabled" in autonomous,
        "autonomous OFF mode must run camera-only")
require("importlib.util.find_spec" not in autonomous, "autonomous CPU validation must not probe Python ONNX modules")
require("model CPU tidak ditemukan/kosong" in autonomous, "CPU model fail-fast validation missing")

require(params.get("perception_mode") == "cpu", "Mini-PC YAML must default to CPU perception")
require(params.get("pt_model_path") == "/home/sirobo/ros/models/yolopv2.pt", "YAML PT path must point to workspace models/yolopv2.pt")
require(float(params.get("cpu_inference_fps", 0.0)) >= 0.5, "CPU inference FPS missing")
require(int(params.get("cpu_threads", -1)) == 0, "CPU thread policy must default to auto=0")
for token in ("lane_metrics_topic", "drivable_space_topic", "perception_obstacle_metrics_topic", "near_field_state_topic"):
    require(token in cpu, f"CPU GUI/BAB IV topic contract missing: {token}")
for token in ("pipeline_p95_ms", "capture_dropped_total", "mean_gradient", "road_width_m"):
    require(token in cpu, f"CPU acquisition metric missing: {token}")

for source, label in ((cpu, "CPU"), (gpu, "GPU")):
    require('get_parameter("camera_metric_calibration_validated")' in source,
            f"{label} backend does not enforce metric calibration")

print("PASS perception backend contract")
print("off=camera-only | cpu=TorchScript .pt direct LibTorch | gpu=CUDA/TensorRT")

require('TORCH_CXX11_ABI' in cmake and 'compiled_with_cxx11_abi' in cmake, 'Torch CXX11 ABI guard missing')
