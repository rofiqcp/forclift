#!/usr/bin/env python3
"""Read-only build/model/camera preflight for OFF/CPU/GPU perception."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
from pathlib import Path

import yaml


def workspace_from(value: str | None) -> Path:
    candidates = [Path(value).expanduser().resolve()] if value else []
    candidates.extend((Path.cwd().resolve(), *Path.cwd().resolve().parents))
    here = Path(__file__).resolve()
    candidates.extend((here, *here.parents))
    for candidate in candidates:
        if (candidate / "src/perception/CMakeLists.txt").is_file():
            return candidate
    raise FileNotFoundError("workspace ROS tidak ditemukan; gunakan --workspace")


def command(command_line: list[str], timeout: float = 8.0) -> str:
    try:
        result = subprocess.run(
            command_line, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, timeout=timeout, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return ""
    return result.stdout.strip()


def check_dynamic_links(executable: Path) -> list[str]:
    if not executable.is_file():
        return []
    output = command(["ldd", str(executable)], timeout=8.0)
    if not output:
        return [f"ldd gagal untuk {executable}"]
    return [line.strip() for line in output.splitlines() if "not found" in line]




def discover_cpu_model(workspace: Path, configured: str, override: str | None) -> Path:
    if override:
        return Path(override).expanduser().resolve()
    env = __import__("os").environ.get("YOLOPV2_PT_PATH", "").strip()
    if env:
        return Path(env).expanduser().resolve()
    configured = str(configured or "auto").strip()
    if configured.lower() != "auto":
        path = Path(configured).expanduser()
        return path.resolve() if path.is_absolute() else (workspace / path).resolve()
    candidates = [
        workspace / "models/yolopv2.pt",
        Path.home() / "ros/models/yolopv2.pt",
        Path("/home/sirobo/ros/models/yolopv2.pt"),
    ]
    for candidate in candidates:
        if candidate.is_file() and candidate.stat().st_size > 0:
            return candidate.resolve()
    return candidates[0].resolve()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace")
    parser.add_argument("--mode", required=True, choices=("off", "cpu", "gpu"))
    parser.add_argument("--model", help="override yolopv2.pt/engine path")
    parser.add_argument("--require-camera", action="store_true")
    parser.add_argument("--runtime", action="store_true")
    args = parser.parse_args()

    workspace = workspace_from(args.workspace)
    package = workspace / "src/perception"
    config = yaml.safe_load(
        (package / "config/astra_yolop_gpu.yaml").read_text(encoding="utf-8"))
    params = config["perception"]["ros__parameters"]
    executable_name = (
        "camera_only_node" if args.mode == "off" else
        ("perception_cpu_node" if args.mode == "cpu" else "perception_node"))
    configured_model = "" if args.mode == "off" else params[
        "pt_model_path" if args.mode == "cpu" else "engine_path"]
    model = None
    if args.mode == "cpu":
        model = discover_cpu_model(workspace, configured_model, args.model)
    elif args.mode == "gpu":
        model = Path(args.model or configured_model).expanduser()
        if not model.is_absolute():
            model = (workspace / model).resolve()

    cmake = (package / "CMakeLists.txt").read_text(encoding="utf-8")
    errors: list[str] = []
    waits: list[str] = []
    if "add_executable(perception_cpu_node src/astra_yolop_cpu_pt_node.cpp)" not in cmake:
        errors.append("core inference CPU direct .pt hilang dari CMake")
    if "add_executable(camera_only_node src/camera_only_node.cpp)" not in cmake:
        errors.append("camera-only backend hilang dari CMake")
    if "find_package(Torch QUIET)" not in cmake or "PERCEPTION_CPU_AVAILABLE" not in cmake:
        errors.append("LibTorch capability detection CPU belum dikonfigurasi")
    if "check_language(CUDA)" not in cmake:
        errors.append("target GPU tidak capability-gated")
    if args.mode == "cpu":
        torch_probe = command(["/usr/bin/python3", "-c",
            "import torch; f=getattr(torch,'compiled_with_cxx11_abi',None); "
            "assert callable(f) and f(), 'CXX11_ABI harus 1'; "
            "print('PASS', torch.__version__); print(torch.utils.cmake_prefix_path)"])
        if not torch_probe.startswith("PASS "):
            errors.append(
                "PyTorch/LibTorch CPU CXX11_ABI=1 belum tersedia pada /usr/bin/python3")
        elif model is not None and model.is_file():
            model_probe = command(["/usr/bin/python3", "-c",
                "import sys,torch; m=torch.jit.load(sys.argv[1], map_location='cpu'); "
                "m.eval(); x=torch.zeros(1,3,384,640); "
                "o=m(x); assert isinstance(o,(tuple,list)) and len(o)==3; "
                "d,s,l=o; assert isinstance(d,(tuple,list)) and len(d)==2; "
                "assert len(d[0])==3 and len(d[1])==3; print('PASS')",
                str(model)], timeout=30.0)
            if not model_probe.strip().endswith("PASS"):
                errors.append("model .pt gagal kontrak TorchScript YOLOPv2 8-output")
    if model is not None and (not model.is_file() or model.stat().st_size <= 0):
        waits.append(f"model belum tersedia: {model}")

    installed = workspace / "install/perception/lib/perception" / executable_name
    if not installed.is_file():
        waits.append(f"executable belum ter-build: {installed}")
    else:
        unresolved = check_dynamic_links(installed)
        for item in unresolved:
            errors.append(f"shared-library unresolved: {item}")

    camera_paths = sorted(Path("/dev/v4l/by-id").glob("*")) if Path("/dev/v4l/by-id").is_dir() else []
    numeric_video = sorted(Path("/dev").glob("video*"))
    camera_candidate = camera_paths[0] if camera_paths else (numeric_video[0] if numeric_video else None)
    if camera_candidate is None:
        waits.append("kamera V4L2 belum terdeteksi")
    else:
        try:
            camera_target = camera_candidate.resolve(strict=True)
        except OSError:
            camera_target = camera_candidate
        if not os.access(camera_target, os.R_OK | os.W_OK):
            waits.append(f"permission kamera belum read/write untuk user aktif: {camera_target}")

    if args.mode == "gpu":
        gpu_runtime = (Path("/dev/nvidia0").exists() or Path("/proc/driver/nvidia/version").exists()
                       or Path("/etc/nv_tegra_release").exists())
        if not gpu_runtime:
            waits.append("runtime/device NVIDIA/Jetson belum terdeteksi")
        nvcc = shutil.which("nvcc") or ("/usr/local/cuda/bin/nvcc" if Path("/usr/local/cuda/bin/nvcc").is_file() else "")
        if not nvcc:
            waits.append("CUDA nvcc belum terdeteksi")
        if not command(["bash", "-lc", "ldconfig -p 2>/dev/null | grep -m1 libnvinfer.so"]):
            waits.append("TensorRT libnvinfer belum terdeteksi")

    print("PERCEPTION BACKEND PREFLIGHT")
    print(f"workspace : {workspace}")
    print(f"mode      : {args.mode.upper()}")
    print(f"executable: {installed}")
    print(f"model     : {model if model is not None else '(tidak dimuat)'}")
    print(f"camera    : {camera_paths[0] if camera_paths else (numeric_video[0] if numeric_video else 'not found')}")

    if args.runtime:
        if not shutil.which("ros2"):
            errors.append("ros2 CLI tidak ditemukan untuk --runtime")
        else:
            nodes = set(command(["ros2", "node", "list"]).splitlines())
            topics = set(command(["ros2", "topic", "list"]).splitlines())
            expected_node = "/perception_camera" if args.mode == "off" else "/perception"
            if expected_node not in nodes:
                errors.append(f"node {expected_node} tidak aktif")
            base_required = {"/perception/camera_connected", "/perception/camera_healthy",
                             "/camera/yolop/image_annotated"}
            for missing in sorted(base_required - topics):
                errors.append(f"topic kamera runtime hilang: {missing}")
            if args.mode != "off":
                required = {
                    "/perception/object_points", "/perception/drivable_boundary_points",
                    "/perception/performance", "/perception/obstacle_metrics",
                    "/perception/drivable_space", "/perception/near_field_state",
                    "/yolop/lane_metrics",
                }
                for missing in sorted(required - topics):
                    errors.append(f"topic inference runtime hilang: {missing}")

    if errors:
        print("\nERROR")
        for item in errors:
            print(f"  - {item}")
        return 2
    if waits:
        print("\nWAIT")
        for item in waits:
            print(f"  - {item}")
        if args.require_camera or args.runtime:
            return 3
    else:
        print("\nPASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
