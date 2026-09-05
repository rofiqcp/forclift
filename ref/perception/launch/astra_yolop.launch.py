#!/usr/bin/env python3
"""Selectable OFF/CPU/GPU YOLOPv2 camera launch.

Use autonomous.launch.py or gui.launch.py for the complete robot stack. This
direct launch exists for camera/perception calibration and backend benchmarks.
"""

import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _runtime_path(value: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(value))).resolve()




def _discover_cpu_model(configured: str, share_path: str = "") -> str:
    """Return a portable YOLOPv2 TorchScript path for mini-PC CPU runtime."""
    env = os.environ.get("YOLOPV2_PT_PATH", "").strip()
    if env:
        return str(_runtime_path(env))

    configured = (configured or "auto").strip()
    if configured and configured.lower() != "auto":
        return str(_runtime_path(configured))

    candidates = []
    if share_path:
        share = Path(share_path).resolve()
        parts = list(share.parts)
        if "install" in parts:
            idx = parts.index("install")
            workspace = Path(*parts[:idx]) if idx > 0 else Path("/")
            candidates.append(workspace / "models" / "yolopv2.pt")
    candidates.extend([
        Path.home() / "ros" / "models" / "yolopv2.pt",
        Path("/home/sirobo/ros/models/yolopv2.pt"),
    ])
    for candidate in candidates:
        candidate = candidate.expanduser().resolve()
        if candidate.is_file() and candidate.stat().st_size > 0:
            return str(candidate)
    # Keep a deterministic, portable expected location for the launch error.
    return str(candidates[0].expanduser().resolve()) if candidates else str(
        (Path.home() / "ros" / "models" / "yolopv2.pt").resolve())


def _validate_dynamic_links(executable: Path, label: str) -> None:
    try:
        result = subprocess.run(
            ['ldd', str(executable)], text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, timeout=8.0, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f'{label}: ldd gagal: {exc}') from exc
    output = result.stdout or ''
    missing = [line.strip() for line in output.splitlines() if 'not found' in line]
    if result.returncode != 0 or missing:
        detail = '; '.join(missing) if missing else output.strip()
        raise RuntimeError(f'{label}: shared-library unresolved: {detail}')


def _validate_request(
        context, camera_available: bool, cpu_available: bool, gpu_available: bool):
    mode = LaunchConfiguration("perception_mode").perform(context).strip()
    if mode not in {"off", "cpu", "gpu"}:
        raise RuntimeError("perception_mode wajib tepat: off, cpu, atau gpu")
    if mode == "off":
        if not camera_available:
            raise RuntimeError("perception_mode=off tetapi camera_only_node tidak terpasang")
        return []
    if mode == "cpu":
        if not cpu_available:
            raise RuntimeError(
                "perception_mode=cpu tetapi perception_cpu_node LibTorch tidak terpasang")
        model = _runtime_path(LaunchConfiguration("pt_model_path").perform(context))
        if model.suffix.lower() not in {".pt", ".torchscript"}:
            raise RuntimeError(f"model CPU harus TorchScript .pt/.torchscript: {model}")
        if not model.is_file() or model.stat().st_size <= 0:
            raise RuntimeError(f"model CPU tidak ditemukan/kosong: {model}. "
                               "Jalankan: python3 src/perception/tools/download_yolopv2.py")
        share = Path(get_package_share_directory("perception"))
        _validate_dynamic_links(share.resolve().parents[1] / "lib/perception/perception_cpu_node", "CPU")
        return []
    if not gpu_available:
        raise RuntimeError("perception_mode=gpu tetapi executable CUDA/TensorRT tidak terpasang")
    engine = _runtime_path(LaunchConfiguration("engine_path").perform(context))
    if engine.suffix.lower() != ".engine":
        raise RuntimeError(f"model GPU harus .engine: {engine}")
    if not engine.is_file() or engine.stat().st_size <= 0:
        raise RuntimeError(f"TensorRT engine tidak ditemukan/kosong: {engine}")
    share = Path(get_package_share_directory("perception"))
    _validate_dynamic_links(share.resolve().parents[1] / "lib/perception/perception_node", "GPU")
    return []


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("perception")
    default_pt_model = _discover_cpu_model("auto", share)
    lib_dir = Path(share).resolve().parents[1] / "lib" / "perception"
    camera_executable = lib_dir / "camera_only_node"
    cpu_executable = lib_dir / "perception_cpu_node"
    gpu_executable = lib_dir / "perception_node"
    camera_available = camera_executable.is_file() and os.access(camera_executable, os.X_OK)
    cpu_available = cpu_executable.is_file() and os.access(cpu_executable, os.X_OK)
    gpu_available = gpu_executable.is_file() and os.access(gpu_executable, os.X_OK)
    config = LaunchConfiguration("config_file")
    mode = LaunchConfiguration("perception_mode")
    respawn = PythonExpression(["'", LaunchConfiguration("respawn_node"), "' == 'true'"])
    cpu_enabled = PythonExpression(["'", mode, "' == 'cpu' and ", str(cpu_available)])
    gpu_enabled = PythonExpression(["'", mode, "' == 'gpu' and ", str(gpu_available)])
    mode_off = PythonExpression(["'", mode, "' == 'off' and ", str(camera_available)])
    invalid = PythonExpression(["'", mode, "' not in ['off', 'cpu', 'gpu']"])
    unavailable = PythonExpression([
        "('", mode, "' == 'off' and not ", str(camera_available), ") or ('",
        mode, "' == 'cpu' and not ", str(cpu_available), ") or ('",
        mode, "' == 'gpu' and not ", str(gpu_available), ")",
    ])

    common = {
        "rgb_device": LaunchConfiguration("rgb_device"),
        "rgb_width": ParameterValue(LaunchConfiguration("rgb_width"), value_type=int),
        "rgb_height": ParameterValue(LaunchConfiguration("rgb_height"), value_type=int),
        "fps": ParameterValue(LaunchConfiguration("camera_fps"), value_type=int),
        "strict_camera_mode": ParameterValue(
            LaunchConfiguration("strict_camera_mode"), value_type=bool),
        "v4l2_pixel_format": LaunchConfiguration("v4l2_pixel_format"),
        "allow_mjpeg_cpu_fallback": ParameterValue(
            LaunchConfiguration("allow_mjpeg_cpu_fallback"), value_type=bool),
        "use_v4l2_userptr_zero_copy": ParameterValue(
            LaunchConfiguration("use_v4l2_userptr_zero_copy"), value_type=bool),
        "camera_metric_calibration_validated": ParameterValue(
            LaunchConfiguration("camera_metric_calibration_validated"), value_type=bool),
        "lane_safety_enabled": ParameterValue(
            LaunchConfiguration("enable_lane_safety"), value_type=bool),
        "control_mode": ParameterValue(LaunchConfiguration("lane_safety_mode"), value_type=str),
    }

    camera = Node(
        package="perception", executable="camera_only_node", name="perception_camera",
        output="screen", emulate_tty=True, condition=IfCondition(mode_off),
        respawn=respawn, respawn_delay=5.0,
        parameters=[config, {**common}],
    )
    cpu = Node(
        package="perception", executable="perception_cpu_node", name="perception",
        output="screen", emulate_tty=True, condition=IfCondition(cpu_enabled),
        respawn=respawn, respawn_delay=5.0,
        parameters=[config, {
            "pt_model_path": LaunchConfiguration("pt_model_path"),
            "cpu_inference_fps": ParameterValue(
                LaunchConfiguration("cpu_inference_fps"), value_type=float),
            "cpu_threads": ParameterValue(LaunchConfiguration("cpu_threads"), value_type=int),
            **common,
        }],
    )
    gpu = Node(
        package="perception", executable="perception_node", name="perception",
        output="screen", emulate_tty=True, condition=IfCondition(gpu_enabled),
        respawn=respawn, respawn_delay=5.0,
        parameters=[config, {
            "engine_path": LaunchConfiguration("engine_path"),
            "gpu_device": ParameterValue(LaunchConfiguration("gpu_device"), value_type=int),
            **common,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", default_value=os.path.join(share, "config", "astra_yolop_gpu.yaml")),
        DeclareLaunchArgument("perception_mode", default_value="cpu"),
        DeclareLaunchArgument("engine_path", default_value=os.environ.get(
            "YOLOP_ENGINE_PATH", "/home/sirobo/ros/models/yolopv2.engine")),
        DeclareLaunchArgument("pt_model_path", default_value=default_pt_model),
        DeclareLaunchArgument("cpu_inference_fps", default_value="2.0"),
        DeclareLaunchArgument("cpu_threads", default_value="0"),
        DeclareLaunchArgument("rgb_device", default_value="auto"),
        DeclareLaunchArgument("rgb_width", default_value="1280"),
        DeclareLaunchArgument("rgb_height", default_value="720"),
        DeclareLaunchArgument("camera_fps", default_value="30"),
        DeclareLaunchArgument("gpu_device", default_value="0"),
        DeclareLaunchArgument("strict_camera_mode", default_value="false"),
        DeclareLaunchArgument("v4l2_pixel_format", default_value="MJPEG"),
        DeclareLaunchArgument("allow_mjpeg_cpu_fallback", default_value="true"),
        DeclareLaunchArgument("use_v4l2_userptr_zero_copy", default_value="false"),
        DeclareLaunchArgument("camera_metric_calibration_validated", default_value="false"),
        DeclareLaunchArgument("enable_lane_safety", default_value="false"),
        DeclareLaunchArgument("lane_safety_mode", default_value="monitor"),
        DeclareLaunchArgument("respawn_node", default_value="true"),
        SetEnvironmentVariable("CUDA_MODULE_LOADING", "LAZY"),
        OpaqueFunction(function=_validate_request, args=[camera_available, cpu_available, gpu_available]),
        LogInfo(condition=IfCondition(mode_off), msg="[PERCEPTION] mode=OFF; camera-only aktif, model/inference tidak dijalankan"),
        LogInfo(condition=IfCondition(cpu_enabled), msg="[PERCEPTION] mode=CPU; yolopv2.pt dibaca langsung via LibTorch, tanpa ONNX"),
        LogInfo(condition=IfCondition(gpu_enabled), msg="[PERCEPTION] backend=GPU CUDA/TensorRT"),
        LogInfo(condition=IfCondition(invalid), msg="[PERCEPTION] ERROR perception_mode wajib off, cpu, atau gpu"),
        LogInfo(condition=IfCondition(unavailable),
                msg="[PERCEPTION] ERROR executable backend tidak tersedia pada build ini"),
        camera, cpu, gpu,
    ])
