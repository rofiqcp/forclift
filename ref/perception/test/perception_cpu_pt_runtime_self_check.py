#!/usr/bin/env python3
"""Dependency-light static contract for direct yolopv2.pt CPU runtime."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "src/astra_yolop_cpu_pt_node.cpp").read_text(encoding="utf-8")
cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
launch = (ROOT.parent / "navigation/launch/autonomous.launch.py").read_text(encoding="utf-8")

def require(cond, msg):
    if not cond:
        raise SystemExit("FAIL: " + msg)

for token in (
    'declare_parameter<std::string>("pt_model_path", "/home/sirobo/ros/models/yolopv2.pt")',
    'torch::jit::load(pt_model_path_, torch::kCPU)',
    'module_.eval()',
    'torch::InferenceMode',
    'tensor.permute({0, 3, 1, 2})',
    'prediction_heads.size() != 3U',
    'const torch::IValue element = list.get(i)',
    'anchor_heads.size() != 3U',
    'YOLOPv2 CPU TorchScript warm-up + 8-output contract: PASS',
    'resolveCpuModelPath',
    'resolveCpuThreadCount',
    'torch::set_num_interop_threads(1)',
    'pipeline_p95_ms',
    'capture_dropped_total',
):
    require(token in source, f"direct PT runtime missing: {token}")
require('readNetFromONNX' not in source, 'CPU source must not use OpenCV ONNX loader')
require('prepare_onnx' not in source, 'CPU source must not prepare ONNX')
require('find_package(Torch QUIET)' in cmake and 'PERCEPTION_CPU_AVAILABLE' in cmake, 'LibTorch capability detection missing')
require('importlib.util.find_spec' not in launch, 'autonomous launch must not perform legacy Python module probe')
print('PASS direct yolopv2.pt LibTorch runtime contract')

require('for (const auto & element : list)' not in source, 'GenericList proxy iteration must not call IValue methods directly')
require('set(CMAKE_MESSAGE_LOG_LEVEL ERROR)' in cmake, 'Torch optional Kineto warning suppression missing')

require('TORCH_CXX11_ABI' in cmake and 'compiled_with_cxx11_abi' in cmake, 'Torch CXX11 ABI guard missing')
