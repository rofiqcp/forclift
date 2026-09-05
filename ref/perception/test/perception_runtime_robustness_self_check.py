#!/usr/bin/env python3
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[3]
per = root / 'src/perception'
nav = root / 'src/navigation'
esc = root / 'src/esc'
cmake = (per/'CMakeLists.txt').read_text()
auto = (nav/'launch/autonomous.launch.py').read_text()
direct = (per/'launch/astra_yolop.launch.py').read_text()
cpu = (per/'src/astra_yolop_cpu_pt_node.cpp').read_text()
camera_only = (per/'src/camera_only_node.cpp').read_text()
preflight = (per/'tools/perception_backend_preflight.py').read_text()
gpu = (per/'src/astra_yolop_gpu_node.cpp').read_text()
rviz = (nav/'rviz/autonomous.rviz').read_text()
teleop = (esc/'src/motor_teleop.cpp').read_text()

def need(cond, msg):
    if not cond:
        print('FAIL:', msg)
        raise SystemExit(1)

need('INSTALL_RPATH "${TORCH_RUNTIME_DIRS}"' in cmake, 'LibTorch install RPATH missing')
need('TORCH_PYTHON_LIB_DIR' in cmake, 'Torch Python lib directory discovery missing')
need('$<$<COMPILE_LANGUAGE:CXX>:-Wall>' in cmake, 'CXX warning flags not language-gated')
need('nice -n' not in auto and 'nice -n' not in direct, 'nice wrapper still present')
need('stop_on_cpu_exit' not in auto and 'stop_on_gpu_exit' not in auto, 'perception exit still shuts down stack')
need("DeclareLaunchArgument('perception_respawn', default_value='true')" in auto, 'perception respawn default not robust')
need('_validate_dynamic_links' in auto and "['ldd', str(executable)]" in auto, 'autonomous dynamic-link preflight missing')
need('_validate_dynamic_links' in direct, 'direct perception dynamic-link preflight missing')
need('/dev/v4l/by-id' in cpu and 'index < 64' in cpu and 'candidate.read(probe)' in cpu,
     'CPU camera discovery/probe not power-cycle robust')
need('/dev/v4l/by-id' in camera_only and 'index < 64' in camera_only and
     'probe.type() != CV_8UC3' in camera_only and 'frame.type() != CV_8UC3' in camera_only,
     'camera-only discovery must reject depth/mono/metadata V4L2 streams')
need('probe.type() == CV_8UC3' in cpu and 'frame.type() != CV_8UC3' in cpu,
     'CPU backend must reject non-BGR8 camera streams')
need('/dev/v4l/by-id' in gpu and 'i < 64' in gpu, 'GPU camera discovery not power-cycle robust')
need('rviz_default_plugins/InitializeTool' not in rviz and 'rviz_default_plugins/SetInitialPose' in rviz,
     'RViz Humble initial-pose tool not fixed')
need('RCLCPP_ERROR(get_logger(), "[JOY] Gamepad ada tetapi permission' not in teleop,
     'optional joystick permission still logged as ERROR')
need('converter_pt_to_onnx_to_engine' not in auto and '.onnx' not in auto,
     'CPU/GPU runtime launch references converter/ONNX')
need('_discover_cpu_model' in auto and 'YOLOPV2_PT_PATH' in auto, 'portable CPU .pt discovery missing')
for token in ('camera_candidate', 'os.access(camera_target, os.R_OK | os.W_OK)', '--require-camera'):
    need(token in preflight, f'camera hardware preflight missing: {token}')

print('PASS perception runtime robustness contract')
