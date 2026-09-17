#!/usr/bin/python3
"""Static contract for navigation/perception integration outside ESC firmware."""
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT / "src/navigation"
YOLO = ROOT / "src/yolo_obstacle_detection_ros2"
RUNTIME = ROOT / "config/runtime"
errors = []

def require(ok, message):
    if not ok:
        errors.append(message)

def text(path):
    return path.read_text(encoding="utf-8")

# Canonical workspace: active source/runtime may not fall back to the retired ROS tree.
for base in (NAV, YOLO, RUNTIME / "navigation", RUNTIME / "yolo_obstacle_detection_ros2"):
    for path in base.rglob("*"):
        if not path.is_file() or "__pycache__" in path.parts or ".git" in path.parts:
            continue
        if ".bak" in path.name or path.suffix in {".pyc", ".so", ".png", ".pgm"}:
            continue
        try:
            data = text(path)
        except (UnicodeDecodeError, OSError):
            continue
        require("/home/otomasi2/ros" not in data, f"retired workspace path: {path.relative_to(ROOT)}")
# BAB 4.2 must use the isolated LiDAR-only engine and must not own shared sensors.
shared = text(NAV / "launch/mapping_shared_slam.launch.py")
require("use_imu_rotation_gate': False" in shared, "mapping shared SLAM must disable IMU rotation gate")
require("/mapping/map" in shared and "/mapping/pose" in shared, "mapping shared outputs missing")
require("/esc/odom" not in shared and "ekf_node" not in shared, "mapping shared must not consume ESC/EKF")
bridge = text(NAV / "scripts/mapping_lidar_odom_bridge.py")
require("odometry_used': False" in bridge and "imu_used': False" in bridge, "LiDAR gate authority contract missing")

mapping_gui = text(NAV / "src/mapping_gui.cpp")
require("mapping_shared_slam.launch.py" in mapping_gui, "C++ mapping GUI does not use LiDAR-only mapper")
require("mapping_runtime.launch.py" not in mapping_gui, "C++ mapping GUI still starts full mapping runtime")
require("requestLidarStop" not in mapping_gui and "/lidar/stop_motor" not in mapping_gui,
        "mapping GUI must not stop shared LiDAR")
require("/mapping/map" in mapping_gui and "/mapping/pose" in mapping_gui,
        "mapping GUI is not bound to isolated mapping topics")

runtime_launch = text(NAV / "launch/mapping_runtime.launch.py")
require("fuser -k -9 /dev/ttyUSB" not in runtime_launch, "raw ttyUSB kill remains in mapping runtime")

manifest = yaml.safe_load(text(RUNTIME / "runtime_manifest.yaml"))
require(manifest.get("valid") is True, "runtime manifest invalid")
require(manifest.get("runtime_root") == str(RUNTIME), "runtime manifest root is not canonical")
require(not manifest.get("missing_files"), "runtime manifest has missing files")
# Keep generated/source trees free of temporary editor/build leftovers.
for base in (NAV, YOLO):
    for path in base.rglob("*"):
        if path.is_dir() and path.name == "__pycache__":
            errors.append(f"python cache present: {path.relative_to(ROOT)}")
        elif path.is_file() and (".web.bak." in path.name or path.name.endswith("~")):
            errors.append(f"backup artifact present: {path.relative_to(ROOT)}")

if errors:
    print("NON_ESC_RUNTIME_CONTRACT_FAIL")
    for item in errors:
        print("FAIL", item)
    sys.exit(1)
print("NON_ESC_RUNTIME_CONTRACT_PASS")
