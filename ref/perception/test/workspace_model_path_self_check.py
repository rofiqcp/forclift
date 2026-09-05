#!/usr/bin/env python3
from pathlib import Path
import re
ROOT = Path(__file__).resolve().parents[2]
expected = "/home/sirobo/ros/models/yolopv2.pt"
yaml = (ROOT/"perception/config/astra_yolop_gpu.yaml").read_text()
assert f"pt_model_path: {expected}" in yaml, "YAML default model path tidak sesuai workspace models"
for rel in ["perception/launch/astra_yolop.launch.py", "navigation/launch/gui.launch.py", "navigation/launch/autonomous.launch.py", "perception/src/astra_yolop_cpu_pt_node.cpp", "perception/tools/perception_backend_preflight.py"]:
    text=(ROOT/rel).read_text()
    assert "src/perception/models/yolopv2.pt" not in text, f"source-model fallback masih ada: {rel}"
assert expected in (ROOT/"perception/src/astra_yolop_cpu_pt_node.cpp").read_text(), "C++ CPU default model path tidak sesuai"
assert 'MODEL_TARGET="$ROOT/models/yolopv2.pt"' in (ROOT/"setup_minipc_cpu_yolopv2.sh").read_text()
print("PASS workspace model path contract: <workspace>/models/yolopv2.pt")
