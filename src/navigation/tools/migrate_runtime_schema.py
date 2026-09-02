#!/usr/bin/env python3
from __future__ import annotations
import json
from ament_index_python.packages import get_package_share_directory
from navigation_runtime.runtime_schema import ensure_runtime_schema


def main():
    state = ensure_runtime_schema(
        get_package_share_directory('navigation'),
        get_package_share_directory('esc'),
        get_package_share_directory('yolo_obstacle_detection_ros2'))
    print(json.dumps(state, indent=2, sort_keys=True, default=str))
    raise SystemExit(0 if state.get('valid') else 2)


if __name__ == '__main__': main()
