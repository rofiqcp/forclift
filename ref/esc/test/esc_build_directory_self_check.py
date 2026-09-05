#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
cmake = (ROOT / 'CMakeLists.txt').read_text(errors='ignore')
required = [
    'file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/motor_teleop.dir/src")',
    'file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/ackermann_controller_server.dir/src")',
]
for token in required:
    if token not in cmake:
        raise SystemExit(f'FAIL missing defensive depfile directory rule: {token}')
print('PASS ESC depfile build-directory guard')
