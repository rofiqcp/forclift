#!/usr/bin/env bash
set -euo pipefail
NAV="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PASS=0; FAIL=0
ok(){ echo "[PASS] $*"; PASS=$((PASS+1)); }
bad(){ echo "[FAIL] $*" >&2; FAIL=$((FAIL+1)); }
need(){ local label="$1" file="$2" pattern="$3"; if grep -Eq "$pattern" "$file"; then ok "$label"; else bad "$label"; fi; }
forbid(){ local label="$1" file="$2" pattern="$3"; if grep -Eq "$pattern" "$file"; then bad "$label"; else ok "$label"; fi; }

echo '=== V45 warning-free Humble build verifier ==='
need 'mapping reset uses stable QStringList storage' "$NAV/src/mapping_gui.cpp" 'static const QStringList extensions'
need 'mapping reset iterates QStringList by const reference' "$NAV/src/mapping_gui.cpp" 'for \(const QString[[:space:]]*&[[:space:]]*ext[[:space:]]*:[[:space:]]*extensions\)'
forbid 'temporary initializer-list range loop removed' "$NAV/src/mapping_gui.cpp" 'for \(const QString[^:]*:[[:space:]]*\{QStringLiteral' 
need 'CMake detects Python interpreter before pytest registration' "$NAV/CMakeLists.txt" 'find_package\(Python3 COMPONENTS Interpreter QUIET\)'
need 'CMake checks import pytest quietly' "$NAV/CMakeLists.txt" 'COMMAND "\$\{Python3_EXECUTABLE\}" -c "import pytest"'
need 'pytest tests only registered when available' "$NAV/CMakeLists.txt" 'if\(NAVIGATION_PYTEST_AVAILABLE\)'
need 'pytest absence is STATUS, not WARNING' "$NAV/CMakeLists.txt" 'message\(STATUS "navigation: pytest not installed; optional Python unit tests are skipped"\)'
forbid 'unconditional ament_cmake_pytest find removed' "$NAV/CMakeLists.txt" '^  find_package\(ament_cmake_pytest REQUIRED\)$'
need 'V44 mixed integer fix preserved' "$NAV/src/scan_self_filter.cpp" 'std::max\(1, static_cast<int>\(declare_parameter<int>'
need 'native C++ main GUI preserved' "$NAV/CMakeLists.txt" 'add_executable\(agv_gui_cpp src/agv_gui.cpp\)'
need 'native C++ mapping GUI preserved' "$NAV/CMakeLists.txt" 'add_executable\(mapping_gui_cpp src/mapping_gui.cpp\)'
need 'native C++ Goal Pose bridge preserved' "$NAV/CMakeLists.txt" 'add_executable\(goal_pose_nav2_bridge src/goal_pose_nav2_bridge.cpp\)'
need 'native C++ scan filter preserved' "$NAV/CMakeLists.txt" 'add_executable\(scan_self_filter src/scan_self_filter.cpp\)'
need 'V45 apply script installed' "$NAV/CMakeLists.txt" 'tools/APPLY_V45_ON_JETSON.sh'
need 'V45 marker installed' "$NAV/CMakeLists.txt" 'V45_WARNING_FREE_BUILD_FIX.txt'

echo
printf 'V45 STATIC RESULT: %d PASS / %d FAIL\n' "$PASS" "$FAIL"
[[ "$FAIL" -eq 0 ]]
