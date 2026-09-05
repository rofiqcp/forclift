#!/usr/bin/env python3
"""Dependency-light checks for the master GUI + navigation BAB IV catalog."""
from __future__ import annotations

import re
import sys
from pathlib import Path

import yaml


GUI = Path(__file__).resolve().parents[1]
NAV = GUI.parent


def fail(message: str) -> None:
    raise AssertionError(message)


def nested(data, dotted: str):
    current = data
    for token in dotted.split('.'):
        if not isinstance(current, dict) or token not in current:
            return None
        current = current[token]
    return current


def main() -> int:
    catalog = (GUI / 'agv_navigation_bab4_catalog.inc').read_text(encoding='utf-8')
    blocks = re.findall(r'navAdd\((.*?)\);', catalog, flags=re.S)
    if len(blocks) != 68:
        fail(f'expected 68 navigation acquisition leaves (14 for DOCX 4.1 + 54 existing), found {len(blocks)}')

    required_41_tables = [
        'Tabel 4.1 Pengujian komunikasi sensor saat AGV diam',
        'Tabel 4.2 Stabilitas komunikasi pada kondisi aktuator berbeda',
        'Tabel 4.3 Contoh raw LaserScan',
        'Tabel 4.4 Akurasi LiDAR terhadap jarak referensi',
        'Tabel 4.5 Pengaruh getaran dan aktuator terhadap LiDAR',
        'Tabel 4.6 Kualitas LaserScan saat AGV bergerak',
        'Tabel 4.7 Contoh raw data IMU',
        'Tabel 4.8 Akurasi yaw IMU pada delapan orientasi',
        'Tabel 4.9 Drift yaw saat IMU diam',
        'Tabel 4.10 Pengaruh operasi aktuator terhadap yaw IMU',
        'Tabel 4.11 Contoh raw data odometri',
        'Tabel 4.12 Akurasi vx odometri',
        'Tabel 4.13 Respons perubahan kecepatan odometri',
        'Tabel 4.14 Sinkronisasi data sensor',
        'Tabel 4.15 Validasi transformasi TF',
        'Tabel 4.16 Rekapitulasi kelayakan input navigasi',
    ]
    for table in required_41_tables:
        if table not in catalog:
            fail(f'Navigation DOCX 4.1 table missing: {table}')
    for heading in (
        '4.1.1 Pengujian Komunikasi Sensor', '4.1.2 Pengujian Raw Data LiDAR',
        '4.1.3 Pengujian Raw Data IMU', '4.1.4 Pengujian Raw Data Odometri',
        '4.1.5 Analisis Sinkronisasi LiDAR, IMU, dan Odometri',
        '4.1.6 Validasi Transformasi TF', '4.1.7 Rekapitulasi Kelayakan Data Sensor'):
        if heading not in catalog:
            fail(f'Navigation DOCX 4.1 heading missing: {heading}')

    expected_counts = {
        '4.1': 14, '4.2': 6, '4.3': 5, '4.4': 5, '4.5': 8, '4.6': 9,
        '4.7': 5, '4.8': 9, '4.9': 5, '4.10': 1, '4.11': 1,
    }
    ids = []
    legacy_blocks = []
    legacy_table_numbers = []
    for block in blocks:
        strings = re.findall(r'"([^"]*)"', block)
        if len(strings) < 6:
            fail('malformed navAdd block')
        group, leaf_id = strings[0], strings[2]
        ids.append(leaf_id)
        if group != '4.1':
            legacy_blocks.append(block)
            match = re.search(r'Tabel 4\.(\d+)', block)
            if not match:
                fail(f'missing BAB IV table number in {leaf_id}')
            legacy_table_numbers.append(int(match.group(1)))
    for group, count in expected_counts.items():
        actual = sum(item.startswith(group + '.') for item in ids)
        if actual != count:
            fail(f'{group}: expected {count} navAdd leaves, found {actual}')
    if legacy_table_numbers != list(range(2, 56)):
        fail(f'existing tables 4.2..4.55 changed unexpectedly: {legacy_table_numbers}')
    if len(ids) != len(set(ids)):
        fail('duplicate navigation leaf IDs')

    required_columns = {
        2: ['Jarak ref (m)', 'RMSE (m)', 'σ (m)', 'Valid ratio (%)', 'Status'],
        7: ['Run', 'RMSE 2 m (m)', 'Valid ratio (%)', 'Dropout (%)', 'Scan rate (Hz)', 'Status'],
        8: ['Yaw ref', 'Yaw IMU', 'Error', '|Error|', 'Status'],
        13: ['v_ref (m/s)', 'vx odom (m/s)', 'Error', 'Error relatif', 'Status'],
        18: ['Parameter/kondisi', 'Map 1', 'Map 2', 'Map 3', 'Kontrol'],
        20: ['Ref', 'Fisik (m)', 'Map 1 (m)', 'Map 2 (m)', 'Map 3 (m)', 'Terbaik'],
        26: ['min/max', 'RMSE pos', 'RMSE yaw', 'Conv. time', 'CPU', 'Status'],
        34: ['Skenario', 'RMSE pos', 'RMSE yaw', 'Conv. time', 'Lost count', 'Success'],
        35: ['Resolution', 'Planning time', 'Min clearance', 'Path length', 'CPU', 'Status'],
        40: ['Rmin', 'Planning time', 'Path length', 'Tracking RMSE', 'Steering saturation', 'Status'],
        47: ['w_smooth', 'Path length', 'Curvature peak', 'Tracking RMSE', 'Clearance min', 'Status'],
        48: ['Skenario', 'Planning time', 'Path length', 'Min clearance', 'CTE', 'Success'],
        49: ['Run', 'Waktu (s)', 'Final e_pos', 'Final e_yaw', 'CTE', 'Status'],
        53: ['Run', 'Waktu (s)', 'Final e_pos', 'Final e_yaw', 'CTE', 'Status'],
        54: ['Lapisan', 'Parameter', 'Baseline/source', 'Kandidat akhir', 'Dasar'],
        55: ['Metrik', 'Baseline', 'Setelah tuning', 'Perubahan'],
    }
    for number, columns in required_columns.items():
        block = legacy_blocks[number - 2]
        for column in columns:
            if f'"{column}"' not in block:
                fail(f'Tabel 4.{number} missing column {column!r}')

    gui_core = (GUI / 'modules' / 'gui_core.cpp').read_text(encoding='utf-8')
    components = (GUI / 'modules' / 'experiment_components.cpp').read_text(encoding='utf-8')
    main_window = (GUI / 'modules' / 'main_window.cpp').read_text(encoding='utf-8')
    for marker in ('#d8b033', '#101215', '#171a1f'):
        if marker not in gui_core:
            fail(f'master visual marker {marker} is missing')
    for marker in ('menuPopup', 'floatingMenuButton', 'Navigasi', 'Persepsi', 'ESC'):
        if marker not in main_window:
            fail(f'master GUI menu marker {marker} is missing')
    for marker in ('std::llround', 'fkind == QStringLiteral("bool")', 'store->set(fp, v)'):
        if marker not in components:
            fail(f'YAML editor contract marker {marker} is missing')

    nav2_path = NAV / 'config' / 'nav2_ackermann.yaml'
    lidar_path = NAV / 'config' / 'lidar.yaml'
    nav2 = yaml.safe_load(nav2_path.read_text(encoding='utf-8'))
    lidar = yaml.safe_load(lidar_path.read_text(encoding='utf-8'))
    bindings = re.findall(
        r'navField\("[^"]+",\s*"[^"]+",\s*"(?:float|int|bool)",\s*"(nav2|lidar)",\s*"([^"]+)"',
        catalog,
    )
    allowed_insertions = {
        'planner_server.ros__parameters.GridBased.downsampling_factor',
        'planner_server.ros__parameters.GridBased.smoother.w_smooth',
    }
    for store, path in bindings:
        value = nested(nav2 if store == 'nav2' else lidar, path)
        if value is None and path not in allowed_insertions:
            fail(f'GUI tuning binding does not resolve in source YAML: {store}:{path}')
    if 'patchExisting' not in gui_core or 'Sisipkan scalar yang memang belum ada' not in gui_core:
        fail('safe scalar insertion support is missing for allowed YAML additions')

    print('PASS: master GUI shell, DOCX 4.1 Tables 4.1..4.16, existing Tables 4.2..4.55 unchanged, telemetry, and YAML bindings')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        raise SystemExit(1)
