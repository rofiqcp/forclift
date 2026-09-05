#!/usr/bin/env python3
"""Dependency-light checks for the master GUI + latest Navigation BAB IV catalog."""
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

    # Latest TA structure has 32 Navigation leaves. 4.1.6 TF is intentionally
    # a manual ExperimentSpec (default TF rows), so 31 are navAdd blocks.
    if len(blocks) != 31:
        fail(f'expected 31 navAdd leaves + manual 4.1.6 TF leaf, found {len(blocks)} navAdd blocks')
    if 'spec.id = QStringLiteral("4.1.6")' not in catalog:
        fail('manual TF leaf 4.1.6 is missing')

    expected_headings = [
        '4.1.1 Pengujian Komunikasi Sensor',
        '4.1.2 Pengujian Data Raw LiDAR',
        '4.1.3 Pengujian Data Raw IMU',
        '4.1.4 Pengujian Data Raw Odometri',
        '4.1.5 Analisis Sinkronisasi LiDAR, IMU, dan Odometri',
        '4.1.6 Validasi Transformasi TF',
        '4.2.1 Hasil Mapping 1',
        '4.2.2 Hasil Mapping 2',
        '4.2.3 Hasil Mapping 3',
        '4.2.4 Loop Closure dan Drift',
        '4.2.5 Analisis Kualitas Struktur Map',
        '4.2.6 Perbandingan dan Pemilihan Map Terbaik',
        '4.2.7 Validasi Map Terpilih',
        '4.3.1 Akurasi Pose Statis AMCL',
        '4.3.2 Repeatability Pose AMCL',
        '4.3.3 Waktu Konvergensi Initial Pose',
        '4.3.4 Tuning min_particles / max_particles',
        '4.3.5 Pengujian Relocalization',
        '4.3.6 Pengujian Localization Dinamis',
        '4.3.7 Validasi AMCL Final',
        '4.4.1 Validasi Input Global Costmap',
        '4.4.2 Tuning minimum_turning_radius',
        '4.4.3 Pengujian Beberapa Start-Goal',
        '4.4.4 Repeatability Planner',
        '4.4.5 Analisis Feasibility Jalur terhadap Kinematika AGV',
        '4.4.6 Validasi Hybrid-A* Final',
        '4.5.1 Lintasan Lurus',
        '4.5.2 Belok 90°',
        '4.5.3 Koridor',
        '4.5.4 U-Turn',
        '4.5.5 Skenario Kompleks',
        '4.5.6 Rekapitulasi Performa Sistem',
    ]
    for heading in expected_headings:
        if heading not in catalog:
            fail(f'latest BAB IV heading missing: {heading}')

    # Old Navigation chapters must no longer be exposed in the active catalog.
    for obsolete in (
        '4.6 Pengujian dan Tuning AMCL',
        '4.7 Pengujian dan Tuning Global Costmap',
        '4.8 Pengujian dan Tuning Smac Hybrid-A*',
        '4.9 Validasi Navigasi End-to-End',
        '4.10 Rekapitulasi Konfigurasi Kandidat Akhir',
        '4.11 Rekap',
    ):
        if obsolete in catalog:
            fail(f'obsolete Navigation BAB IV group still present: {obsolete}')

    ids = []
    by_id = {}
    for block in blocks:
        strings = re.findall(r'"([^"]*)"', block)
        if len(strings) < 5:
            fail('malformed navAdd block')
        leaf_id = strings[2]
        ids.append(leaf_id)
        by_id[leaf_id] = block
    ids.append('4.1.6')
    if len(ids) != len(set(ids)):
        fail('duplicate Navigation leaf IDs')

    expected_counts = {'4.1': 6, '4.2': 7, '4.3': 7, '4.4': 6, '4.5': 6}
    for group, expected in expected_counts.items():
        actual = sum(item.startswith(group + '.') for item in ids)
        if actual != expected:
            fail(f'{group}: expected {expected} leaves, found {actual}')

    required_columns = {
        '4.1.2': ['Jarak Ref (m)', 'Mean LiDAR (m)', 'MAE (m)', 'RMSE (m)', 'Std Dev (m)', 'Valid Ratio (%)'],
        '4.2.6': ['Metrik', 'Map 1', 'Map 2', 'Map 3', 'Terbaik', 'Keputusan'],
        '4.3.1': ['GT X (m)', 'GT Y (m)', 'GT Yaw (deg)', 'AMCL X (m)', 'AMCL Y (m)', 'Error Posisi (cm)', 'Error Yaw (deg)'],
        '4.3.4': ['min_particles', 'max_particles', 'Waktu Konvergensi (s)', 'Error Posisi (cm)', 'CPU Mean (%)'],
        '4.4.2': ['minimum_turning_radius (m)', 'Planning Time (ms)', 'Path Length (m)', 'Feasible Fisik'],
        '4.4.3': ['Skenario', 'Start', 'Goal', 'Path Length (m)', 'Planning Time (ms)', 'Reverse/Cusp'],
        '4.5.6': ['Skenario', 'Percobaan', 'Berhasil', 'Success Rate (%)', 'Waktu Rata-rata (s)', 'Error Akhir Rata-rata (m)'],
    }
    for leaf_id, columns in required_columns.items():
        block = by_id.get(leaf_id)
        if block is None:
            fail(f'cannot inspect required leaf {leaf_id}')
        for column in columns:
            if f'"{column}"' not in block:
                fail(f'{leaf_id} missing column {column!r}')

    # 4.5.1..4.5.5 intentionally share one endColumns definition.
    for column in ('Run', 'Waktu (s)', 'Path Length (m)', 'Error Posisi Akhir (m)',
                   'Error Yaw Akhir (deg)', 'CTE RMSE (m)', 'Status'):
        if f'"{column}"' not in catalog:
            fail(f'4.5 shared end-to-end columns missing {column!r}')

    required_graphs = (
        'Gambar 4.1 Error pengukuran LiDAR terhadap jarak referensi',
        'Gambar 4.2 Perbandingan RMSE struktur tiga map',
        'Gambar 4.3 Error posisi akhir pada lima skenario navigasi',
    )
    for graph in required_graphs:
        if graph not in catalog:
            fail(f'BAB IV report graph binding missing: {graph}')

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

    # Only validate YAML bindings still exposed by the latest BAB IV Navigation UI.
    nav2 = yaml.safe_load((NAV / 'config' / 'nav2_ackermann.yaml').read_text(encoding='utf-8'))
    bindings = re.findall(
        r'navField\("[^"]+",\s*"[^"]+",\s*"(?:float|int|bool|yaml_readonly)",\s*"nav2",\s*"([^"]+)"',
        catalog,
    )
    for path in bindings:
        if nested(nav2, path) is None:
            fail(f'GUI Navigation binding does not resolve in source YAML: nav2:{path}')

    summary = (GUI / 'modules' / 'system_and_gnss_pages.cpp').read_text(encoding='utf-8')
    for marker in (
        'NAVIGATION BAB IV terbaru (TA_Ernanta_Revisi_Final.docx)',
        'currentId_.startsWith(QStringLiteral("4.2"))',
        'currentId_.startsWith(QStringLiteral("4.3"))',
        'currentId_.startsWith(QStringLiteral("4.4"))',
        'currentId_.startsWith(QStringLiteral("4.5"))',
    ):
        if marker not in summary:
            fail(f'latest Navigation summary dispatch marker missing: {marker}')

    print('PASS: latest TA BAB IV Navigation GUI = 32 leaves (4.1..4.5), report graphs, telemetry dispatch, and YAML bindings')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        raise SystemExit(1)
