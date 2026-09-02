"""Persistent runtime migration for PART-5 Stage-2 mapping/planning safety."""
from __future__ import annotations

import os
import shutil
import time
from pathlib import Path
import yaml


# V39: Nav2 Humble + MPPI full-footprint collision checking requires the
# inflation field remains above the 0.40 m inscribed half-width while the
# full 1.30 x 0.80 m footprint stays authoritative for collision checking.
# 0.45 m avoids the previous 0.77 m over-inflation that closed narrow free-space
# corridors and prevented Smac Hybrid-A* from producing a path.
COMPACT_INFLATION_RADIUS_M = 0.45
COMPACT_COST_SCALING_FACTOR = 12.0


def _runtime_nav_dir() -> Path:
    ws = Path(os.environ.get('AGV_WS', '/home/otomasi2/ros')).expanduser()
    root = Path(os.environ.get('AGV_RUNTIME_CONFIG_ROOT', str(ws / 'config' / 'runtime'))).expanduser()
    path = root / 'navigation'
    path.mkdir(parents=True, exist_ok=True)
    return path


def _atomic_yaml(path: Path, data) -> None:
    tmp = path.with_name(path.name + '.tmp')
    with tmp.open('w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    # Parse readback so truncated/invalid writes never look successful.
    with path.open('r', encoding='utf-8') as f:
        yaml.safe_load(f)


def ensure_stage5_planning_runtime(package_share: str) -> Path:
    """Migrate persistent Nav2 YAML while preserving prior tuning values."""
    runtime = _runtime_nav_dir()
    path = runtime / 'nav2_ackermann.yaml'
    src = Path(package_share) / 'config' / 'nav2_ackermann.yaml'
    if not path.exists() and src.is_file():
        shutil.copy2(src, path)
    if not path.is_file():
        raise RuntimeError('nav2_ackermann.yaml is unavailable')

    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    before = yaml.safe_dump(data, sort_keys=False)

    g = data.setdefault('global_costmap', {}).setdefault('global_costmap', {}).setdefault('ros__parameters', {})
    g['track_unknown_space'] = True
    plugins = list(g.get('plugins') or [])
    # Canonical production order: static -> live marking/clearing -> inflation.
    canonical = ['static_layer', 'obstacle_layer', 'inflation_layer']
    g['plugins'] = canonical
    static = g.setdefault('static_layer', {})
    static['plugin'] = 'nav2_costmap_2d::StaticLayer'
    static['enabled'] = True
    # /map is activated by the same lifecycle milestone used by AMCL and is
    # therefore the only safe canonical source for the production global
    # costmap.  Do not let an older persistent /nav_map override reintroduce a
    # startup deadlock.
    static['map_topic'] = '/map'
    static['map_subscribe_transient_local'] = True
    static['subscribe_to_updates'] = False
    obstacle = g.setdefault('obstacle_layer', {})
    obstacle['plugin'] = 'nav2_costmap_2d::ObstacleLayer'
    obstacle['enabled'] = True
    obstacle['observation_sources'] = 'scan'
    scan = obstacle.setdefault('scan', {})
    scan['topic'] = '/scan_nav'
    scan['data_type'] = 'LaserScan'
    scan['clearing'] = True
    scan['marking'] = True
    scan['max_obstacle_height'] = float(scan.get('max_obstacle_height', 2.0))
    scan['obstacle_max_range'] = min(8.0, max(0.5, float(scan.get('obstacle_max_range', 5.0))))
    scan['obstacle_min_range'] = min(1.0, max(0.0, float(scan.get('obstacle_min_range', 0.12))))
    scan['raytrace_max_range'] = min(10.0, max(scan['obstacle_max_range'], float(scan.get('raytrace_max_range', 6.0))))
    scan['raytrace_min_range'] = min(1.0, max(0.0, float(scan.get('raytrace_min_range', 0.0))))
    scan['inf_is_valid'] = True
    # Short persistence: enough for global replanning without baking transient people into the map.
    scan['observation_persistence'] = min(2.0, max(0.0, float(scan.get('observation_persistence', 0.75))))

    # Persistent runtime YAML survives rebuilds.  Repair older V28/V35 values
    # here so a stale 0.55 m halo or non-zero padding cannot silently return.
    local = data.setdefault('local_costmap', {}).setdefault('local_costmap', {}).setdefault('ros__parameters', {})
    local['plugins'] = ['obstacle_layer', 'inflation_layer']
    for costmap in (local, g):
        costmap['footprint_padding'] = 0.0
        inflation = costmap.setdefault('inflation_layer', {})
        inflation['plugin'] = 'nav2_costmap_2d::InflationLayer'
        inflation['enabled'] = True
        inflation['inflation_radius'] = COMPACT_INFLATION_RADIUS_M
        inflation['cost_scaling_factor'] = COMPACT_COST_SCALING_FACTOR
        inflation['inflate_unknown'] = False
        inflation['inflate_around_unknown'] = False

    planner = data.setdefault('planner_server', {}).setdefault('ros__parameters', {}).setdefault('GridBased', {})
    planner['allow_unknown'] = False

    controller = data.setdefault('controller_server', {}).setdefault('ros__parameters', {})
    follow = controller.setdefault('FollowPath', {})
    # MPPI visualization only publishes while FollowPath is executing.  Keep
    # it enabled in persistent runtime YAML and aggressively down-sample the
    # candidate pool to avoid saturating RViz/DDS on Jetson.
    follow['visualize'] = True
    visualizer = follow.setdefault('TrajectoryVisualizer', {})
    visualizer['trajectory_step'] = 20
    visualizer['time_step'] = 3

    after = yaml.safe_dump(data, sort_keys=False)
    if after != before:
        backup_dir = runtime / '.stage5_planning_backups'
        backup_dir.mkdir(exist_ok=True)
        stamp = time.time_ns()
        shutil.copy2(path, backup_dir / f'{path.stem}_{stamp}{path.suffix}')
        _atomic_yaml(path, data)
    return path
