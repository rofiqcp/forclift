"""Idempotent PART-4 Stage-2 runtime migration for split LiDAR safety/navigation paths.

Persistent runtime YAML intentionally survives software upgrades, so package-default
changes alone are not enough.  This module updates only Stage-2-owned keys while
preserving every unrelated calibration/tuning value and taking timestamped backups.
"""
from __future__ import annotations

from pathlib import Path
import os
import shutil
import tempfile
import time
from typing import Any, Dict, Iterable, Tuple

import yaml


class LidarSafetyConfigError(RuntimeError):
    pass


def _runtime_root() -> Path:
    ws = Path(os.path.expanduser(os.environ.get('AGV_WS', '/home/otomasi2/ros')))
    return Path(os.path.expanduser(os.environ.get(
        'AGV_RUNTIME_CONFIG_ROOT', str(ws / 'config' / 'runtime'))))


def _seed(navigation_share: Path, filename: str) -> Path:
    """Seed one runtime YAML and recover only an unreadable persistent copy.

    Runtime configuration survives rebuilds.  A syntactically broken YAML from
    an older source revision otherwise aborts autonomous/gui launch before any
    sensor process is created.  Valid runtime tuning is preserved verbatim; only
    a file that cannot be parsed as a YAML mapping is backed up and replaced by
    the packaged default.
    """
    target_dir = _runtime_root() / 'navigation'
    target_dir.mkdir(parents=True, exist_ok=True)
    target = target_dir / filename
    source = navigation_share / 'config' / filename
    if not source.is_file():
        raise LidarSafetyConfigError(f'missing package default: {source}')
    if not target.exists():
        shutil.copy2(source, target)
        return target

    try:
        current = yaml.safe_load(target.read_text(encoding='utf-8'))
        if not isinstance(current, dict):
            raise ValueError('YAML root is not a mapping')
    except Exception:
        # Validate the fallback before touching the persistent runtime file.
        try:
            fallback = yaml.safe_load(source.read_text(encoding='utf-8'))
        except Exception as exc:
            raise LidarSafetyConfigError(f'cannot parse package default {source}: {exc}') from exc
        if not isinstance(fallback, dict):
            raise LidarSafetyConfigError(f'package default YAML root must be mapping: {source}')
        backup_dir = target_dir / '.invalid_runtime_backups'
        backup_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(target, backup_dir / f'{target.name}.{time.time_ns()}.bak')
        fd, tmp = tempfile.mkstemp(prefix=f'.{target.name}.', suffix='.repair.tmp', dir=str(target_dir))
        try:
            os.close(fd)
            shutil.copy2(source, tmp)
            os.replace(tmp, target)
        finally:
            if os.path.exists(tmp):
                os.unlink(tmp)
    return target


def _load(path: Path) -> Dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except Exception as exc:
        raise LidarSafetyConfigError(f'cannot parse {path}: {exc}') from exc
    if not isinstance(data, dict):
        raise LidarSafetyConfigError(f'YAML root must be mapping: {path}')
    return data


def _atomic_write(path: Path, data: Dict[str, Any]) -> bool:
    old = _load(path) if path.is_file() else None
    if old == data:
        return False
    backup_dir = path.parent / '.stage2_safety_backups'
    backup_dir.mkdir(parents=True, exist_ok=True)
    if path.exists():
        shutil.copy2(path, backup_dir / f'{path.name}.{time.time_ns()}.bak')
    fd, tmp = tempfile.mkstemp(prefix=f'.{path.name}.', suffix='.tmp', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as fh:
            yaml.safe_dump(data, fh, sort_keys=False, allow_unicode=True)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    if _load(path) != data:
        raise LidarSafetyConfigError(f'readback mismatch after writing {path}')
    return True


def _nested(data: Dict[str, Any], keys: Iterable[str]) -> Dict[str, Any]:
    cur: Any = data
    path = tuple(keys)
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            raise LidarSafetyConfigError(f'missing required config key: {"/".join(path)}')
        cur = cur[key]
    if not isinstance(cur, dict):
        raise LidarSafetyConfigError(f'expected mapping at: {"/".join(path)}')
    return cur


def _ros_parameters(data: Dict[str, Any], node_name: str) -> Dict[str, Any]:
    """Return a ROS 2 parameter mapping for a named node or wildcard file.

    ROS 2 parameter YAML legitimately supports a wildcard ``/**`` root.  The
    packaged LiDAR defaults intentionally use that form so the same transport
    settings remain valid if the process node name is remapped.  Older runtime
    migrations incorrectly required a literal ``lidar_node`` root and therefore
    crashed exactly when an empty runtime directory was seeded from the package
    default.  Accept node-specific and wildcard forms without rewriting operator
    configuration.
    """
    candidates = (node_name, f'/{node_name}', '/**', '**')
    for root in candidates:
        value = data.get(root)
        if not isinstance(value, dict):
            continue
        params = value.get('ros__parameters')
        if isinstance(params, dict):
            return params
    raise LidarSafetyConfigError(
        f'missing ROS parameters for {node_name}; expected one of: ' +
        ', '.join(f'{root}/ros__parameters' for root in candidates)
    )


def ensure_stage2_lidar_runtime(navigation_share: os.PathLike[str] | str) -> Dict[str, Any]:
    """Seed/migrate Stage-2-owned runtime keys without clobbering tuning."""
    share = Path(navigation_share)
    changed = []

    # New Stage-2 health config has no legacy equivalent. Seed it, then merge
    # newly introduced keys from package defaults while preserving tuned
    # numeric thresholds. Topic ownership is canonical and is enforced.
    health_path = _seed(share, 'lidar_safety.yaml')
    health_doc = _load(health_path)
    health_default = _load(share / 'config' / 'lidar_safety.yaml')
    hp = _nested(health_doc, ('lidar_safety_health', 'ros__parameters'))
    hpd = _nested(health_default, ('lidar_safety_health', 'ros__parameters'))
    for key, value in hpd.items():
        hp.setdefault(key, value)
    hp['scan_topic'] = '/scan_safety'
    hp['healthy_topic'] = '/lidar/safety_healthy'
    hp['health_text_topic'] = '/lidar/safety_health'

    # Safety-critical bounds are enforced at migration time as well as in the
    # GUI. Persistent YAML can be hand-edited, so GUI validation alone is not
    # an adequate motion-safety boundary. Values inside the range are preserved.
    bounds = {
        'scan_timeout_sec': (0.10, 0.60),
        'status_timeout_sec': (0.50, 2.50),
        'min_valid_beams': (3, 360),
        'min_valid_ratio': (0.005, 1.0),
        'min_scan_hz': (1.0, 20.0),
        'max_scan_hz': (2.0, 50.0),
        'recovery_good_scans': (2, 20),
        'max_invalid_packets_per_status': (0, 50),
        'max_malformed_packets_per_status': (0, 20),
        'max_checksum_failures_per_status': (0, 100),
        'publish_rate_hz': (2.0, 50.0),
    }
    integer_keys = {
        'min_valid_beams', 'recovery_good_scans',
        'max_invalid_packets_per_status', 'max_malformed_packets_per_status',
        'max_checksum_failures_per_status',
    }
    for key, (lo, hi) in bounds.items():
        default = hpd[key]
        value = hp.get(key, default)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            value = default
        value = max(lo, min(hi, value))
        hp[key] = int(value) if key in integer_keys else float(value)
    if hp['max_scan_hz'] <= hp['min_scan_hz']:
        hp['max_scan_hz'] = min(50.0, max(hpd['max_scan_hz'], hp['min_scan_hz'] + 1.0))
    hp['require_driver_connected'] = True
    hp['require_motor_running'] = True

    if _atomic_write(health_path, health_doc):
        changed.append(str(health_path))

    # V43 LiDAR transport/geometry hardening. Persistent lidar.yaml may come
    # from an older release, therefore package-default edits alone are not
    # sufficient. Enforce only protocol/range fields that are part of the
    # canonical T-mini Plus data path; preserve calibration/tuning fields.
    # T-mini Plus SH normal samples are 4-byte: intensity16 + distance16.
    lidar_path = _seed(share, 'lidar.yaml')
    lidar_doc = _load(lidar_path)
    lp = _ros_parameters(lidar_doc, 'lidar_node')
    lp['intensity_mode'] = True
    lp['intensity_bits'] = 16
    lp['strict_checksum'] = True
    lp['range_max'] = 6.0
    lp['spatial_filter_enabled'] = True
    lp['temporal_filter_enabled'] = True
    if _atomic_write(lidar_path, lidar_doc):
        changed.append(str(lidar_path))

    collision_path = _seed(share, 'collision_monitor.yaml')
    collision_doc = _load(collision_path)
    # One previous source revision accidentally wrote a geometry-only document
    # into collision_monitor.yaml.  It is valid YAML, so syntax recovery above
    # cannot detect it, but it aborts Stage-2 before LiDAR/IMU/ESC are launched.
    # Replace only this incompatible schema, preserving every valid runtime file.
    try:
        c = _nested(collision_doc, ('collision_monitor', 'ros__parameters'))
    except LidarSafetyConfigError:
        default_doc = _load(share / 'config' / 'collision_monitor.yaml')
        c_default = _nested(default_doc, ('collision_monitor', 'ros__parameters'))
        backup_dir = collision_path.parent / '.invalid_runtime_backups'
        backup_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(collision_path, backup_dir / f'{collision_path.name}.{time.time_ns()}.schema.bak')
        collision_doc = default_doc
        c = c_default
    c['source_timeout'] = 0.30
    for name, max_points in (
        ('FrontStop', 0), ('FrontSlow', 1), ('RearStop', 0), ('RearSlow', 1)):
        poly = _nested(c, (name,))
        poly['max_points'] = max_points
    scan = _nested(c, ('scan',))
    scan['topic'] = '/scan_safety'
    scan['source_timeout'] = 0.30
    if _atomic_write(collision_path, collision_doc):
        changed.append(str(collision_path))

    nav2_path = _seed(share, 'nav2_ackermann.yaml')
    nav2_doc = _load(nav2_path)
    amcl = _nested(nav2_doc, ('amcl', 'ros__parameters'))
    amcl['scan_topic'] = '/scan_nav'
    amcl['laser_max_range'] = 5.5
    for side in ('local_costmap', 'global_costmap'):
        params = _nested(nav2_doc, (side, side, 'ros__parameters'))
        obstacle = _nested(params, ('obstacle_layer', 'scan'))
        obstacle['topic'] = '/scan_nav'
        obstacle['obstacle_max_range'] = min(5.0, float(obstacle.get('obstacle_max_range', 5.0)))
        obstacle['raytrace_max_range'] = min(5.5, float(obstacle.get('raytrace_max_range', 5.5)))
    if _atomic_write(nav2_path, nav2_doc):
        changed.append(str(nav2_path))

    slam_path = _seed(share, 'slam_toolbox.yaml')
    slam_doc = _load(slam_path)
    slam_params = _nested(slam_doc, ('slam_toolbox', 'ros__parameters'))
    slam_params['scan_topic'] = '/scan_nav'
    slam_params['max_laser_range'] = 5.5
    slam_params['scan_buffer_maximum_scan_distance'] = 6.0
    # Do not pre-gate scans on wheel-odom movement. During mapping the scan
    # matcher must keep working if wheel feedback is unavailable or the AGV is
    # pushed manually. A 10 Hz time gate bounds CPU load.
    slam_params['minimum_travel_distance'] = 0.0
    slam_params['minimum_travel_heading'] = 0.0
    slam_params['minimum_time_interval'] = 0.10
    slam_params['scan_queue_size'] = max(5, int(slam_params.get('scan_queue_size', 10)))
    if _atomic_write(slam_path, slam_doc):
        changed.append(str(slam_path))

    for filename in ('hector.yaml', 'hector_autonomous.yaml'):
        path = _seed(share, filename)
        doc = _load(path)
        _nested(doc, ('hector_slam_node', 'ros__parameters'))['scan_topic'] = '/scan_nav'
        if _atomic_write(path, doc):
            changed.append(str(path))

    return {
        'runtime_root': str(_runtime_root()),
        'changed_files': changed,
        'scan_safety': '/scan_safety',
        'scan_navigation': '/scan_nav',
    }
