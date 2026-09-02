"""PART-5 Stage-1 localization runtime migration.

Production 2-D AGV localization contract:
  /esc/odom + /imu/data -> robot_localization EKF -> odom -> base_footprint
  /scan_nav             -> AMCL                  -> map  -> odom
  /lidar/odom            -> diagnostics only (never fused by the production EKF)

Persistent runtime YAML survives package upgrades, so changing package defaults is
not sufficient. This module migrates only localization-owned keys and preserves
unrelated controller/calibration tuning with timestamped backups + readback.
"""
from __future__ import annotations

from pathlib import Path
import os
import shutil
import tempfile
import time
from typing import Any, Dict, Iterable

import yaml


class LocalizationConfigError(RuntimeError):
    pass


def _runtime_root() -> Path:
    ws = Path(os.path.expanduser(os.environ.get('AGV_WS', '/home/otomasi2/ros')))
    return Path(os.path.expanduser(os.environ.get(
        'AGV_RUNTIME_CONFIG_ROOT', str(ws / 'config' / 'runtime'))))


def _seed(share: Path, filename: str) -> Path:
    target_dir = _runtime_root() / 'navigation'
    target_dir.mkdir(parents=True, exist_ok=True)
    target = target_dir / filename
    source = share / 'config' / filename
    if not target.exists():
        if not source.is_file():
            raise LocalizationConfigError(f'missing package default: {source}')
        shutil.copy2(source, target)
        return target

    # A persistent runtime file survives colcon rebuilds.  One older source
    # revision could seed nav2_ackermann.yaml with an invalid list fragment in
    # velocity_smoother.  Repair only that unreadable runtime file here, before
    # autonomous.launch.py consumes it.  Valid operator tuning is never replaced.
    if filename == 'nav2_ackermann.yaml':
        try:
            current = yaml.safe_load(target.read_text(encoding='utf-8'))
            if not isinstance(current, dict):
                raise LocalizationConfigError(f'YAML root must be mapping: {target}')
        except Exception:
            if not source.is_file():
                raise LocalizationConfigError(f'missing package default: {source}')
            try:
                fallback = yaml.safe_load(source.read_text(encoding='utf-8'))
            except Exception as exc:
                raise LocalizationConfigError(f'cannot parse package default {source}: {exc}') from exc
            if not isinstance(fallback, dict):
                raise LocalizationConfigError(f'package default YAML root must be mapping: {source}')

            backup_dir = target_dir / '.invalid_runtime_backups'
            backup_dir.mkdir(parents=True, exist_ok=True)
            backup = backup_dir / f'{target.name}.{time.time_ns()}.bak'
            shutil.copy2(target, backup)

            fd, tmp = tempfile.mkstemp(prefix=f'.{target.name}.', suffix='.repair.tmp', dir=str(target_dir))
            try:
                os.close(fd)
                shutil.copy2(source, tmp)
                check = yaml.safe_load(Path(tmp).read_text(encoding='utf-8'))
                if not isinstance(check, dict):
                    raise LocalizationConfigError(f'repaired YAML root must be mapping: {source}')
                os.replace(tmp, target)
            finally:
                if os.path.exists(tmp):
                    os.unlink(tmp)
    return target


def _load(path: Path) -> Dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except Exception as exc:
        raise LocalizationConfigError(f'cannot parse {path}: {exc}') from exc
    if not isinstance(data, dict):
        raise LocalizationConfigError(f'YAML root must be mapping: {path}')
    return data


def _nested(data: Dict[str, Any], keys: Iterable[str]) -> Dict[str, Any]:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            raise LocalizationConfigError(f'missing required key: {"/".join(keys)}')
        cur = cur[key]
    if not isinstance(cur, dict):
        raise LocalizationConfigError(f'expected mapping: {"/".join(keys)}')
    return cur


def _atomic_write(path: Path, data: Dict[str, Any]) -> bool:
    if path.is_file() and _load(path) == data:
        return False
    backup_dir = path.parent / '.stage5_localization_backups'
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
        raise LocalizationConfigError(f'readback mismatch after writing {path}')
    return True


def _remove_sensor_prefix(params: Dict[str, Any], prefix: str) -> None:
    """Remove robot_localization source and every option belonging to it."""
    for key in list(params):
        if key == prefix or key.startswith(prefix + '_'):
            params.pop(key, None)


def _migrate_ekf(path: Path) -> bool:
    doc = _load(path)
    p = _nested(doc, ('ekf_filter_node', 'ros__parameters'))

    # Canonical production local-odometry ownership.
    p['odom0'] = '/esc/odom'
    p['odom0_config'] = [
        False, False, False,
        False, False, False,
        True, True, False,
        False, False, False,
        False, False, False,
    ]
    p['odom0_differential'] = False
    p['odom0_relative'] = False
    p.setdefault('odom0_queue_size', 50)
    p.setdefault('odom0_nodelay', True)
    p.setdefault('odom0_twist_rejection_threshold', 2.0)

    # `/lidar/odom` is intentionally diagnostic-only. Removing every odom1_*
    # option avoids stale runtime YAML accidentally keeping the old fusion path.
    _remove_sensor_prefix(p, 'odom1')

    p['imu0'] = '/imu/data'
    p['imu0_config'] = [
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, True,
        False, False, False,
    ]
    p['imu0_differential'] = False
    p['imu0_relative'] = False
    p['imu0_remove_gravitational_acceleration'] = False
    p.setdefault('imu0_queue_size', 100)
    p.setdefault('imu0_nodelay', True)
    p.setdefault('imu0_twist_rejection_threshold', 1.5)

    p['world_frame'] = 'odom'
    p['odom_frame'] = 'odom'
    p['base_link_frame'] = 'base_footprint'
    p['publish_tf'] = True
    if path.name == 'ekf_mapping_reference.yaml':
        # Mapping simultaneously runs scan matching, Ceres and RViz on Jetson.
        # A 20 Hz EKF is still faster than every downstream mapping consumer and
        # avoids the repeated 30 Hz deadline miss observed in log(6).
        p['frequency'] = 20.0
        p['print_diagnostics'] = False
    return _atomic_write(path, doc)


def ensure_stage5_localization_runtime(navigation_share: os.PathLike[str] | str) -> Dict[str, Any]:
    """Migrate persistent runtime files to the Stage-5 localization contract."""
    share = Path(navigation_share)
    changed = []

    for filename in ('ekf_autonomous.yaml', 'ekf_mapping_reference.yaml'):
        path = _seed(share, filename)
        if _migrate_ekf(path):
            changed.append(str(path))

    nav_path = _seed(share, 'nav2_ackermann.yaml')
    nav_doc = _load(nav_path)
    amcl = _nested(nav_doc, ('amcl', 'ros__parameters'))
    amcl['scan_topic'] = '/scan_nav'
    # Fail-safe initialization: AMCL must wait for an operator pose or an
    # explicitly configured map-bound known pose. Never silently assume origin.
    amcl['set_initial_pose'] = False
    amcl['always_reset_initial_pose'] = False
    amcl.pop('initial_pose', None)
    if _atomic_write(nav_path, nav_doc):
        changed.append(str(nav_path))

    # New config: seed, merge future keys, and enforce a measured initialization mode.
    startup_path = _seed(share, 'localization_startup.yaml')
    startup_doc = _load(startup_path)
    startup_default = _load(share / 'config' / 'localization_startup.yaml')
    p = _nested(startup_doc, ('autonomous_amcl_ready_gate', 'ros__parameters'))
    pd = _nested(startup_default, ('autonomous_amcl_ready_gate', 'ros__parameters'))
    for key, value in pd.items():
        p.setdefault(key, value)
    mode = str(p.get('initial_pose_mode', 'GLOBAL_LOCALIZATION')).strip().upper()
    if mode not in {'OPERATOR', 'KNOWN_POSE', 'GLOBAL_LOCALIZATION'}:
        p['initial_pose_mode'] = 'GLOBAL_LOCALIZATION'
    else:
        p['initial_pose_mode'] = mode
    p['require_map_hash_for_known_pose'] = True
    # Bound startup convergence values; these are readiness checks, not estimator tuning.
    bounds = {
        'bootstrap_delay_sec': (0.2, 5.0),
        'nomotion_update_period_sec': (0.2, 5.0),
        'max_cov_x': (0.01, 4.0),
        'max_cov_y': (0.01, 4.0),
        'max_cov_yaw': (0.01, 3.2),
        'stability_translation_m': (0.01, 1.0),
        'stability_yaw_rad': (0.01, 1.57),
        'diagnostic_timeout_sec': (10.0, 600.0),
    }
    for key, (lo, hi) in bounds.items():
        value = p.get(key, pd[key])
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            value = pd[key]
        p[key] = float(max(lo, min(hi, value)))
    samples = p.get('convergence_min_samples', pd['convergence_min_samples'])
    if not isinstance(samples, int) or isinstance(samples, bool):
        samples = pd['convergence_min_samples']
    p['convergence_min_samples'] = max(2, min(20, samples))
    if _atomic_write(startup_path, startup_doc):
        changed.append(str(startup_path))

    return {
        'runtime_root': str(_runtime_root()),
        'changed_files': changed,
        'ekf_sources': ['/esc/odom', '/imu/data'],
        'lidar_odom_role': 'diagnostic_only',
        'initial_pose_mode': p['initial_pose_mode'],
        'localization_startup': str(startup_path),
    }
