"""Universal persistent-runtime schema, migration orchestration and profile activation.

The runtime tree is the single configuration source consumed by launch and GUI.
This module upgrades it conservatively: owned historical migrations run first,
then missing keys from the packaged defaults are recursively added without ever
overwriting an operator/calibration value that already exists.
"""
from __future__ import annotations

import hashlib
import os
import shutil
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, Mapping, MutableMapping, Tuple

import yaml

from .lidar_safety_config import ensure_stage2_lidar_runtime
from .localization_config import ensure_stage5_localization_runtime
from .planning_safety_config import ensure_stage5_planning_runtime
from .vehicle_geometry import load_runtime_geometry, sync_derived_configs


SCHEMA_VERSION = 1


def _root() -> Path:
    ws = Path(os.environ.get("AGV_WS", "/home/otomasi2/ros")).expanduser()
    return Path(os.environ.get("AGV_RUNTIME_CONFIG_ROOT", str(ws / "config" / "runtime"))).expanduser()


def _load(path: Path) -> Dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML root must be mapping: {path}")
    return data


def _atomic_yaml(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            yaml.safe_dump(dict(data), handle, sort_keys=False, allow_unicode=True)
            handle.flush()
            os.fsync(handle.fileno())
        parsed = yaml.safe_load(Path(tmp_name).read_text(encoding="utf-8"))
        if parsed != dict(data):
            raise RuntimeError(f"runtime manifest staged readback mismatch: {path}")
        os.replace(tmp_name, path)
        if _load(path) != dict(data):
            raise RuntimeError(f"runtime final readback mismatch: {path}")
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)


def _sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def _seed_package(share: Path, package: str, files: Mapping[str, Any]) -> Path:
    target = _root() / package
    target.mkdir(parents=True, exist_ok=True)
    for name in files:
        src = share / "config" / name
        dst = target / name
        if not dst.exists() and src.is_file():
            shutil.copy2(src, dst)
    return target


def _merge_missing(active: MutableMapping[str, Any], default: Mapping[str, Any], prefix: str = "") -> Tuple[bool, list[str]]:
    """Recursively add missing default keys, preserving every existing value."""
    changed = False
    added: list[str] = []
    for key, value in default.items():
        path = f"{prefix}.{key}" if prefix else str(key)
        if key not in active:
            # YAML round-trip gives a safe deep copy without importing copy for
            # arbitrary scalar/list/mapping structures.
            active[key] = yaml.safe_load(yaml.safe_dump(value, sort_keys=False))
            changed = True
            added.append(path)
        elif isinstance(active[key], dict) and isinstance(value, dict):
            sub_changed, sub_added = _merge_missing(active[key], value, path)
            changed = changed or sub_changed
            added.extend(sub_added)
    return changed, added


def _merge_package_defaults(share: Path, package: str, files: Mapping[str, Any], contract: str) -> Dict[str, Any]:
    """Merge only missing keys from package defaults into persistent runtime."""
    root = _root() / package
    result: Dict[str, Any] = {}
    backup_stamp = time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1_000_000_000:09d}"
    for name in files:
        src = share / "config" / name
        dst = root / name
        if not src.is_file() or not dst.is_file():
            continue
        default = _load(src)
        active = _load(dst)

        # ROS 2 parameter files that use named node sections must never carry a
        # stray root-level ros__parameters mapping. Older GUI geometry sync
        # revisions could append such a fragment to otherwise valid files
        # (notably collision_monitor.yaml and autonomy_health.yaml). rcl then
        # aborts the node before lifecycle startup. Remove only this impossible
        # root fragment when the packaged schema itself does not define it.
        removed_invalid_root = False
        if "ros__parameters" in active and "ros__parameters" not in default:
            active.pop("ros__parameters", None)
            removed_invalid_root = True

        changed, added = _merge_missing(active, default)
        changed = changed or removed_invalid_root
        if not changed:
            continue
        backup = root / ".runtime_schema_backups" / f"{contract}_{backup_stamp}" / name
        backup.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(dst, backup)
        _atomic_yaml(dst, active)
        result[name] = {"added_keys": added, "backup": str(backup)}
    return result


def ensure_runtime_schema(
    navigation_share: str | os.PathLike[str],
    esc_share: str | os.PathLike[str] | None = None,
    yolo_share: str | os.PathLike[str] | None = None,
) -> Dict[str, Any]:
    """Upgrade runtime config idempotently and emit one auditable manifest.

    Existing calibrated/tuned values always win. Missing keys introduced by a
    newer software contract are added from its packaged defaults. Historical
    migrations additionally remove or clamp keys whose old semantics are no
    longer safe.
    """
    nav_share = Path(navigation_share)
    schema = _load(nav_share / "config" / "runtime_schema.yaml")
    if int(schema.get("schema_version", 0)) != SCHEMA_VERSION:
        raise RuntimeError("unsupported runtime_schema.yaml version")
    managed = schema.get("managed_files", {})
    if not isinstance(managed, dict):
        raise RuntimeError("managed_files must be mapping")
    contract = str(schema.get("software_contract", "PART5_STAGE3"))

    # Migrate canonical geometry first so every later owner sees schema-v2.
    geometry_path, _geometry = load_runtime_geometry(nav_share)
    migrations: Dict[str, Any] = {
        "geometry": {"path": str(geometry_path)},
        "lidar": ensure_stage2_lidar_runtime(nav_share),
        "localization": ensure_stage5_localization_runtime(nav_share),
        "planning": str(ensure_stage5_planning_runtime(str(nav_share))),
    }
    if esc_share:
        migrations["geometry_sync"] = sync_derived_configs(nav_share, Path(esc_share))
    shares = {"navigation": nav_share}
    if esc_share:
        shares["esc"] = Path(esc_share)
    if yolo_share:
        shares["yolo_obstacle_detection_ros2"] = Path(yolo_share)

    default_merges: Dict[str, Any] = {}
    for package, share in shares.items():
        files = managed.get(package, {})
        if isinstance(files, dict):
            _seed_package(share, package, files)
            default_merges[package] = _merge_package_defaults(share, package, files, contract)

    file_state: Dict[str, Any] = {}
    missing: list[str] = []
    for package, files in managed.items():
        if not isinstance(files, dict):
            continue
        package_state: Dict[str, Any] = {}
        for name, revision in files.items():
            path = _root() / package / name
            if not path.is_file():
                missing.append(f"{package}/{name}")
                continue
            _load(path)
            package_state[name] = {
                "revision": int(revision),
                "sha256": _sha(path),
                "size_bytes": path.stat().st_size,
            }
        file_state[package] = package_state

    manifest = {
        "schema_version": SCHEMA_VERSION,
        "software_contract": contract,
        "generated_at_unix": time.time(),
        "runtime_root": str(_root()),
        "files": file_state,
        "missing_files": missing,
        "migrations": migrations,
        "default_key_merges": default_merges,
        "valid": not missing,
    }
    _atomic_yaml(_root() / "runtime_manifest.yaml", manifest)
    return manifest



def iter_runtime_yaml(root: Path):
    """Yield active runtime YAML while excluding internal backup/staging trees."""
    root = Path(root)
    if not root.is_dir():
        return
    for path in root.rglob("*.yaml"):
        rel = path.relative_to(root)
        if any(part.startswith(".") for part in rel.parts):
            continue
        yield path

def stage_profile(profile_dir: Path, package_dirs: Mapping[str, Path], staging_root: Path) -> Dict[str, Path]:
    """Copy a profile into an isolated staging tree; never touch active runtime."""
    profile_dir = Path(profile_dir)
    staging_root = Path(staging_root)
    if staging_root.exists():
        shutil.rmtree(staging_root)
    staged: Dict[str, Path] = {}
    for package in package_dirs:
        src = profile_dir / package
        dst = staging_root / package
        dst.mkdir(parents=True, exist_ok=True)
        if src.is_dir():
            for item in iter_runtime_yaml(src):
                rel = item.relative_to(src)
                (dst / rel).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(item, dst / rel)
        for item in iter_runtime_yaml(dst):
            _load(item)
        staged[package] = dst
    return staged


def restore_profile_backup(package_dirs: Mapping[str, Path], backup_root: Path, preexisting: Mapping[str, set[str]] | None = None) -> None:
    """Restore package YAML from a profile backup and remove newly introduced files."""
    backup_root = Path(backup_root)
    for package, dst_dir in package_dirs.items():
        dst_dir = Path(dst_dir)
        if preexisting is not None:
            allowed = preexisting.get(package, set())
            for active in list(iter_runtime_yaml(dst_dir)) if dst_dir.is_dir() else []:
                rel = str(active.relative_to(dst_dir))
                if rel not in allowed:
                    try:
                        active.unlink()
                    except OSError:
                        pass
        src_dir = backup_root / package
        if src_dir.is_dir():
            for backup in iter_runtime_yaml(src_dir):
                rel = backup.relative_to(src_dir)
                dst = dst_dir / rel
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(backup, dst)
                _load(dst)


def activate_staged_profile(
    staged_dirs: Mapping[str, Path],
    package_dirs: Mapping[str, Path],
    backup_root: Path,
) -> Dict[str, set[str]]:
    """Activate already-validated staged YAML with automatic rollback.

    Returns a snapshot of which YAML files existed before activation. The GUI
    can use it to rollback even if a separate post-activation validator fails.
    """
    backup_root = Path(backup_root)
    backup_root.mkdir(parents=True, exist_ok=True)
    replacements: list[tuple[Path, Path]] = []
    preexisting: Dict[str, set[str]] = {}
    try:
        for package, dst_dir in package_dirs.items():
            dst_dir = Path(dst_dir)
            preexisting[package] = set()
            for active in list(iter_runtime_yaml(dst_dir)) if dst_dir.is_dir() else []:
                rel = active.relative_to(dst_dir)
                preexisting[package].add(str(rel))
                backup = backup_root / package / rel
                backup.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(active, backup)

        # Prepare and parse EVERY temporary replacement before touching active files.
        for package, staged_dir in staged_dirs.items():
            dst_dir = Path(package_dirs[package])
            dst_dir.mkdir(parents=True, exist_ok=True)
            for src in iter_runtime_yaml(Path(staged_dir)):
                rel = src.relative_to(staged_dir)
                dst = dst_dir / rel
                dst.parent.mkdir(parents=True, exist_ok=True)
                _load(src)
                fd, tmp_name = tempfile.mkstemp(prefix=f".{dst.name}.", suffix=".profile.tmp", dir=str(dst.parent))
                os.close(fd)
                shutil.copy2(src, tmp_name)
                _load(Path(tmp_name))
                replacements.append((dst, Path(tmp_name)))

        for dst, tmp in replacements:
            os.replace(tmp, dst)
        for dst, _tmp in replacements:
            _load(dst)
        return preexisting
    except Exception:
        restore_profile_backup(package_dirs, backup_root, preexisting)
        raise
    finally:
        for _dst, tmp in replacements:
            try:
                if tmp.exists():
                    tmp.unlink()
            except OSError:
                pass
