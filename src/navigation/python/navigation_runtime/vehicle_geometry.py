"""Canonical AGV geometry loading, validation and derived-config synchronization.

The project previously duplicated wheelbase, steering limits, footprint and
turning radius across Nav2, ESC and URDF.  This module makes
navigation/config/vehicle_geometry.yaml the authoritative source and updates
only the geometry-owned fields in runtime YAML files.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import os
import shutil
import tempfile
import time
from typing import Any, Dict, Iterable, Tuple

import yaml


class GeometryError(RuntimeError):
    pass


def runtime_root() -> Path:
    ws = Path(os.path.expanduser((os.environ.get("AGV_ROOT") or os.environ.get("AGV_WS") or str(Path.home() / "forclift"))))
    root = Path(os.path.expanduser(os.environ.get("AGV_RUNTIME_CONFIG_ROOT", str(ws / "config" / "runtime"))))
    return root


def _atomic_yaml_write(path: Path, data: Dict[str, Any]) -> bool:
    """Atomically write YAML iff semantic content changed."""
    path.parent.mkdir(parents=True, exist_ok=True)
    old = None
    if path.is_file():
        try:
            old = yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception:
            old = None
    if old == data:
        return False
    if path.exists():
        backup_dir = path.parent / ".geometry_backups"
        backup_dir.mkdir(parents=True, exist_ok=True)
        backup = backup_dir / f"{path.name}.{time.time_ns()}.bak"
        shutil.copy2(path, backup)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            yaml.safe_dump(data, fh, sort_keys=False, allow_unicode=True)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp_name, path)
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)
    # Mandatory readback verification.
    check = yaml.safe_load(path.read_text(encoding="utf-8"))
    if check != data:
        raise GeometryError(f"readback mismatch after writing {path}")
    return True


def _seed_runtime_file(package_share: Path, package_name: str, filename: str) -> Path:
    target_dir = runtime_root() / package_name
    target_dir.mkdir(parents=True, exist_ok=True)
    target = target_dir / filename
    source = package_share / "config" / filename
    if not target.exists() and source.is_file():
        shutil.copy2(source, target)
    return target if target.is_file() else source


def load_geometry_file(path: os.PathLike[str] | str) -> Dict[str, Any]:
    path = Path(path)
    if not path.is_file():
        raise GeometryError(f"geometry file not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise GeometryError(f"geometry YAML must contain a mapping: {path}")
    validate_geometry(data, require_validated=False)
    return data


def _migrate_legacy_runtime_geometry(path: Path, template_path: Path) -> None:
    """Migrate pre-PART-4 geometry while preserving all legacy numeric values.

    This is intentionally narrow: only vehicle_geometry.yaml is migrated here.
    Every physical-validation flag is reset to false because old files did not
    prove REP-103 orientation or real measurements.
    """
    try:
        legacy = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception as exc:
        raise GeometryError(f"cannot parse legacy geometry {path}: {exc}") from exc
    if isinstance(legacy, dict) and legacy.get("schema_version") == 2:
        return
    template = yaml.safe_load(template_path.read_text(encoding="utf-8"))
    if not isinstance(template, dict) or template.get("schema_version") != 2:
        raise GeometryError(f"installed geometry template is not schema 2: {template_path}")
    old_vehicle = legacy.get("vehicle", legacy) if isinstance(legacy, dict) else {}
    if not isinstance(old_vehicle, dict):
        old_vehicle = {}
    migrated = template
    new_vehicle = migrated["vehicle"]
    # Preserve only known operational numeric keys.  Do not copy old validation
    # claims because schema-1 had no physical evidence contract.
    aliases = {"max_steering_rad": "max_steering_angle_rad"}
    numeric_keys = (
        "wheelbase_m", "wheel_radius_m", "body_length_m", "body_width_m",
        "footprint_half_length_m", "footprint_half_width_m",
        "max_steering_angle_rad", "min_turning_radius_m",
        "max_forward_speed_mps", "max_reverse_speed_mps",
        "nav2_max_forward_speed_mps", "nav2_max_reverse_speed_mps",
        "max_yaw_rate_rps",
    )
    for key in numeric_keys:
        source_key = key
        if source_key not in old_vehicle:
            for old_key, new_key in aliases.items():
                if new_key == key and old_key in old_vehicle:
                    source_key = old_key
                    break
        value = old_vehicle.get(source_key)
        if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value)):
            new_vehicle[key] = float(value)
    new_vehicle["wheelbase_validated"] = False
    new_vehicle["wheel_radius_validated"] = False
    new_vehicle["max_steering_validated"] = False
    new_vehicle["wheelbase_source"] = "MIGRATED_LEGACY_VALUE_FIELD_REVALIDATION_REQUIRED"
    migrated["frame_convention"]["rep103_alignment_validated"] = False
    _atomic_yaml_write(path, migrated)


def load_runtime_geometry(navigation_share: os.PathLike[str] | str) -> Tuple[Path, Dict[str, Any]]:
    share = Path(navigation_share)
    path = _seed_runtime_file(share, "navigation", "vehicle_geometry.yaml")
    source = share / "config" / "vehicle_geometry.yaml"
    if path.is_file():
        try:
            current = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except Exception as exc:
            raise GeometryError(f"cannot parse runtime geometry {path}: {exc}") from exc
        if not isinstance(current, dict) or current.get("schema_version") != 2:
            _migrate_legacy_runtime_geometry(path, source)
    return path, load_geometry_file(path)


def _num(mapping: Dict[str, Any], key: str, lo: float | None = None) -> float:
    value = mapping.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
        raise GeometryError(f"{key} must be a finite number")
    value = float(value)
    if lo is not None and value <= lo:
        raise GeometryError(f"{key} must be > {lo}")
    return value


def validate_geometry(data: Dict[str, Any], require_validated: bool = False) -> Dict[str, float]:
    schema = data.get("schema_version")
    if schema != 2:
        raise GeometryError(f"vehicle_geometry schema_version must be 2, got {schema!r}")
    frame = data.get("frame_convention")
    vehicle = data.get("vehicle")
    cad = data.get("cad_reference")
    if not isinstance(frame, dict) or not isinstance(vehicle, dict) or not isinstance(cad, dict):
        raise GeometryError("vehicle_geometry requires frame_convention, vehicle and cad_reference mappings")
    if frame.get("standard") != "REP-103":
        raise GeometryError("frame_convention.standard must be REP-103")
    if (frame.get("forward_axis"), frame.get("left_axis"), frame.get("up_axis")) != ("+X", "+Y", "+Z"):
        raise GeometryError("navigation frame must be +X forward, +Y left, +Z up")
    candidate = frame.get("cad_forward_axis_candidate")
    yaw = _num(frame, "cad_to_base_yaw_rad")
    expected = {"+Y": -math.pi / 2.0, "-Y": math.pi / 2.0}.get(candidate)
    if expected is None:
        raise GeometryError("cad_forward_axis_candidate must be '+Y' or '-Y'")
    if abs(math.atan2(math.sin(yaw - expected), math.cos(yaw - expected))) > 1e-6:
        raise GeometryError(
            f"cad_to_base_yaw_rad={yaw:.12f} is inconsistent with CAD forward {candidate}; expected {expected:.12f}"
        )

    wb = _num(vehicle, "wheelbase_m", 0.0)
    wr = _num(vehicle, "wheel_radius_m", 0.0)
    bl = _num(vehicle, "body_length_m", 0.0)
    bw = _num(vehicle, "body_width_m", 0.0)
    hl = _num(vehicle, "footprint_half_length_m", 0.0)
    hw = _num(vehicle, "footprint_half_width_m", 0.0)
    steer = _num(vehicle, "max_steering_angle_rad", 0.0)
    radius = _num(vehicle, "min_turning_radius_m", 0.0)
    if steer >= math.pi / 2.0:
        raise GeometryError("max_steering_angle_rad must be < pi/2")
    geometric_radius = wb / math.tan(steer)
    if radius + 1e-6 < geometric_radius:
        raise GeometryError(
            f"min_turning_radius_m={radius:.4f} is smaller than bicycle geometry {geometric_radius:.4f} m"
        )
    if abs(bl - 2.0 * hl) > 0.02 or abs(bw - 2.0 * hw) > 0.02:
        raise GeometryError("body_length/body_width must agree with footprint half extents within 2 cm")

    cad_wb = _num(cad, "axle_separation_m", 0.0)
    cad_steer_track = _num(cad, "steering_axle_track_m", 0.0)
    cad_fixed_track = _num(cad, "fixed_axle_track_m", 0.0)

    if require_validated:
        missing = []
        if frame.get("rep103_alignment_validated") is not True:
            missing.append("frame_convention.rep103_alignment_validated")
        for key in ("wheelbase_validated", "wheel_radius_validated", "max_steering_validated"):
            if vehicle.get(key) is not True:
                missing.append(f"vehicle.{key}")
        if missing:
            raise GeometryError("physical geometry not validated: " + ", ".join(missing))

    return {
        "cad_to_base_yaw_rad": yaw,
        "wheelbase_m": wb,
        "wheel_radius_m": wr,
        "body_length_m": bl,
        "body_width_m": bw,
        "footprint_half_length_m": hl,
        "footprint_half_width_m": hw,
        "max_steering_angle_rad": steer,
        "min_turning_radius_m": radius,
        "geometric_turning_radius_m": geometric_radius,
        "max_forward_speed_mps": _num(vehicle, "max_forward_speed_mps", 0.0),
        "max_reverse_speed_mps": _num(vehicle, "max_reverse_speed_mps", 0.0),
        "nav2_max_forward_speed_mps": _num(vehicle, "nav2_max_forward_speed_mps", 0.0),
        "nav2_max_reverse_speed_mps": _num(vehicle, "nav2_max_reverse_speed_mps", 0.0),
        "max_yaw_rate_rps": _num(vehicle, "max_yaw_rate_rps", 0.0),
        "cad_axle_separation_m": cad_wb,
        "cad_steering_track_m": cad_steer_track,
        "cad_fixed_track_m": cad_fixed_track,
    }


def _footprint_string(g: Dict[str, float]) -> str:
    x = g["footprint_half_length_m"]
    y = g["footprint_half_width_m"]
    return f"[[{x:.6g},{y:.6g}],[{x:.6g},{-y:.6g}],[{-x:.6g},{-y:.6g}],[{-x:.6g},{y:.6g}]]"


def _load_yaml(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        raise GeometryError(f"derived config not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise GeometryError(f"derived config must be YAML mapping: {path}")
    return data


def sync_derived_configs(
    navigation_share: os.PathLike[str] | str,
    esc_share: os.PathLike[str] | str | None = None,
    *,
    dry_run: bool = False,
) -> Dict[str, Any]:
    """Synchronize geometry-owned fields into persistent runtime YAML.

    Only geometry-owned fields are touched.  All controller/estimator tuning
    remains untouched.  Returns a structured summary suitable for logs/tests.
    """
    nav_share = Path(navigation_share)
    geometry_path, geometry_data = load_runtime_geometry(nav_share)
    g = validate_geometry(geometry_data, require_validated=False)
    changed = []

    nav2_path = _seed_runtime_file(nav_share, "navigation", "nav2_ackermann.yaml")
    nav2 = _load_yaml(nav2_path)
    planner = nav2["planner_server"]["ros__parameters"]["GridBased"]
    follow = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = nav2["velocity_smoother"]["ros__parameters"]
    local = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    glob = nav2["global_costmap"]["global_costmap"]["ros__parameters"]
    planner["minimum_turning_radius"] = g["min_turning_radius_m"]
    follow["AckermannConstraints"]["min_turning_r"] = g["min_turning_radius_m"]
    follow["vx_max"] = g["nav2_max_forward_speed_mps"]
    follow["vx_min"] = -g["nav2_max_reverse_speed_mps"]
    follow["wz_max"] = min(float(follow.get("wz_max", g["max_yaw_rate_rps"])), g["max_yaw_rate_rps"])
    smoother["max_velocity"][0] = g["nav2_max_forward_speed_mps"]
    smoother["min_velocity"][0] = -g["nav2_max_reverse_speed_mps"]
    smoother["max_velocity"][2] = min(float(smoother["max_velocity"][2]), g["max_yaw_rate_rps"])
    smoother["min_velocity"][2] = -min(abs(float(smoother["min_velocity"][2])), g["max_yaw_rate_rps"])
    fp = _footprint_string(g)
    local["footprint"] = fp
    glob["footprint"] = fp
    if not dry_run and _atomic_yaml_write(nav2_path, nav2):
        changed.append(str(nav2_path))

    if esc_share is not None:
        esc_share = Path(esc_share)
        for filename in ("ackermann_1_board.yaml", "ackermann_2_board.yaml"):
            path = _seed_runtime_file(esc_share, "esc", filename)
            data = _load_yaml(path)
            # Recover a schema-corrupted persistent ESC profile before geometry
            # synchronization.  Valid calibrated profiles are never replaced.
            try:
                p = data["esc_driver"]["ros__parameters"]
                if not isinstance(p, dict):
                    raise KeyError("esc_driver.ros__parameters")
                # A ROS 2 parameter file may not contain a bare root-level
                # ros__parameters mapping alongside the named node.  Older GUI
                # geometry synchronization could append exactly that fragment,
                # which made rcl_yaml_param_parser reject the whole ESC profile.
                if "ros__parameters" in data:
                    raise KeyError("stray root ros__parameters")
            except (KeyError, TypeError):
                default_path = esc_share / "config" / filename
                default_data = _load_yaml(default_path)
                try:
                    p = default_data["esc_driver"]["ros__parameters"]
                    if not isinstance(p, dict):
                        raise KeyError("esc_driver.ros__parameters")
                except (KeyError, TypeError) as exc:
                    raise GeometryError(
                        f"ESC profile missing esc_driver.ros__parameters: {default_path}") from exc
                if path.is_file() and path.resolve() != default_path.resolve() and not dry_run:
                    backup_dir = path.parent / ".invalid_runtime_backups"
                    backup_dir.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(path, backup_dir / f"{path.name}.{time.time_ns()}.schema.bak")
                    shutil.copy2(default_path, path)
                data = default_data
            p["wheelbase"] = g["wheelbase_m"]
            p["wheel_radius"] = g["wheel_radius_m"]
            p["max_steering_rad"] = g["max_steering_angle_rad"]
            if not dry_run and _atomic_yaml_write(path, data):
                changed.append(str(path))

        mux_path = _seed_runtime_file(esc_share, "esc", "esc_mux.yaml")
        mux = _load_yaml(mux_path)
        p = mux["esc_command_mux"]["ros__parameters"]
        p["wheelbase_m"] = g["wheelbase_m"]
        p["max_forward_speed_mps"] = g["max_forward_speed_mps"]
        p["max_reverse_speed_mps"] = g["max_reverse_speed_mps"]
        p["max_steering_angle_rad"] = g["max_steering_angle_rad"]
        p["max_yaw_rate_rps"] = g["max_yaw_rate_rps"]
        if not dry_run and _atomic_yaml_write(mux_path, mux):
            changed.append(str(mux_path))

        keyboard_path = _seed_runtime_file(esc_share, "esc", "keyboard_teleop.yaml")
        keyboard = _load_yaml(keyboard_path)
        p = keyboard["esc_keyboard_teleop"]["ros__parameters"]
        p["wheelbase_m"] = g["wheelbase_m"]
        p["max_forward_speed_mps"] = g["max_forward_speed_mps"]
        p["max_reverse_speed_mps"] = g["max_reverse_speed_mps"]
        p["max_steering_angle_rad"] = g["max_steering_angle_rad"]
        if not dry_run and _atomic_yaml_write(keyboard_path, keyboard):
            changed.append(str(keyboard_path))

    return {
        "geometry_path": str(geometry_path),
        "geometry": g,
        "validated": {
            "rep103": bool(geometry_data["frame_convention"].get("rep103_alignment_validated", False)),
            "wheelbase": bool(geometry_data["vehicle"].get("wheelbase_validated", False)),
            "wheel_radius": bool(geometry_data["vehicle"].get("wheel_radius_validated", False)),
            "max_steering": bool(geometry_data["vehicle"].get("max_steering_validated", False)),
        },
        "changed_files": changed,
    }
