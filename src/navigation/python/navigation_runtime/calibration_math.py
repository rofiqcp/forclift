"""Pure calibration mathematics used by the AGV GUI and automated tests.

This module deliberately contains no Qt/rclpy imports so the calibration
algorithms can be regression-tested on any host before deploying to Jetson.
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple


class CalibrationError(ValueError):
    pass


def _finite(value) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise CalibrationError("non-finite calibration sample")
    return value


def _mean(values: Sequence[float]) -> float:
    if not values:
        raise CalibrationError("no calibration samples")
    return sum(values) / len(values)


def _std(values: Sequence[float], mean: float | None = None) -> float:
    if not values:
        raise CalibrationError("no calibration samples")
    m = _mean(values) if mean is None else float(mean)
    return math.sqrt(sum((v - m) ** 2 for v in values) / len(values))


def estimate_imu_stationary(
    samples: Iterable[Mapping[str, float]],
    current_gyro_bias: Sequence[float] = (0.0, 0.0, 0.0),
    current_accel_bias: Sequence[float] = (0.0, 0.0, 0.0),
    *,
    gravity_mps2: float = 9.80665,
    level_positive_z: bool = True,
    min_samples: int = 100,
    max_gyro_std_rps: float = 0.03,
    max_accel_norm_std_mps2: float = 0.20,
) -> Dict[str, object]:
    rows = []
    for row in samples:
        try:
            rows.append({k: _finite(row[k]) for k in ("gx", "gy", "gz", "ax", "ay", "az")})
        except (KeyError, TypeError, CalibrationError):
            continue
    if len(rows) < int(min_samples):
        raise CalibrationError(f"need at least {min_samples} valid IMU samples, got {len(rows)}")

    means = {k: _mean([r[k] for r in rows]) for k in rows[0]}
    stds = {k: _std([r[k] for r in rows], means[k]) for k in rows[0]}
    accel_norms = [math.sqrt(r["ax"] ** 2 + r["ay"] ** 2 + r["az"] ** 2) for r in rows]
    accel_norm_mean = _mean(accel_norms)
    accel_norm_std = _std(accel_norms, accel_norm_mean)
    gyro_std_max = max(stds["gx"], stds["gy"], stds["gz"])

    current_g = [float(v) for v in current_gyro_bias]
    current_a = [float(v) for v in current_accel_bias]
    if len(current_g) != 3 or len(current_a) != 3:
        raise CalibrationError("bias vectors must contain exactly 3 values")

    # `/imu/data` is already corrected by the current configured bias.  The
    # residual mean therefore needs to be ADDED to the existing calibration.
    gyro_candidate = [
        current_g[0] + means["gx"],
        current_g[1] + means["gy"],
        current_g[2] + means["gz"],
    ]

    accel_candidate = list(current_a)
    if level_positive_z:
        expected = (0.0, 0.0, float(gravity_mps2))
        accel_candidate = [
            current_a[0] + means["ax"] - expected[0],
            current_a[1] + means["ay"] - expected[1],
            current_a[2] + means["az"] - expected[2],
        ]

    quality_ok = gyro_std_max <= float(max_gyro_std_rps) and accel_norm_std <= float(max_accel_norm_std_mps2)
    return {
        "sample_count": len(rows),
        "mean": means,
        "std": stds,
        "accel_norm_mean_mps2": accel_norm_mean,
        "accel_norm_std_mps2": accel_norm_std,
        "gyro_std_max_rps": gyro_std_max,
        "gyro_bias_rps": gyro_candidate,
        "accel_bias_mps2": accel_candidate,
        "level_positive_z": bool(level_positive_z),
        "quality_ok": bool(quality_ok),
        "quality_limits": {
            "max_gyro_std_rps": float(max_gyro_std_rps),
            "max_accel_norm_std_mps2": float(max_accel_norm_std_mps2),
        },
    }


def fit_steering_calibration(points: Iterable[Tuple[float, float]], *, max_residual_ratio: float = 0.02) -> Dict[str, object]:
    """Fit raw encoder ticks = intercept + slope * physical steering angle.

    `points` are `(angle_rad, raw_ticks)` captured from physical center/left/right
    measurements.  Positive ROS steering convention is left turn.
    """
    pts = [(_finite(a), _finite(t)) for a, t in points]
    if len(pts) < 3:
        raise CalibrationError("steering calibration requires at least center, left and right points")
    angles = [p[0] for p in pts]
    ticks = [p[1] for p in pts]
    if min(angles) >= -1e-6 or max(angles) <= 1e-6:
        raise CalibrationError("steering calibration requires physical points on both sides of center")
    am = _mean(angles); tm = _mean(ticks)
    denom = sum((a - am) ** 2 for a in angles)
    if denom <= 1e-12:
        raise CalibrationError("steering reference angles have insufficient span")
    slope = sum((a - am) * (t - tm) for a, t in pts) / denom
    intercept = tm - slope * am
    if abs(slope) < 10.0:
        raise CalibrationError("steering encoder slope is implausibly small")
    residuals = [t - (intercept + slope * a) for a, t in pts]
    rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))
    span = max(ticks) - min(ticks)
    if abs(span) < 10.0:
        raise CalibrationError("steering encoder tick span is too small")
    ratio = rms / abs(span)
    max_angle = min(abs(min(angles)), abs(max(angles)))
    return {
        "steering_zero_ticks": intercept,
        "steering_ticks_per_rad": abs(slope),
        "steering_sign": 1.0 if slope > 0.0 else -1.0,
        "max_steering_rad": max_angle,
        "residual_rms_ticks": rms,
        "residual_ratio": ratio,
        "quality_ok": ratio <= float(max_residual_ratio),
        "points": [{"angle_rad": a, "ticks": t} for a, t in pts],
    }


def calibrate_wheel_radius(
    old_radius_m: float,
    actual_distance_m: float,
    start_pose: Sequence[float],
    end_pose: Sequence[float],
    *,
    max_yaw_change_rad: float = 0.20,
    min_reported_distance_m: float = 0.20,
) -> Dict[str, object]:
    old = _finite(old_radius_m)
    actual = abs(_finite(actual_distance_m))
    if old <= 0.0 or actual <= 0.0:
        raise CalibrationError("wheel radius and physical distance must be positive")
    if len(start_pose) < 3 or len(end_pose) < 3:
        raise CalibrationError("start/end pose must contain x,y,yaw")
    sx, sy, syaw = map(_finite, start_pose[:3])
    ex, ey, eyaw = map(_finite, end_pose[:3])
    reported = math.hypot(ex - sx, ey - sy)
    yaw_delta = math.atan2(math.sin(eyaw - syaw), math.cos(eyaw - syaw))
    if reported < float(min_reported_distance_m):
        raise CalibrationError(f"reported odometry distance is too short ({reported:.3f} m)")
    if abs(yaw_delta) > float(max_yaw_change_rad):
        raise CalibrationError(f"test was not straight enough (yaw change {math.degrees(yaw_delta):.2f} deg)")
    scale = actual / reported
    candidate = old * scale
    if not 0.5 <= scale <= 1.5:
        raise CalibrationError(f"wheel scale {scale:.3f} is outside conservative 0.5..1.5 bounds")
    return {
        "reported_distance_m": reported,
        "actual_distance_m": actual,
        "yaw_change_rad": yaw_delta,
        "scale": scale,
        "wheel_radius_m": candidate,
        "quality_ok": True,
    }


def lidar_safety_baseline(
    samples: Iterable[Mapping[str, float]],
    *,
    current_min_valid_beams: int,
    current_min_valid_ratio: float,
) -> Dict[str, object]:
    beams: List[float] = []
    ratios: List[float] = []
    for row in samples:
        try:
            b = _finite(row["valid_beams"])
            r = _finite(row["valid_ratio"])
        except (KeyError, TypeError, CalibrationError):
            continue
        if b >= 0 and 0.0 <= r <= 1.0:
            beams.append(b); ratios.append(r)
    if len(beams) < 30:
        raise CalibrationError("LiDAR baseline needs at least 30 safety scans")
    beams_sorted = sorted(beams); ratios_sorted = sorted(ratios)
    idx = max(0, min(len(beams_sorted) - 1, int(math.floor(0.05 * (len(beams_sorted) - 1)))))
    p05_beams = beams_sorted[idx]
    p05_ratio = ratios_sorted[idx]
    # Calibration is not allowed to silently weaken safety.  It may only keep
    # or tighten the existing threshold; loosening requires explicit engineering review.
    suggested_beams = max(int(current_min_valid_beams), max(3, int(math.floor(0.25 * p05_beams))))
    suggested_ratio = max(float(current_min_valid_ratio), max(0.01, 0.25 * p05_ratio))
    return {
        "sample_count": len(beams),
        "p05_valid_beams": p05_beams,
        "p05_valid_ratio": p05_ratio,
        "mean_valid_beams": _mean(beams),
        "mean_valid_ratio": _mean(ratios),
        "suggested_min_valid_beams": suggested_beams,
        "suggested_min_valid_ratio": suggested_ratio,
        "never_loosened": True,
        "quality_ok": p05_beams >= 3 and p05_ratio >= 0.01,
    }
