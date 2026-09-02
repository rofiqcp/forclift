"""Pure path tracking metrics shared by GUI and tests."""
from __future__ import annotations
import math
from typing import Optional, Sequence, Tuple

Pose2 = Tuple[float, float, float]


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(float(value)), math.cos(float(value)))


def tracking_error(poses: Sequence[Pose2], x: float, y: float, yaw: float) -> Tuple[Optional[float], Optional[float]]:
    """Return nearest-segment CTE and vehicle-heading error.

    Uses path-pose orientation rather than segment travel direction, so a
    Reeds-Shepp reverse segment does not look like a 180-degree heading error.
    """
    if len(poses) < 2:
        return None, None
    best_d2 = float("inf")
    best_heading = None
    for a, b in zip(poses, poses[1:]):
        dx = float(b[0] - a[0]); dy = float(b[1] - a[1])
        seg2 = dx * dx + dy * dy
        if seg2 <= 1e-12:
            continue
        t = ((x - a[0]) * dx + (y - a[1]) * dy) / seg2
        t = min(1.0, max(0.0, t))
        px = a[0] + t * dx; py = a[1] + t * dy
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_heading = a[2] + t * wrap_angle(b[2] - a[2])
    if best_heading is None:
        return None, None
    return math.sqrt(max(0.0, best_d2)), wrap_angle(yaw - best_heading)
