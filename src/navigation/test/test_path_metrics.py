import math
from navigation_runtime.path_metrics import tracking_error


def test_reverse_reeds_shepp_uses_vehicle_orientation_not_travel_direction():
    # Vehicle points +X while path coordinates travel from +X toward 0 (reverse).
    poses=[(2.0,0.0,0.0),(1.0,0.0,0.0),(0.0,0.0,0.0)]
    cte, heading=tracking_error(poses,1.4,0.10,0.02)
    assert abs(cte-0.10)<1e-9
    assert abs(heading-0.02)<1e-9


def test_heading_interpolation_wraps_pi_boundary():
    poses=[(0.0,0.0,math.radians(179)),(1.0,0.0,math.radians(-179))]
    _cte, heading=tracking_error(poses,0.5,0.0,math.pi)
    assert abs(heading)<math.radians(2)
