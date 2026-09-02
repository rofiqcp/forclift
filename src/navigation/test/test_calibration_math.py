import math
from navigation_runtime.calibration_math import (
    estimate_imu_stationary, fit_steering_calibration,
    calibrate_wheel_radius, lidar_safety_baseline,
)


def test_imu_stationary_bias_residual_added_to_existing():
    rows=[]
    for i in range(300):
        n=(i%7-3)*1e-4
        rows.append(dict(gx=0.01+n, gy=-0.02-n, gz=0.005+n,
                         ax=0.03+n, ay=-0.04-n, az=9.80665+0.08+n))
    out=estimate_imu_stationary(rows,[0.10,0.20,0.30],[0.4,0.5,0.6],min_samples=200)
    assert out['quality_ok']
    assert abs(out['gyro_bias_rps'][0]-0.11)<1e-3
    assert abs(out['gyro_bias_rps'][1]-0.18)<1e-3
    assert abs(out['accel_bias_mps2'][2]-0.68)<1e-3


def test_steering_fit_recovers_negative_encoder_direction():
    intercept=8123.0; slope=-1250.0
    pts=[(0.0,intercept),(0.30,intercept+slope*0.30),(-0.28,intercept+slope*(-0.28))]
    out=fit_steering_calibration(pts)
    assert out['quality_ok']
    assert abs(out['steering_zero_ticks']-intercept)<1e-6
    assert abs(out['steering_ticks_per_rad']-1250.0)<1e-6
    assert out['steering_sign']==-1.0


def test_wheel_radius_scales_from_physical_distance():
    out=calibrate_wheel_radius(0.145,2.0,(0,0,0),(1.8,0.01,0.01))
    assert out['quality_ok']
    assert abs(out['wheel_radius_m']-(0.145*2.0/math.hypot(1.8,0.01)))<1e-9


def test_lidar_baseline_never_loosens_threshold():
    rows=[{'valid_beams':100+i%5,'valid_ratio':0.40+(i%4)*0.01} for i in range(100)]
    out=lidar_safety_baseline(rows,current_min_valid_beams=30,current_min_valid_ratio=0.20)
    assert out['quality_ok']
    assert out['suggested_min_valid_beams']>=30
    assert out['suggested_min_valid_ratio']>=0.20
