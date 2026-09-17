#!/usr/bin/env python3
"""BAB 4.2 isolated LiDAR-only SLAM runtime.

Pipeline:
  /scan_nav_raw -> robot self-mask -> session gate -> Hector scan matching
  -> /mapping/map + /mapping/pose
No wheel odometry, /lidar/odom, EKF odometry, or IMU is consumed by this runtime.
"""
from launch import LaunchDescription
import os
from pathlib import Path
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from navigation_runtime.vehicle_geometry import load_runtime_geometry, validate_geometry
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_share = get_package_share_directory('navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    _geometry_path, geometry_doc = load_runtime_geometry(nav_share)
    geometry = validate_geometry(geometry_doc, require_validated=False)
    half_x = geometry['footprint_half_length_m']
    half_y = geometry['footprint_half_width_m']
    workspace = Path(os.environ.get('AGV_ROOT') or os.environ.get('AGV_WS') or (Path.home() / 'forclift'))
    hector_params = str(workspace / 'config/runtime/navigation/hector_autonomous.yaml')

    mapping_filter = Node(
        package='navigation',
        executable='scan_self_filter',
        name='mapping_scan_self_filter',
        output='screen',
        parameters=[{
            'input_topic': '/scan_nav_raw',
            'output_topic': '/mapping/scan_filtered',
            'legacy_output_topic': '',
            'base_frame': 'base_footprint',
            'fail_open_on_tf_error': False,
            'denoise_enabled': False,
            'output_rate_limit_hz': 0.0,
            'max_output_range': 0.0,
            'min_x': -half_x,
            'max_x': half_x,
            'min_y': -half_y,
            'max_y': half_y,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    gate = Node(
        package='navigation',
        executable='mapping_lidar_odom_bridge.py',
        name='mapping_lidar_odom_bridge',
        output='screen',
        parameters=[{
            'source_scan_topic': '/mapping/scan_filtered',
            'output_scan_topic': '/mapping/scan_nav',
            'session_control_enabled': ParameterValue(LaunchConfiguration('session_control_enabled'), value_type=bool),
            'initial_session_enabled': ParameterValue(LaunchConfiguration('initial_session_enabled'), value_type=bool),
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    hector = Node(
        package='navigation',
        executable='hector_slam_node',
        name='mapping_hector_slam_node',
        output='screen',
        parameters=[hector_params, {
            'scan_topic': '/mapping/scan_nav',
            # 40x40 m at 0.05 m = 800x800 cells; sufficient for BAB 4.2
            # while keeping live web-map processing responsive.
            'map_width': 40.0,
            'map_height': 40.0,
            'map_resolution': 0.05,
            # BAB 4.2 is LiDAR-only.  Tune motion detection for the slow AGV
            # and bias ambiguous scan matches toward physically small motion.
            'motion_min_overlap': 24,
            'motion_confirm_threshold': 1,
            'motion_median_threshold': 0.010,
            'motion_changed_range_threshold': 0.035,
            'motion_changed_ratio_threshold': 0.12,
            'motion_reference_max_scans': 12,
            'stationary_confirm_threshold': 10,
            'deadband_dist': 0.003,
            'deadband_angle_deg': 0.45,
            'search_tie_epsilon': 0.002,
            'coarse_search_x': 0.10,
            'coarse_search_y': 0.08,
            'coarse_search_theta_deg': 4.0,
            'max_single_trans': 0.14,
            'max_single_rot_deg': 6.0,
            'max_vel_trans': 0.90,
            'max_vel_rot': 1.00,
            'deviation_threshold': 0.12,
            'deviation_threshold_init': 0.18,
            'delta_ema_alpha': 0.45,
            'map_update_distance_threshold': 0.008,
            'map_update_angle_threshold': 0.006,
            'use_imu_rotation_gate': False,
            'publish_map_topic': True,
            'publish_pose_topic': True,
            'publish_odom_topic': False,
            'publish_map_odom_tf': False,
            'publish_odom_tf': False,
            'robot_initial_x': 0.0,
            'robot_initial_y': 0.0,
            'robot_initial_yaw': 0.0,
            'map_pub_period': 0.25,
            'pose_pub_period': 0.05,
            'tf_pub_period': 0.05,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        remappings=[
            ('/map', '/mapping/map'),
            ('/pose', '/mapping/pose'),
            ('/odom', '/mapping/lidar_pose'),
            ('/trajectory', '/mapping/trajectory'),
            ('/robot_pose_text', '/mapping/robot_pose_text'),
            ('/scan_match_quality', '/mapping/scan_match_quality'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('session_control_enabled', default_value='true'),
        DeclareLaunchArgument('initial_session_enabled', default_value='false'),
        # Kept only for compatibility with the existing controller command.
        DeclareLaunchArgument('slam_params_file', default_value=hector_params),
        DeclareLaunchArgument('transform_publish_period', default_value='0.05'),
        mapping_filter,
        gate,
        hector,
    ])
