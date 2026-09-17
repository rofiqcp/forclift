"""Independent operational Build Map SLAM runtime.

This runtime never uses BAB IV map slots.  It can reuse an already-running
LiDAR publisher or, when requested, own the LiDAR + robot TF itself.  All SLAM
outputs stay under /build_map/*.
"""
import os
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from navigation_runtime.vehicle_geometry import load_runtime_geometry, validate_geometry


def generate_launch_description():
    nav_share = get_package_share_directory('navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    _geometry_path, geometry_doc = load_runtime_geometry(nav_share)
    geometry = validate_geometry(geometry_doc, require_validated=False)
    half_x = geometry['footprint_half_length_m']
    half_y = geometry['footprint_half_width_m']
    workspace = Path(os.environ.get('AGV_ROOT') or os.environ.get('AGV_WS') or (Path.home() / 'forclift'))
    hector_params = str(workspace / 'config/runtime/navigation/hector_build_map.yaml')

    robot_description = xacro.process_file(
        os.path.join(nav_share, 'urdf', 'agv.urdf.xacro'),
        mappings={
            'cad_to_base_yaw_rad': str(geometry['cad_to_base_yaw_rad']),
            'max_steering_angle_rad': str(geometry['max_steering_angle_rad']),
        }).toxml()

    robot_state = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='build_map_robot_state_publisher', output='screen',
        condition=IfCondition(start_lidar),
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'publish_frequency': 30.0,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'lidar.launch.py')),
        condition=IfCondition(start_lidar),
        launch_arguments={
            'port': '/tmp/agv_devices/lidar',
            'baudrate': '230400',
            'frame_id': 'lidar_link',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    scan_filter = Node(
        package='navigation', executable='scan_self_filter',
        name='build_map_scan_self_filter', output='screen',
        parameters=[{
            'input_topic': '/scan_nav_raw',
            'output_topic': '/build_map/scan_filtered',
            'legacy_output_topic': '',
            'base_frame': 'base_footprint',
            'fail_open_on_tf_error': False,
            'denoise_enabled': False,
            'output_rate_limit_hz': 0.0,
            'max_output_range': 0.0,
            'min_x': -half_x, 'max_x': half_x,
            'min_y': -half_y, 'max_y': half_y,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    hector = Node(
        package='navigation', executable='hector_slam_node',
        name='build_map_hector_slam_node', output='screen',
        parameters=[hector_params, {
            'scan_topic': '/build_map/scan_filtered',
            'map_width': 40.0, 'map_height': 40.0, 'map_resolution': 0.05,
            # Decisive first-hit log-odds only for operational Build Map.
            # Defaults 0.15/0.35 keep first endpoints below the 65% occupied
            # threshold, making obstacles look free in the GUI/PGM.
            'update_factor_free': 1.20,
            'update_factor_occupied': 1.20,
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
            'coarse_search_x': 0.10, 'coarse_search_y': 0.08,
            'coarse_search_theta_deg': 4.0,
            'max_single_trans': 0.14, 'max_single_rot_deg': 6.0,
            'max_vel_trans': 0.90, 'max_vel_rot': 1.00,
            'deviation_threshold': 0.12, 'deviation_threshold_init': 0.18,
            'delta_ema_alpha': 0.45,
            'map_update_distance_threshold': 0.008,
            'map_update_angle_threshold': 0.006,
            'use_imu_rotation_gate': False,
            'publish_map_topic': True, 'publish_pose_topic': True,
            'publish_odom_topic': False,
            'publish_map_odom_tf': False, 'publish_odom_tf': False,
            'robot_initial_x': 0.0, 'robot_initial_y': 0.0, 'robot_initial_yaw': 0.0,
            'map_pub_period': 0.25, 'pose_pub_period': 0.05, 'tf_pub_period': 0.05,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        remappings=[
            ('/map', '/build_map/map'), ('/pose', '/build_map/pose'),
            ('/odom', '/build_map/lidar_pose'), ('/trajectory', '/build_map/trajectory'),
            ('/robot_pose_text', '/build_map/robot_pose_text'),
            ('/scan_match_quality', '/build_map/scan_match_quality'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='false'),
        robot_state, lidar, scan_filter, hector,
    ])
