#!/usr/bin/env python3
"""Launch the YDLiDAR T-mini Plus with independent safety/navigation scans.

Each physical revolution produces a minimally processed safety stream before
anti-starburst cleanup and a filtered navigation stream afterwards.  Robot-self
returns are masked independently on both outputs.
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from navigation_runtime.vehicle_geometry import load_runtime_geometry, validate_geometry
from navigation_runtime.lidar_safety_config import ensure_stage2_lidar_runtime



def _runtime_config(package_share: str, package_name: str, filename: str) -> str:
    """Return persistent runtime YAML, seeding it once from package defaults."""
    ws = (os.environ.get("AGV_ROOT") or os.environ.get("AGV_WS") or os.path.expanduser("~/forclift"))
    base = os.environ.get("AGV_RUNTIME_CONFIG_ROOT", os.path.join(ws, "config", "runtime"))
    target_dir = os.path.join(os.path.expanduser(base), package_name)
    os.makedirs(target_dir, exist_ok=True)
    target = os.path.join(target_dir, filename)
    source = os.path.join(package_share, "config", filename)
    if not os.path.exists(target) and os.path.isfile(source):
        shutil.copy2(source, target)
    return target if os.path.isfile(target) else source

def generate_launch_description():
    share = get_package_share_directory('navigation')
    ensure_stage2_lidar_runtime(share)
    params = _runtime_config(share, 'navigation', 'lidar.yaml')
    safety_params = _runtime_config(share, 'navigation', 'lidar_safety.yaml')
    _geometry_path, geometry_doc = load_runtime_geometry(share)
    geometry = validate_geometry(geometry_doc, require_validated=False)
    # Safety self-mask must never extend beyond the canonical physical footprint:
    # an oversized mask could hide a real obstacle adjacent to the chassis.
    mask_half_x = geometry['footprint_half_length_m']
    mask_half_y = geometry['footprint_half_width_m']

    args = [
        DeclareLaunchArgument('port',
            default_value='/tmp/agv_devices/lidar',
            description='Stable LiDAR alias from USB role resolver'),
        DeclareLaunchArgument('baudrate', default_value='230400'),
        DeclareLaunchArgument('frame_id', default_value='lidar_link'),
        DeclareLaunchArgument('intensity_mode', default_value='true'),
        DeclareLaunchArgument('intensity_bits', default_value='16'),
        DeclareLaunchArgument('strict_checksum', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    node = Node(
        package='navigation',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        respawn=True,
        respawn_delay=1.5,
        parameters=[params, {
            'port': LaunchConfiguration('port'),
            'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
            'frame_id': LaunchConfiguration('frame_id'),
            'intensity_mode': ParameterValue(LaunchConfiguration('intensity_mode'), value_type=bool),
            'intensity_bits': ParameterValue(LaunchConfiguration('intensity_bits'), value_type=int),
            'strict_checksum': ParameterValue(LaunchConfiguration('strict_checksum'), value_type=bool),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
        # Keep physical safety returns separate from the anti-starburst
        # navigation product. Robot-self masking is applied downstream to both.
        remappings=[
            ('scan_safety', '/scan_safety_raw'),
            ('scan', '/scan_nav_raw'),
        ],
    )

    safety_self_filter = Node(
        package='navigation',
        executable='scan_self_filter',
        name='scan_self_filter_safety',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            'input_topic': '/scan_safety_raw',
            'output_topic': '/scan_safety',
            'legacy_output_topic': '',
            'base_frame': 'base_footprint',
            'fail_open_on_tf_error': False,
            'denoise_enabled': False,
            'max_output_range': 8.0,
            'min_x': -mask_half_x,
            'max_x': mask_half_x,
            'min_y': -mask_half_y,
            'max_y': mask_half_y,
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    nav_self_filter = Node(
        package='navigation',
        executable='scan_self_filter',
        name='scan_self_filter_nav',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            'input_topic': '/scan_nav_raw',
            'output_topic': '/scan_nav',
            # One canonical production scan topic prevents tools or old launch
            # files from silently subscribing to a second, ambiguous stream.
            'legacy_output_topic': '/scan',
            'base_frame': 'base_footprint',
            # Hold navigation output until the static chassis->LiDAR transform
            # exists. Forwarding a raw first revolution caused self-returns and
            # radial noise to enter SLAM/costmaps during their initialization.
            # The driver keeps running and publication begins automatically as
            # soon as robot_state_publisher has delivered the static TF.
            'fail_open_on_tf_error': False,
            'denoise_enabled': True,
            # BAB 4.2 mapping target: pass every verified 8 Hz physical revolution.
            'output_rate_limit_hz': 8.0,
            # Do not apply a second hard-coded range cap here. The LiDAR
            # driver's lidar.yaml range_max is the single source of truth, so
            # GUI SAVE YAML changes propagate through /scan_nav immediately
            # after the isolated lidar_node respawn. Denoise/self-mask stay on.
            'max_output_range': 0.0,
            'median_radius_bins': 2,
            'min_neighbor_support': 2,
            'outlier_abs_m': 0.30,
            'outlier_rel': 0.06,
            'isolated_min_range_m': 1.0,
            'temporal_jump_m': 0.55,
            'min_x': -mask_half_x,
            'max_x': mask_half_x,
            'min_y': -mask_half_y,
            'max_y': mask_half_y,
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    lidar_safety_health = Node(
        package='navigation',
        executable='lidar_safety_health.py',
        name='lidar_safety_health',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[safety_params, {
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )
    return LaunchDescription(args + [safety_self_filter, nav_self_filter, lidar_safety_health, node])
