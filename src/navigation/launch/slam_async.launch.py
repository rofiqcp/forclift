#!/usr/bin/env python3
"""Master-aligned asynchronous SLAM Toolbox lifecycle launcher.

This mirrors the supplied master behavior: async_slam_toolbox_node is
configured and activated explicitly, using navigation/config/slam_toolbox.yaml.
The async node processes incoming /scan callbacks directly instead of building
the synchronous processing queue, which keeps mapping responsive at the 10 Hz
LiDAR input rate while /map publication remains controlled by map_update_interval.
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition



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
    nav_share = get_package_share_directory('navigation')

    autostart = LaunchConfiguration('autostart')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_topic = LaunchConfiguration('map_topic')
    transform_publish_period = LaunchConfiguration('transform_publish_period')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    scan_topic = LaunchConfiguration('scan_topic')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Configure and activate SLAM Toolbox automatically.')
    declare_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Use an external lifecycle manager instead of local autostart.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=_runtime_config(nav_share, 'navigation', 'slam_toolbox.yaml'))
    declare_map_topic = DeclareLaunchArgument(
        'map_topic', default_value='/map',
        description='OccupancyGrid output topic; default preserves standalone mapping behavior.')
    declare_transform_publish_period = DeclareLaunchArgument(
        'transform_publish_period', default_value='0.05',
        description='SLAM map->odom TF publish period; set 0.0 for isolated BAB 4.2 mapping.')
    declare_odom_frame = DeclareLaunchArgument('odom_frame', default_value='odom')
    declare_base_frame = DeclareLaunchArgument('base_frame', default_value='base_footprint')
    declare_scan_topic = DeclareLaunchArgument('scan_topic', default_value='/scan_nav')

    params = ParameterFile(slam_params_file, allow_substs=True)

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[params, {
            'use_sim_time': use_sim_time,
            'transform_publish_period': ParameterValue(transform_publish_period, value_type=float),
            'odom_frame': ParameterValue(odom_frame, value_type=str),
            'base_frame': ParameterValue(base_frame, value_type=str),
            'scan_topic': ParameterValue(scan_topic, value_type=str),
        }],
        remappings=[('map', map_topic)],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart))

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[SLAM] async_slam_toolbox_node configured; activating'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ))],
        ),
        condition=IfCondition(autostart))

    return LaunchDescription([
        declare_autostart,
        declare_lifecycle_manager,
        declare_use_sim_time,
        declare_params,
        declare_map_topic,
        declare_transform_publish_period,
        declare_odom_frame,
        declare_base_frame,
        declare_scan_topic,
        slam_node,
        configure,
        activate,
    ])
