#!/usr/bin/env python3
"""V43 persistent RViz + native C++ Mapping Control GUI."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('navigation')
    enable_rviz = LaunchConfiguration('enable_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    args = [
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(nav_share, 'rviz', 'mapping.rviz')),
    ]
    gui = Node(
        package='navigation', executable='mapping_gui_cpp',
        name='mapping_control_gui_cpp', output='screen', emulate_tty=True)
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2_mapping',
        output='screen', condition=IfCondition(enable_rviz), arguments=['-d', rviz_config])
    return LaunchDescription(args + [LogInfo(msg='[MAPPING-GUI-CPP] Native C++ mapping GUI starting'), gui, rviz])
