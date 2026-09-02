#!/usr/bin/env python3

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from navigation_runtime.vehicle_geometry import load_runtime_geometry


# Fungsi: Menyusun LaunchDescription beserta node, parameter, kondisi, dan remapping yang diperlukan.
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    pkg_share = get_package_share_directory('navigation')
    urdf_xacro = os.path.join(pkg_share, 'urdf', 'agv.urdf.xacro')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'urdf.rviz')
    _, geometry_data = load_runtime_geometry(pkg_share)
    frame = geometry_data['frame_convention']
    vehicle = geometry_data['vehicle']

    robot_description = xacro.process_file(urdf_xacro, mappings={
        'agv_description': pkg_share,
        'cad_to_base_yaw_rad': str(frame['cad_to_base_yaw_rad']),
        'max_steering_angle_rad': str(vehicle['max_steering_angle_rad']),
    }).toxml()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),
        LogInfo(msg=(
            f"[URDF-GEOMETRY] cad_link yaw={float(frame['cad_to_base_yaw_rad']):.6f} rad; "
            f"REP-103 validated={bool(frame.get('rep103_alignment_validated', False))}; "
            "verify physical FRONT -> +X and LEFT -> +Y before autonomous use."
        )),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(gui),
            parameters=[{
                'use_sim_time': use_sim_time,
                'rate': 30,
            }],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(gui),
            parameters=[{'use_sim_time': use_sim_time}],
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
