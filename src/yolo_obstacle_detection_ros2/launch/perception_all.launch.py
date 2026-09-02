#!/usr/bin/env python3
"""Complete RGB perception launch: camera + YOLO + hole-block alignment."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    yolo_share = get_package_share_directory('yolo_obstacle_detection_ros2')
    default_model = '/home/otomasi2/ros/models/yolov8n_agv_forklift.onnx'
    camera_params = os.path.join(yolo_share, 'config', 'camera_v4l2.yaml')
    yolo_params = os.path.join(yolo_share, 'config', 'yolo_detection.yaml')
    alignment_params = os.path.join(
        yolo_share, 'hole_block_alignment', 'alignment_realtime.yaml')

    args = [
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('engine', default_value=''),
        DeclareLaunchArgument('use_tensorrt', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_yolo', default_value='true'),
        DeclareLaunchArgument('enable_hole_alignment', default_value='true'),
        DeclareLaunchArgument('enable_pallet_pose', default_value='false',
                              description='Compatibility argument; RGB-only V4L2 has no depth source'),
        DeclareLaunchArgument('camera_device', default_value='auto'),
        DeclareLaunchArgument('camera_width', default_value='1280'),
        DeclareLaunchArgument('camera_height', default_value='720'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('camera_pixel_format', default_value='MJPG'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]

    camera_node = Node(
        package='yolo_obstacle_detection_ros2',
        executable='astra_rgb_v4l2_node',
        name='astra_rgb_v4l2_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_camera')),
        parameters=[camera_params, {
            'device': LaunchConfiguration('camera_device'),
            'width': ParameterValue(LaunchConfiguration('camera_width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('camera_height'), value_type=int),
            'fps': ParameterValue(LaunchConfiguration('camera_fps'), value_type=int),
            'pixel_format': LaunchConfiguration('camera_pixel_format'),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    obstacle_node = Node(
        package='yolo_obstacle_detection_ros2',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_yolo')),
        parameters=[yolo_params, {
            'model_path': LaunchConfiguration('model'),
            'engine_path': LaunchConfiguration('engine'),
            'use_tensorrt': ParameterValue(LaunchConfiguration('use_tensorrt'), value_type=bool),
            'input_topic': '/camera/color/image_raw',
            'output_topic': '/obstacle_detection/obstacles',
            'visualization_topic': '/obstacle_detection/visualization',
            'max_inference_fps': 30.0,
            'max_visualization_fps': 20.0,
        }],
    )

    alignment_node = Node(
        package='yolo_obstacle_detection_ros2',
        executable='hole_block_alignment_node.py',
        name='hole_block_alignment_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_hole_alignment')),
        parameters=[{
            'config': alignment_params,
            'image_topic': '/camera/color/image_raw',
            'obstacle_topic': '/obstacle_detection/obstacles',
            'state_topic': '/fork_alignment/state',
            'output_image_topic': '/fork_alignment/image',
            'save_csv': False,
            'save_output_video': False,
            'visualization_fps': 20.0,
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )

    return LaunchDescription(args + [camera_node, obstacle_node, alignment_node])
