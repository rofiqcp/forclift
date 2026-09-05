"""Node Astra YOLOP GPU untuk dipanggil langsung atau dari package bringup.

Launch ini tidak membuka OpenCV atau RViz, tetapi secara default tetap
mempublikasikan /camera/yolop/image_annotated agar RViz utama milik bringup
langsung dapat menampilkannya.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE_NAME = "perception"


# Fungsi: Menyusun LaunchDescription beserta node, parameter, kondisi, dan remapping yang diperlukan.
def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory(PACKAGE_NAME)

    config_file = LaunchConfiguration("config_file")
    engine_path = LaunchConfiguration("engine_path")
    rgb_device = LaunchConfiguration("rgb_device")
    rgb_width = LaunchConfiguration("rgb_width")
    rgb_height = LaunchConfiguration("rgb_height")
    camera_fps = LaunchConfiguration("camera_fps")
    gpu_device = LaunchConfiguration("gpu_device")
    publish_annotated = LaunchConfiguration("publish_annotated")
    publish_masks = LaunchConfiguration("publish_masks")
    publish_detections = LaunchConfiguration("publish_detections")
    publish_class_mask = LaunchConfiguration("publish_class_mask")
    allow_fps_resolution_fallback = LaunchConfiguration(
        "allow_fps_resolution_fallback"
    )
    strict_camera_mode = LaunchConfiguration("strict_camera_mode")
    v4l2_pixel_format = LaunchConfiguration("v4l2_pixel_format")
    allow_mjpeg_cpu_fallback = LaunchConfiguration("allow_mjpeg_cpu_fallback")
    use_v4l2_userptr_zero_copy = LaunchConfiguration("use_v4l2_userptr_zero_copy")
    respawn_node = LaunchConfiguration("respawn_node")

    node = Node(
        package=PACKAGE_NAME,
        executable="perception_node",
        name="perception",
        output="screen",
        emulate_tty=True,
        respawn=PythonExpression(["'", respawn_node, "' == 'true'"]),
        respawn_delay=5.0,
        parameters=[
            config_file,
            {
                "engine_path": engine_path,
                "rgb_device": rgb_device,
                "rgb_width": ParameterValue(rgb_width, value_type=int),
                "rgb_height": ParameterValue(rgb_height, value_type=int),
                "fps": ParameterValue(camera_fps, value_type=int),
                "gpu_device": ParameterValue(gpu_device, value_type=int),
                "v4l2_pixel_format": v4l2_pixel_format,
                "allow_mjpeg_cpu_fallback": ParameterValue(
                    allow_mjpeg_cpu_fallback, value_type=bool
                ),
                "use_v4l2_userptr_zero_copy": ParameterValue(
                    use_v4l2_userptr_zero_copy, value_type=bool
                ),
                "v4l2_buffer_count": 3,
                "allow_resolution_fallback_for_fps": ParameterValue(
                    allow_fps_resolution_fallback, value_type=bool
                ),
                "strict_camera_mode": ParameterValue(
                    strict_camera_mode, value_type=bool
                ),
                "publish_annotated": ParameterValue(
                    publish_annotated, value_type=bool
                ),
                "publish_raw_rgb": False,
                "publish_masks": ParameterValue(publish_masks, value_type=bool),
                "publish_class_mask": ParameterValue(
                    publish_class_mask, value_type=bool
                ),
                "publish_detections": ParameterValue(
                    publish_detections, value_type=bool
                ),
                "publish_only_when_subscribed": True,
                "show_opencv_cuda_window": False,
                "draw_text_labels_cpu": False,
                "async_rviz_publish": ParameterValue(
                    publish_annotated, value_type=bool
                ),
                "rviz_publish_buffer_count": 2,
                # Reliable publisher kompatibel dengan subscriber Reliable
                # maupun Best Effort. Thread publisher tetap latest-frame-only.
                "publisher_reliability": "reliable",
                "annotated_topic": "/camera/yolop/image_annotated",
                "detections_topic": "/yolop/detections",
                "drivable_mask_topic": "/yolop/drivable_mask",
                "lane_mask_topic": "/yolop/lane_mask",
                "class_mask_topic": "/yolop/class_mask",
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(share, "config", "astra_yolop_gpu.yaml"),
            ),
            DeclareLaunchArgument(
                "engine_path",
                default_value="/home/sirobo/ros/models/yolopv2.engine",
            ),
            DeclareLaunchArgument("rgb_device", default_value="auto"),
            DeclareLaunchArgument("rgb_width", default_value="1280"),
            DeclareLaunchArgument("rgb_height", default_value="720"),
            DeclareLaunchArgument("camera_fps", default_value="30"),
            DeclareLaunchArgument("gpu_device", default_value="0"),
            DeclareLaunchArgument("publish_annotated", default_value="true"),
            DeclareLaunchArgument("publish_masks", default_value="false"),
            DeclareLaunchArgument("publish_detections", default_value="true"),
            DeclareLaunchArgument("publish_class_mask", default_value="false"),
            DeclareLaunchArgument(
                "allow_fps_resolution_fallback", default_value="false"
            ),
            DeclareLaunchArgument("strict_camera_mode", default_value="true"),
            DeclareLaunchArgument("v4l2_pixel_format", default_value="MJPEG"),
            DeclareLaunchArgument("allow_mjpeg_cpu_fallback", default_value="true"),
            DeclareLaunchArgument("use_v4l2_userptr_zero_copy", default_value="false"),
            DeclareLaunchArgument("respawn_node", default_value="false"),
            SetEnvironmentVariable("CUDA_MODULE_LOADING", "LAZY"),
            node,
        ]
    )
