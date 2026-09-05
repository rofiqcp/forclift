"""Astra YOLOP GPU 1280x720 dengan viewer OpenCV ringkas.

Node hanya mencoba CUDA/OpenGL bila build OpenCV menyatakan dukungan OpenGL.
Pada build GTK tanpa OpenGL, viewer langsung memakai pinned-host HighGUI tanpa
warning exception dan tanpa mengubah jalur TensorRT/CUDA utama.
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
    window_width = LaunchConfiguration("window_width")
    window_height = LaunchConfiguration("window_height")
    fullscreen = LaunchConfiguration("fullscreen")
    use_cuda_opengl = LaunchConfiguration("use_cuda_opengl")
    allow_fps_resolution_fallback = LaunchConfiguration(
        "allow_fps_resolution_fallback"
    )
    strict_camera_mode = LaunchConfiguration("strict_camera_mode")
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
                "v4l2_pixel_format": "YUYV",
                "allow_mjpeg_cpu_fallback": False,
                "use_v4l2_userptr_zero_copy": True,
                "v4l2_buffer_count": 3,
                "allow_resolution_fallback_for_fps": ParameterValue(
                    allow_fps_resolution_fallback, value_type=bool
                ),
                "strict_camera_mode": ParameterValue(
                    strict_camera_mode, value_type=bool
                ),
                "show_opencv_cuda_window": True,
                "opencv_use_cuda_opengl": ParameterValue(
                    use_cuda_opengl, value_type=bool
                ),
                "opencv_window_width": ParameterValue(window_width, value_type=int),
                "opencv_window_height": ParameterValue(
                    window_height, value_type=int
                ),
                "opencv_window_fullscreen": ParameterValue(
                    fullscreen, value_type=bool
                ),
                "publish_annotated": False,
                "publish_raw_rgb": False,
                "publish_masks": False,
                "publish_class_mask": False,
                "publish_detections": True,
                "publish_only_when_subscribed": True,
                "draw_text_labels_cpu": False,
                "async_rviz_publish": False,
                "publisher_reliability": "reliable",
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
            DeclareLaunchArgument("window_width", default_value="640"),
            DeclareLaunchArgument("window_height", default_value="480"),
            DeclareLaunchArgument("fullscreen", default_value="false"),
            DeclareLaunchArgument("use_cuda_opengl", default_value="true"),
            DeclareLaunchArgument(
                "allow_fps_resolution_fallback", default_value="false"
            ),
            DeclareLaunchArgument("strict_camera_mode", default_value="true"),
            DeclareLaunchArgument("respawn_node", default_value="false"),
            SetEnvironmentVariable("CUDA_MODULE_LOADING", "LAZY"),
            SetEnvironmentVariable("__GL_SYNC_TO_VBLANK", "0"),
            SetEnvironmentVariable("vblank_mode", "0"),
            node,
        ]
    )
