"""Standalone electric-winch ROS 2 bridge."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import shutil



def _runtime_winch():
    ws=os.environ.get("AGV_WS","/home/otomasi2/ros")
    root=os.environ.get("AGV_RUNTIME_CONFIG_ROOT",os.path.join(ws,"config","runtime"))
    target_dir=os.path.join(os.path.expanduser(root),"esc"); os.makedirs(target_dir,exist_ok=True)
    target=os.path.join(target_dir,"winch.yaml")
    source=os.path.join(get_package_share_directory("esc"),"config","winch.yaml")
    if not os.path.exists(target) and os.path.isfile(source): shutil.copy2(source,target)
    return target

def generate_launch_description():
    params = _runtime_winch()
    return LaunchDescription([
        DeclareLaunchArgument("winch_port", default_value="auto"),
        Node(
            package="esc",
            executable="winch_serial_node",
            name="winch_serial_node",
            output="screen",
            parameters=[params, {"port": LaunchConfiguration("winch_port")}],
        ),
    ])
