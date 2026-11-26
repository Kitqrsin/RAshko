"""Companion window: GUI only; the active launch owns state and execution."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from arm_controller.pi_endpoint import default_camera_url


def _default_camera_url():
    return default_camera_url()


def generate_launch_description():
    use_camera = LaunchConfiguration("camera")
    camera_backend = LaunchConfiguration("camera_backend")
    camera_frame_rate = LaunchConfiguration("camera_frame_rate")
    camera_url = LaunchConfiguration("camera_url")
    camera_info_file = LaunchConfiguration("camera_info_file")
    controller_share = get_package_share_directory("arm_controller")
    gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controller_share, "launch", "arm.launch.py")),
        launch_arguments={
            "gui": "true",
            "camera": use_camera,
            "camera_preview": "true",
            "shutdown_on_gui_exit": "true",
            "camera_backend": camera_backend,
            "camera_frame_rate": camera_frame_rate,
            "camera_url": camera_url,
            "camera_info_file": camera_info_file,
            "moveit": "false",
            "bridge": "false",
            "trajectory": "false",
            "robot_state": "false",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="false"),
        DeclareLaunchArgument("camera_frame_rate", default_value="8.0"),
        DeclareLaunchArgument(
            "camera_backend",
            default_value=os.environ.get("RASHKO_CAMERA_BACKEND", "auto"),
        ),
        DeclareLaunchArgument("camera_url", default_value=_default_camera_url()),
        DeclareLaunchArgument(
            "camera_info_file",
            default_value=os.environ.get("RASHKO_CAMERA_INFO_FILE", ""),
        ),
        LogInfo(msg="Starting RAshko companion GUI; joint feedback comes from the active backend"),
        gui,
    ])
