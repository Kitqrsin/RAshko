"""Camera-only, fail-closed inspection of the marked seven-cap fixture."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_camera_url():
    configured = os.environ.get("RASHKO_CAMERA_URL", "").strip()
    if configured:
        return configured
    pi_host = os.environ.get("RASHKO_PI_HOST", "").strip()
    if not pi_host:
        return ""
    port = os.environ.get("RASHKO_CAMERA_PORT", "8080").strip() or "8080"
    return f"http://{pi_host}:{port}/stream.mjpg"


def generate_launch_description():
    start_camera = LaunchConfiguration("camera")
    start_rviz = LaunchConfiguration("rviz")
    config_file = LaunchConfiguration("config_file")
    camera_info_file = LaunchConfiguration("camera_info_file")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")

    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "config_file",
            default_value=os.environ.get(
                "RASHKO_CAP_WORKCELL_CONFIG",
                "/workspace/config/cap_workcell.example.yaml",
            ),
        ),
        DeclareLaunchArgument(
            "camera_info_file",
            default_value=os.environ.get("RASHKO_CAMERA_INFO_FILE", ""),
        ),
        DeclareLaunchArgument(
            "camera_backend",
            default_value=os.environ.get("RASHKO_CAMERA_BACKEND", "auto"),
        ),
        DeclareLaunchArgument("camera_url", default_value=_default_camera_url()),
        DeclareLaunchArgument(
            "width", default_value=os.environ.get("RASHKO_CAP_CAMERA_WIDTH", "640")
        ),
        DeclareLaunchArgument(
            "height", default_value=os.environ.get("RASHKO_CAP_CAMERA_HEIGHT", "480")
        ),
        LogInfo(
            msg=(
                "Starting perception-only cap inspection. This launch cannot "
                "publish arm motion or reset E-stop."
            )
        ),
        Node(
            package="arm_controller",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[{
                "frame_rate": 8.0,
                "width": ParameterValue(width, value_type=int),
                "height": ParameterValue(height, value_type=int),
                "camera_backend": ParameterValue(
                    LaunchConfiguration("camera_backend"), value_type=str
                ),
                "camera_url": ParameterValue(
                    LaunchConfiguration("camera_url"), value_type=str
                ),
                "camera_info_file": ParameterValue(
                    camera_info_file, value_type=str
                ),
            }],
            condition=IfCondition(start_camera),
        ),
        Node(
            package="arm_controller",
            executable="cap_inspection",
            name="cap_inspection",
            output="screen",
            parameters=[{
                "config_file": ParameterValue(config_file, value_type=str),
            }],
        ),
        Node(
            package="arm_controller",
            executable="cap_pick_preview",
            name="cap_pick_preview",
            output="screen",
            parameters=[{
                "config_file": ParameterValue(config_file, value_type=str),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="cap_inspection_rviz",
            output="screen",
            arguments=["-d", "/workspace/cap_inspection.rviz"],
            condition=IfCondition(start_rviz),
        ),
    ])
