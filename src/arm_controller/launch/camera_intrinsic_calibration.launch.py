"""Camera-only monocular intrinsic calibration; no arm nodes are started."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_camera_url():
    configured = os.environ.get("RASHKO_CAMERA_URL", "").strip()
    if configured:
        return configured
    host = os.environ.get("RASHKO_PI_HOST", "").strip()
    if not host:
        return ""
    port = os.environ.get("RASHKO_CAMERA_PORT", "8080").strip() or "8080"
    return f"http://{host}:{port}/stream.mjpg"


def generate_launch_description():
    """Start only the MJPEG camera publisher and ROS calibration GUI."""

    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    return LaunchDescription(
        [
            DeclareLaunchArgument("width", default_value="640"),
            DeclareLaunchArgument("height", default_value="480"),
            DeclareLaunchArgument(
                "camera_backend",
                default_value=os.environ.get("RASHKO_CAMERA_BACKEND", "auto"),
            ),
            DeclareLaunchArgument("camera_url", default_value=_default_camera_url()),
            LogInfo(
                msg=(
                    "Starting CAMERA-ONLY intrinsic calibration. No arm, "
                    "trajectory, UDP bridge, or E-stop reset node is started."
                )
            ),
            Node(
                package="arm_controller",
                executable="camera_node",
                name="camera_calibration_source",
                output="screen",
                parameters=[
                    {
                        "frame_rate": 8.0,
                        "width": ParameterValue(width, value_type=int),
                        "height": ParameterValue(height, value_type=int),
                        "camera_backend": ParameterValue(
                            LaunchConfiguration("camera_backend"),
                            value_type=str,
                        ),
                        "camera_url": ParameterValue(
                            LaunchConfiguration("camera_url"),
                            value_type=str,
                        ),
                        "camera_info_file": "",
                    }
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "camera_calibration",
                    "cameracalibrator",
                    "--size",
                    "8x6",
                    "--square",
                    "0.020",
                    "--no-service-check",
                    "--camera_name",
                    "rashko_gripper_camera",
                    "--ros-args",
                    "--remap",
                    "image:=/camera/image_raw",
                    "--remap",
                    "camera:=/camera",
                ],
                output="screen",
            ),
        ]
    )
