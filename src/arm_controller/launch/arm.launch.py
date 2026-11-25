"""Canonical RAshko launch: GUI/planning on Windows, actuation over UDP."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_camera_url():
    configured_url = os.environ.get("RASHKO_CAMERA_URL", "").strip()
    if configured_url:
        return configured_url
    pi_host = os.environ.get("RASHKO_PI_HOST", "").strip()
    if not pi_host:
        return ""
    camera_port = os.environ.get("RASHKO_CAMERA_PORT", "8080").strip() or "8080"
    return f"http://{pi_host}:{camera_port}/stream.mjpg"


def generate_launch_description():
    urdf_path = os.environ.get("RASHKO_URDF_PATH", "/workspace/robot_urdf/robot_fixed.urdf")
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    use_gui = LaunchConfiguration("gui")
    use_camera = LaunchConfiguration("camera")
    camera_preview = LaunchConfiguration("camera_preview")
    camera_backend = LaunchConfiguration("camera_backend")
    camera_frame_rate = LaunchConfiguration("camera_frame_rate")
    camera_url = LaunchConfiguration("camera_url")
    camera_info_file = LaunchConfiguration("camera_info_file")
    use_moveit = LaunchConfiguration("moveit")
    use_bridge = LaunchConfiguration("bridge")
    use_trajectory = LaunchConfiguration("trajectory")
    trajectory_speed = LaunchConfiguration("trajectory_speed")
    use_robot_state = LaunchConfiguration("robot_state")
    allow_trajectory_execution = LaunchConfiguration("allow_trajectory_execution")
    moveit_position_only_ik = LaunchConfiguration("moveit_position_only_ik")

    moveit_share = get_package_share_directory("arm_moveit_config")
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "moveit.launch.py")),
        launch_arguments={
            "start_robot_state": "false",
            "allow_trajectory_execution": allow_trajectory_execution,
            "position_only_ik": moveit_position_only_ik,
        }.items(),
        condition=IfCondition(use_moveit),
    )
    gui = Node(
        package="arm_controller",
        executable="arm",
        name="arm_gui",
        output="screen",
        parameters=[{
            "camera_enabled": ParameterValue(use_camera, value_type=bool),
            "camera_preview": ParameterValue(camera_preview, value_type=bool),
            "robot_description": robot_description,
        }],
        condition=IfCondition(use_gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("camera_preview", default_value="false"),
        DeclareLaunchArgument("shutdown_on_gui_exit", default_value="false"),
        DeclareLaunchArgument(
            "camera_backend",
            default_value=os.environ.get("RASHKO_CAMERA_BACKEND", "auto"),
        ),
        DeclareLaunchArgument("camera_url", default_value=_default_camera_url()),
        DeclareLaunchArgument("camera_frame_rate", default_value="15.0"),
        DeclareLaunchArgument(
            "camera_info_file",
            default_value=os.environ.get("RASHKO_CAMERA_INFO_FILE", ""),
        ),
        DeclareLaunchArgument("moveit", default_value="true"),
        DeclareLaunchArgument("bridge", default_value="true"),
        DeclareLaunchArgument("trajectory", default_value="true"),
        DeclareLaunchArgument("trajectory_speed", default_value="10.0"),
        DeclareLaunchArgument("robot_state", default_value="true"),
        DeclareLaunchArgument("allow_trajectory_execution", default_value="true"),
        DeclareLaunchArgument("moveit_position_only_ik", default_value="false"),
        LogInfo(msg="Starting the RAshko Windows control stack"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            condition=IfCondition(use_robot_state),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gui,
                on_exit=[EmitEvent(event=Shutdown(reason="RAshko companion GUI closed"))],
            ),
            condition=IfCondition(LaunchConfiguration("shutdown_on_gui_exit")),
        ),
        gui,
        Node(
            package="arm_controller",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[{
                "frame_rate": ParameterValue(camera_frame_rate, value_type=float),
                "camera_preview": ParameterValue(camera_preview, value_type=bool),
                "width": 640,
                "height": 480,
                "camera_backend": ParameterValue(camera_backend, value_type=str),
                "camera_url": ParameterValue(camera_url, value_type=str),
                "camera_info_file": ParameterValue(
                    camera_info_file, value_type=str
                ),
            }],
            condition=IfCondition(use_camera),
        ),
        Node(
            package="arm_controller",
            executable="udp_servo_bridge",
            name="udp_servo_bridge",
            output="screen",
            parameters=[{
                "initial_speed_percent": ParameterValue(
                    trajectory_speed, value_type=float
                ),
            }],
            condition=IfCondition(use_bridge),
        ),
        Node(
            package="arm_controller",
            executable="trajectory_executor",
            name="trajectory_executor",
            output="screen",
            parameters=[{
                "require_hardware_status": ParameterValue(
                    use_bridge, value_type=bool
                ),
                "estop_on_fault": True,
                "speed_percent": ParameterValue(
                    trajectory_speed, value_type=float
                ),
            }],
            condition=IfCondition(use_trajectory),
        ),
        moveit_launch,
    ])
