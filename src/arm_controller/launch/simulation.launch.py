"""Offline RAshko backend: model, simulated servos, MoveIt, and optional RViz/test."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    show_rviz = LaunchConfiguration("rviz")
    run_smoke_test = LaunchConfiguration("smoke_test")
    trajectory_speed = LaunchConfiguration("trajectory_speed")
    moveit_position_only_ik = LaunchConfiguration("moveit_position_only_ik")
    allow_trajectory_execution = PythonExpression(
        ["'", run_smoke_test, "'.lower() != 'true'"]
    )
    urdf_path = os.environ.get("RASHKO_URDF_PATH", "/workspace/robot_urdf/robot_fixed.urdf")
    rviz_path = os.environ.get("RASHKO_RVIZ_CONFIG", "/workspace/robot.rviz")
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    controller_share = get_package_share_directory("arm_controller")
    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controller_share, "launch", "arm.launch.py")),
        launch_arguments={
            "gui": "false",
            "camera": "false",
            "moveit": "true",
            "bridge": "false",
            "trajectory": "true",
            "trajectory_speed": trajectory_speed,
            "robot_state": "false",
            "allow_trajectory_execution": allow_trajectory_execution,
            "moveit_position_only_ik": moveit_position_only_ik,
        }.items(),
    )
    smoke_test = Node(
        package="arm_controller",
        executable="simulation_smoke_test",
        name="simulation_smoke_test",
        output="screen",
        condition=IfCondition(run_smoke_test),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("smoke_test", default_value="false"),
        DeclareLaunchArgument("trajectory_speed", default_value="10.0"),
        DeclareLaunchArgument("moveit_position_only_ik", default_value="false"),
        LogInfo(msg="Starting RAshko offline simulation; no Raspberry Pi traffic will be sent"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        ),
        Node(
            package="arm_controller",
            executable="servo_node",
            name="servo_node",
            output="screen",
        ),
        backend,
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_path],
            condition=IfCondition(show_rviz),
        ),
        smoke_test,
        RegisterEventHandler(
            OnProcessExit(
                target_action=smoke_test,
                on_exit=[EmitEvent(event=Shutdown(reason="simulation smoke test finished"))],
            ),
            condition=IfCondition(run_smoke_test),
        ),
    ])
