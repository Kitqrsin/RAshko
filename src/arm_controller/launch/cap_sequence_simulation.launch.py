"""Dedicated offline seven-cap sequence simulation and visualization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    EmitEvent,
    GroupAction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    controller_share = get_package_share_directory("arm_controller")
    config_file = LaunchConfiguration("config_file")
    show_rviz = LaunchConfiguration("rviz")
    exit_on_complete = LaunchConfiguration("exit_on_complete")
    rviz_config = LaunchConfiguration("rviz_config")

    offline_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "rviz": "false",
            "smoke_test": "false",
            "trajectory_speed": "100.0",
            "moveit_position_only_ik": "true",
        }.items(),
    )
    sequencer = Node(
        package="arm_controller",
        executable="cap_sequence_simulation",
        name="cap_sequence_simulation",
        output="screen",
        parameters=[
            {
                "config_file": ParameterValue(config_file, value_type=str),
                "exit_on_complete": ParameterValue(
                    exit_on_complete, value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="/workspace/config/cap_sequence.simulation.yaml",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("exit_on_complete", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="/workspace/cap_sequence_simulation.rviz",
            ),
            LogInfo(
                msg=(
                    "Starting the dedicated seven-cap OFFLINE simulation. "
                    "Camera and UDP bridge are disabled; no Raspberry Pi or "
                    "hardware traffic is possible from this launch."
                )
            ),
            # Keep the included backend's ``rviz:=false`` launch argument in
            # its own scope.  Without this group, it leaks into this parent
            # launch and disables the dedicated cap-sequence RViz node too.
            GroupAction(actions=[offline_backend], scoped=True),
            Node(
                package="rviz2",
                executable="rviz2",
                name="cap_sequence_simulation_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(show_rviz),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    LogInfo(
                        msg=(
                            "Starting the cap sequencer after the offline "
                            "MoveIt/controller startup delay"
                        )
                    ),
                    sequencer,
                ],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=sequencer,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="cap sequence simulation finished"
                            )
                        )
                    ],
                ),
                condition=IfCondition(exit_on_complete),
            ),
        ]
    )
