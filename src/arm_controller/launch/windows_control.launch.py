"""Start the Windows-side execution and Raspberry Pi transport nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="arm_controller",
            executable="udp_servo_bridge",
            name="udp_servo_bridge",
            output="screen",
            parameters=[{"initial_speed_percent": 10.0}],
        ),
        Node(
            package="arm_controller",
            executable="trajectory_executor",
            name="trajectory_executor",
            output="screen",
            parameters=[{
                "require_hardware_status": True,
                "estop_on_fault": True,
                "speed_percent": 10.0,
            }],
        ),
    ])
