"""MoveIt launch wired to the real RAshko URDF and joint names."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_robot_state = LaunchConfiguration("start_robot_state")
    allow_trajectory_execution = LaunchConfiguration("allow_trajectory_execution")
    position_only_ik = LaunchConfiguration("position_only_ik")
    package_share = get_package_share_directory("arm_moveit_config")
    config_dir = os.path.join(package_share, "config")
    urdf_path = os.environ.get("RASHKO_URDF_PATH", "/workspace/robot_urdf/robot_fixed.urdf")

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()
    with open(os.path.join(config_dir, "robot_arm.srdf"), "r", encoding="utf-8") as srdf_file:
        robot_description_semantic = srdf_file.read()

    ompl_parameters = _load_yaml(os.path.join(config_dir, "ompl_planning.yaml"))
    kinematics_parameters = _load_yaml(
        os.path.join(config_dir, "kinematics.yaml")
    )
    kinematics_parameters["arm"]["position_only_ik"] = ParameterValue(
        position_only_ik, value_type=bool
    )
    moveit_parameters = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": kinematics_parameters,
        "robot_description_planning": _load_yaml(os.path.join(config_dir, "joint_limits.yaml")),
        "use_sim_time": use_sim_time,
        "moveit_manage_controllers": False,
        "allow_trajectory_execution": ParameterValue(
            allow_trajectory_execution, value_type=bool
        ),
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.02,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_parameters,
    }
    moveit_parameters.update(_load_yaml(os.path.join(config_dir, "controllers.yaml")))

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_robot_state", default_value="true"),
        DeclareLaunchArgument("allow_trajectory_execution", default_value="true"),
        DeclareLaunchArgument("position_only_ik", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
            condition=IfCondition(start_robot_state),
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_parameters],
        ),
    ])
