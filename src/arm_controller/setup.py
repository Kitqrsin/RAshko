from setuptools import setup
import os
from glob import glob

package_name = "arm_controller"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="antonia",
    maintainer_email="antonia@todo.todo",
    description="ROS2 controller for 6-DOF robotic arm",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arm = arm_controller.arm_gui:main",
            "trajectory_executor = arm_controller.trajectory_executor:main",
            "camera_node = arm_controller.camera_node:main",
            "object_detector_node = arm_controller.object_detector_node:main",
            "pick_place_node = arm_controller.pick_place_node:main",
            "camera_calibration = arm_controller.camera_calibration:main",
            "camera_calibration_export = arm_controller.camera_calibration_export:main",
            "vision_error_publisher = arm_controller.vision_error_publisher:main",
            "vision_tf_broadcaster = arm_controller.vision_tf_broadcaster:main",
            "gripper_control_service = arm_controller.gripper_control_service:main",
            "bartender_behavior_tree = arm_controller.bartender_behavior_tree:main",
            "face_track_pid = arm_controller.face_track_pid_node:main",
            "udp_servo_bridge = arm_controller.udp_servo_bridge:main",
            "servo_node = arm_controller.sim_servo_node:main",
            "simulation_smoke_test = arm_controller.simulation_smoke_test:main",
            "cartesian_simulation_test = arm_controller.cartesian_simulation_test:main",
            "cap_inspection = arm_controller.cap_inspection_node:main",
            "cap_pick_preview = arm_controller.cap_pick_preview_node:main",
            "cap_fixture_svg = arm_controller.cap_fixture_svg:main",
            "cap_sequence_simulation = arm_controller.cap_sequence_simulation:main",
        ],
    },
)
