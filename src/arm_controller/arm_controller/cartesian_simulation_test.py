"""Bounded simulation regression for GUI Cartesian position/orientation control."""

from __future__ import annotations

import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

from arm_controller.joint_contract import (
    HOME_DEGREES,
    JOINT_NAMES,
    radians_to_servo_degrees,
    servo_degrees_to_radians,
)
from arm_controller.tool_pose import TOOL_LINK, tool_direction
from arm_controller.urdf_kinematics import UrdfKinematics


CASES = (
    ("up", (0.0017, -0.0262, 0.3769), 90.0),
    ("horizontal", (0.18, 0.10, 0.18), 0.0),
    ("down", (-0.0636, -0.0660, 0.0090), -90.0),
)


class CartesianSimulationTest(Node):
    def __init__(self):
        super().__init__("cartesian_simulation_test")
        urdf_path = Path(
            os.environ.get(
                "RASHKO_URDF_PATH", "/workspace/robot_urdf/robot_fixed.urdf"
            )
        )
        self.model = UrdfKinematics(urdf_path.read_text(encoding="utf-8"))
        self.command_publisher = self.create_publisher(
            Float32MultiArray, "/joint_commands", 10
        )
        self.speed_publisher = self.create_publisher(Float32, "/servo_speed", 10)
        self.latest_positions = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in JOINT_NAMES[:5]):
            self.latest_positions = tuple(
                float(positions[name]) for name in JOINT_NAMES[:5]
            )

    def run(self):
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and self.latest_positions is None:
            if time.monotonic() >= deadline:
                raise RuntimeError("timed out waiting for simulated joint states")
            rclpy.spin_once(self, timeout_sec=0.1)

        self.speed_publisher.publish(Float32(data=100.0))
        seed = self.latest_positions or servo_degrees_to_radians(HOME_DEGREES)[:5]
        results = []
        for name, target, elevation_degrees in CASES:
            solution = self.model.solve(
                target,
                math.radians(elevation_degrees),
                seed,
                wrist_roll=0.0,
            )
            if solution is None:
                raise RuntimeError(f"{name}: exact URDF IK returned no solution")

            servo_degrees = [
                radians_to_servo_degrees(index, radians)
                for index, radians in enumerate(solution.radians)
            ]
            servo_degrees.append(HOME_DEGREES[5])
            command = Float32MultiArray(data=servo_degrees)
            motion_deadline = time.monotonic() + 6.0
            while rclpy.ok():
                self.command_publisher.publish(command)
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.latest_positions is not None:
                    error = max(
                        abs(actual - desired)
                        for actual, desired in zip(
                            self.latest_positions, solution.radians
                        )
                    )
                    if error < math.radians(0.02):
                        break
                if time.monotonic() >= motion_deadline:
                    raise RuntimeError(f"{name}: simulated servos did not converge")

            predicted = self.model.forward(solution.radians)
            tf_deadline = time.monotonic() + 2.0
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                actual_position, actual_direction = self._tool_transform()
                model_error = math.dist(actual_position, predicted.position)
                direction_model_error = _angle(
                    actual_direction, predicted.tool_direction
                )
                if (
                    model_error <= 0.00025
                    and direction_model_error <= math.radians(0.25)
                ):
                    break
                if time.monotonic() >= tf_deadline:
                    raise RuntimeError(
                        f"{name}: TF disagrees with URDF FK "
                        f"({model_error * 1000.0:.2f} mm, "
                        f"{math.degrees(direction_model_error):.2f} deg)"
                    )
            results.append(
                f"{name}: target={solution.position_error * 1000.0:.1f} mm/"
                f"{math.degrees(solution.direction_error):.1f} deg, "
                f"TF={model_error * 1000.0:.2f} mm/"
                f"{math.degrees(direction_model_error):.2f} deg"
            )
            seed = solution.radians

        for result in results:
            self.get_logger().info(result)
        self.get_logger().info(
            "PASS: GUI Cartesian URDF solver and RViz TF agree for up/horizontal/down"
        )

    def _tool_transform(self):
        deadline = time.monotonic() + 2.0
        while rclpy.ok():
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link", TOOL_LINK, Time()
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                return (
                    (translation.x, translation.y, translation.z),
                    tool_direction((rotation.x, rotation.y, rotation.z, rotation.w)),
                )
            except TransformException:
                if time.monotonic() >= deadline:
                    raise RuntimeError("timed out waiting for tool_tip TF")
                rclpy.spin_once(self, timeout_sec=0.05)


def _angle(left, right):
    cosine = sum(a * b for a, b in zip(left, right))
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    return math.acos(max(-1.0, min(1.0, cosine / (left_norm * right_norm))))


def main(args=None):
    rclpy.init(args=args)
    node = CartesianSimulationTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
