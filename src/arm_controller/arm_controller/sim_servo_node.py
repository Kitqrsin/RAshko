"""Hardware-free servo node that mirrors the Raspberry Pi motion contract."""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from arm_controller.joint_contract import (
    HOME_DEGREES,
    JOINT_NAMES,
    servo_degrees_to_radians,
    validate_servo_degrees,
)


class SimServoNode(Node):
    """Rate-limited servo model with the same topics and safety rules as the Pi."""

    def __init__(self) -> None:
        super().__init__("servo_node")
        self.rate_hz = float(self.declare_parameter("control_rate_hz", 50.0).value)
        self.min_speed_deg_s = float(self.declare_parameter("min_speed_deg_s", 5.0).value)
        self.max_speed_deg_s = float(self.declare_parameter("max_speed_deg_s", 180.0).value)
        if self.rate_hz <= 0.0 or self.min_speed_deg_s <= 0.0:
            raise ValueError("simulation rates must be positive")
        if self.max_speed_deg_s < self.min_speed_deg_s:
            raise ValueError("max_speed_deg_s must be >= min_speed_deg_s")

        self.current_degrees = list(HOME_DEGREES)
        self.target_degrees = list(HOME_DEGREES)
        self.speed_percent = float(
            self.declare_parameter("initial_speed_percent", 10.0).value
        )
        if not math.isfinite(self.speed_percent) or not 1.0 <= self.speed_percent <= 100.0:
            raise ValueError("initial_speed_percent must be finite and in 1..100")
        self.estop_latched = False
        self.command_count = 0
        self._previous_tick = time.monotonic()

        self.joint_state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.status_publisher = self.create_publisher(String, "/simulation/servo_status", 10)
        self.create_subscription(Float32MultiArray, "/joint_commands", self._command_callback, 10)
        self.create_subscription(Float32, "/servo_speed", self._speed_callback, 10)
        self.create_subscription(Bool, "/emergency_stop", self._estop_callback, 10)
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            "Simulation servo ready: six logical-degree inputs, rate-limited joint-state feedback"
        )

    def _command_callback(self, message: Float32MultiArray) -> None:
        try:
            angles = validate_servo_degrees(message.data)
        except ValueError as exc:
            self.get_logger().error(f"Rejected simulated servo command: {exc}")
            return
        if self.estop_latched:
            self.get_logger().warning("Ignored servo command while emergency stop is latched")
            return
        self.target_degrees = list(angles)
        self.command_count += 1

    def _speed_callback(self, message: Float32) -> None:
        if not math.isfinite(message.data):
            self.get_logger().error("Rejected non-finite servo speed")
            return
        self.speed_percent = max(1.0, min(100.0, float(message.data)))

    def _estop_callback(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested:
            self.estop_latched = True
            self.target_degrees = list(self.current_degrees)
            self.get_logger().warning("Simulation emergency stop latched; motion frozen")
        elif self.estop_latched:
            self.estop_latched = False
            self.target_degrees = list(self.current_degrees)
            self.get_logger().warning("Simulation emergency stop reset; waiting for a new command")

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(now - self._previous_tick, 3.0 / self.rate_hz)
        self._previous_tick = now
        if not self.estop_latched:
            rate = self.min_speed_deg_s + (
                (self.max_speed_deg_s - self.min_speed_deg_s) * self.speed_percent / 100.0
            )
            max_step = rate * dt
            for index, (current, target) in enumerate(zip(self.current_degrees, self.target_degrees)):
                delta = target - current
                if abs(delta) <= max_step:
                    self.current_degrees[index] = target
                else:
                    self.current_degrees[index] = current + math.copysign(max_step, delta)
        self._publish_joint_state()

    def _publish_joint_state(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(servo_degrees_to_radians(self.current_degrees))
        self.joint_state_publisher.publish(message)

    def _publish_status(self) -> None:
        status = String()
        status.data = json.dumps(
            {
                "mode": "simulation",
                "estop_latched": self.estop_latched,
                "speed_percent": round(self.speed_percent, 2),
                "commands_accepted": self.command_count,
                "current_degrees": [round(value, 3) for value in self.current_degrees],
                "target_degrees": [round(value, 3) for value in self.target_degrees],
            },
            separators=(",", ":"),
        )
        self.status_publisher.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
