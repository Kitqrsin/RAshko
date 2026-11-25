"""ROS 2 -> authenticated UDP bridge for the Raspberry Pi pseudo-controller."""

from __future__ import annotations

import json
import math
import os
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
from rashko_communicator import AckCode, ControllerState, ProtocolError, UdpArmClient


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    return default if value is None else value.strip().lower() in {"1", "true", "yes", "on"}


def ack_angles_to_joint_positions(angles_deg) -> tuple[float, ...]:
    """Convert a Pi applied-command estimate into canonical ROS radians."""
    return servo_degrees_to_radians(angles_deg)


class UdpServoBridge(Node):
    """Forwards the existing six-degree /joint_commands topic to the Pi."""

    def __init__(self) -> None:
        super().__init__("udp_servo_bridge")

        self.pi_host = str(
            self.declare_parameter(
                "pi_host", os.environ.get("RASHKO_PI_HOST", "192.168.1.50")
            ).value
        )
        self.pi_port = int(
            self.declare_parameter(
                "pi_port", int(os.environ.get("RASHKO_PI_PORT", "5005"))
            ).value
        )
        self.heartbeat_hz = float(self.declare_parameter("heartbeat_hz", 10.0).value)
        self.ack_timeout_s = float(self.declare_parameter("ack_timeout_s", 1.0).value)
        self.speed_percent = float(
            self.declare_parameter("initial_speed_percent", 10.0).value
        )
        if not math.isfinite(self.speed_percent) or not 1.0 <= self.speed_percent <= 100.0:
            raise ValueError("initial_speed_percent must be finite and in 1..100")
        gripper_max = float(
            os.environ.get("RASHKO_GRIPPER_CLOSED_DEGREES", "0").strip() or "0"
        )
        self.max_angles = tuple(
            float(value)
            for value in self.declare_parameter(
                "max_angles",
                [245.0, 180.0, 180.0, 180.0, 180.0, gripper_max],
            ).value
        )
        if (
            len(self.max_angles) != 6
            or any(not math.isfinite(value) or value < 0.0 for value in self.max_angles)
            or self.max_angles[5] > 180.0
        ):
            raise ValueError(
                "max_angles must contain six finite non-negative values and "
                "the gripper limit must not exceed 180 degrees"
            )
        self.allow_insecure = bool(
            self.declare_parameter("allow_insecure", _env_bool("RASHKO_ALLOW_INSECURE")).value
        )
        secret_env = str(self.declare_parameter("secret_env", "RASHKO_SHARED_SECRET").value)
        secret = os.environ.get(secret_env, "")
        if not secret and not self.allow_insecure:
            raise RuntimeError(
                f"{secret_env} is empty; refusing to send unauthenticated "
                "motor commands"
            )

        self.client = UdpArmClient(
            self.pi_host,
            self.pi_port,
            secret=secret,
            allow_insecure=self.allow_insecure,
        )
        self.estop_active = False
        self.reset_pending = False
        self.pending_reset_sequence = None
        self.sent_anything = False
        self.first_sent_at = 0.0
        self.last_ack_at = 0.0
        self.highest_ack_sequence = None
        self.last_valid_ack_angles = tuple(HOME_DEGREES)
        self.last_timeout_warning = 0.0

        self.create_subscription(
            Float32MultiArray,
            "/joint_commands",
            self._joint_command_callback,
            10,
        )
        self.create_subscription(Float32, "/servo_speed", self._speed_callback, 10)
        self.create_subscription(Bool, "/emergency_stop", self._estop_callback, 10)
        self.status_publisher = self.create_publisher(String, "/raspberry/status", 10)
        # The Pi ACK contains its rate-limited, applied-command estimate.  It is
        # the only hardware-side state available because these servos have no
        # encoders.  Publishing desired GUI targets here would make TF and
        # MoveIt jump ahead of the arm while it is still travelling.
        self.joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self.create_timer(1.0 / self.heartbeat_hz, self._heartbeat_callback)
        self.create_timer(0.05, self._ack_callback)

        mode = "INSECURE" if self.allow_insecure else "authenticated"
        self.get_logger().info(
            f"UDP servo bridge ready: {self.pi_host}:{self.pi_port}, {mode}, "
            f"heartbeat={self.heartbeat_hz:.1f} Hz, "
            f"gripper_limit={self.max_angles[5]:.1f} degrees"
        )

    def destroy_node(self) -> bool:
        self.client.close()
        return super().destroy_node()

    def _joint_command_callback(self, message: Float32MultiArray) -> None:
        angles = list(message.data)
        if len(angles) != 6:
            self.get_logger().error(
                f"Rejected joint command: expected 6 angles, got {len(angles)}"
            )
            return
        for index, angle in enumerate(angles):
            if not math.isfinite(angle) or not 0.0 <= angle <= self.max_angles[index]:
                self.get_logger().error(
                    f"Rejected joint command: J{index + 1}={angle} is outside "
                    f"0..{self.max_angles[index]} degrees"
                )
                return
        if self.estop_active or self.reset_pending:
            reason = (
                "emergency stop is active"
                if self.estop_active
                else "emergency-stop reset is awaiting acknowledgement"
            )
            self.get_logger().warning(f"Joint command ignored while {reason}")
            return
        try:
            self.client.send_command(angles, self.speed_percent)
            self.sent_anything = True
        except (OSError, ProtocolError) as exc:
            self.get_logger().error(f"Failed to send joint command: {exc}")

    def _speed_callback(self, message: Float32) -> None:
        speed = float(message.data)
        if not math.isfinite(speed) or not 1.0 <= speed <= 100.0:
            self.get_logger().error(
                f"Rejected servo speed {speed}: expected a finite value in 1..100 percent"
            )
            return
        self.speed_percent = speed

    def _estop_callback(self, message: Bool) -> None:
        self.estop_active = bool(message.data)
        self.reset_pending = not self.estop_active
        self.pending_reset_sequence = None
        try:
            if self.estop_active:
                self.client.send_emergency_stop()
                self.get_logger().warning("Emergency stop sent to Raspberry Pi")
            else:
                self.pending_reset_sequence = self.client.send_reset_estop()
                self.get_logger().warning(
                    "Emergency stop reset sent; the next joint command can "
                    "re-enable outputs after acknowledgement"
                )
            self.sent_anything = True
        except (OSError, ProtocolError) as exc:
            self.get_logger().error(f"Failed to send emergency-stop state: {exc}")

    def _heartbeat_callback(self) -> None:
        try:
            if self.estop_active:
                # E-stop is repeated because UDP delivery is not guaranteed.
                self.client.send_emergency_stop()
            elif self.reset_pending:
                sequence = self.client.send_reset_estop()
                if self.pending_reset_sequence is None:
                    self.pending_reset_sequence = sequence
            else:
                self.client.send_heartbeat()
            self.sent_anything = True
            if not self.first_sent_at:
                self.first_sent_at = time.monotonic()
        except OSError as exc:
            self.get_logger().error(f"Failed to send heartbeat: {exc}")

        now = time.monotonic()
        ack_reference = self.last_ack_at or self.first_sent_at
        if (
            self.sent_anything
            and ack_reference
            and now - ack_reference > self.ack_timeout_s
            and now - self.last_timeout_warning > 5.0
        ):
            self.last_timeout_warning = now
            self.get_logger().error(
                f"No Raspberry Pi ACK for {now - ack_reference:.1f}s; its "
                "watchdog should have disabled the servos"
            )

    def _ack_callback(self) -> None:
        try:
            acks = self.client.receive_acks()
        except (OSError, ProtocolError) as exc:
            self.get_logger().warning(f"Rejected Raspberry Pi response: {exc}")
            return
        for ack in acks:
            if (
                self.highest_ack_sequence is not None
                and ack.sequence <= self.highest_ack_sequence
            ):
                self.get_logger().debug(
                    f"Ignored non-monotonic Raspberry Pi ACK {ack.sequence}"
                )
                continue
            self.highest_ack_sequence = ack.sequence
            self.last_ack_at = time.monotonic()
            state_names = [flag.name for flag in ControllerState if flag in ack.state]
            try:
                canonical_angles = validate_servo_degrees(ack.angles_deg)
                for index, (angle, maximum) in enumerate(
                    zip(canonical_angles, self.max_angles)
                ):
                    if angle > maximum:
                        raise ValueError(
                            f"{JOINT_NAMES[index]}={angle:.3f} exceeds the "
                            f"configured bridge limit {maximum:.3f} degrees"
                        )
                positions = ack_angles_to_joint_positions(canonical_angles)
            except ValueError as exc:
                # Do not let a mismatched Pi calibration silently corrupt TF or
                # masquerade as a healthy acknowledgement.  Hold the last valid
                # estimate for diagnostic status only and physically stop.
                self.get_logger().error(
                    f"Rejected Raspberry Pi joint-state estimate: {exc}"
                )
                self.estop_active = True
                self.reset_pending = False
                self.pending_reset_sequence = None
                try:
                    self.client.send_emergency_stop()
                    self.sent_anything = True
                except (OSError, ProtocolError) as stop_exc:
                    self.get_logger().error(
                        f"Failed to send emergency stop after invalid ACK: {stop_exc}"
                    )
                self._publish_status(
                    ack.sequence,
                    AckCode.HARDWARE_ERROR.name,
                    state_names,
                    self.last_valid_ack_angles,
                )
                continue

            self.last_valid_ack_angles = canonical_angles
            status_code = ack.code.name
            if self.reset_pending:
                reset_confirmed = (
                    self.pending_reset_sequence is not None
                    and ack.sequence >= self.pending_reset_sequence
                    and ack.code is AckCode.OK
                    and ControllerState.ESTOP_LATCHED not in ack.state
                )
                if reset_confirmed:
                    self.reset_pending = False
                    self.pending_reset_sequence = None
                elif ack.code is AckCode.OK:
                    # An older in-flight OK must not make the ROS safety layer
                    # believe that RESET_ESTOP has already been acknowledged.
                    status_code = "RESET_PENDING"
            elif (
                self.estop_active
                and ack.code is AckCode.OK
                and ControllerState.ESTOP_LATCHED not in ack.state
            ):
                # An ACK already in flight when a local safety fault latched
                # E-stop must not replace HARDWARE_ERROR with a healthy status
                # before the Pi confirms that outputs are disabled.
                status_code = "ESTOP_PENDING"

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = list(JOINT_NAMES)
            joint_state.position = list(positions)
            self.joint_state_publisher.publish(joint_state)
            self._publish_status(
                ack.sequence,
                status_code,
                state_names,
                canonical_angles,
            )
            if ack.code not in {AckCode.OK, AckCode.STALE_SEQUENCE} and not (
                ack.code is AckCode.ESTOP_LATCHED and self.estop_active
            ):
                self.get_logger().error(f"Raspberry Pi rejected command: {ack.code.name}")

    def _publish_status(self, sequence, code, state_names, angles_deg) -> None:
        status = {
            "host": self.pi_host,
            "port": self.pi_port,
            "sequence": sequence,
            "code": code,
            "state": list(state_names),
            "angles_deg": [round(value, 3) for value in angles_deg],
        }
        message = String()
        message.data = json.dumps(status, separators=(",", ":"))
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = UdpServoBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
