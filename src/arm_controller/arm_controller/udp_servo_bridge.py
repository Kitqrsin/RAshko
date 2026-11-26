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
from arm_controller.pi_endpoint import default_env_file, load_dotenv_file, resolve_pi_endpoint, validate_pi_host
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

        endpoint = resolve_pi_endpoint()
        default_host = str(endpoint["pi_host"])
        default_port = int(endpoint["pi_port"])
        self.pi_host = str(self.declare_parameter("pi_host", default_host).value)
        self.pi_port = int(self.declare_parameter("pi_port", default_port).value)
        self.heartbeat_hz = float(self.declare_parameter("heartbeat_hz", 10.0).value)
        self.ack_timeout_s = float(self.declare_parameter("ack_timeout_s", 0.5).value)
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

        self._secret = secret
        self._client_factory = UdpArmClient
        self._env_file = default_env_file()
        self._env_mtime_ns = self._stat_env_mtime_ns()
        self._last_env_check_at = 0.0
        self._endpoint_change_pending = False
        self._follow_env_file = (
            self.pi_host == default_host and self.pi_port == default_port
        )
        self.client = self._client_factory(
            self.pi_host,
            self.pi_port,
            secret=self._secret,
            allow_insecure=self.allow_insecure,
        )
        self.estop_active = False
        self._motion_armed = False
        self._reset_requested_at = 0.0
        self.reset_pending = False
        self.pending_reset_sequence = None
        self.sent_anything = False
        self.first_sent_at = 0.0
        self.last_ack_at = 0.0
        self.highest_ack_sequence = None
        self._status_sequence_offset = 0
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

    def _stat_env_mtime_ns(self) -> int | None:
        env_file = getattr(self, "_env_file", None)
        if env_file is None:
            return None
        try:
            return env_file.stat().st_mtime_ns
        except OSError:
            return None

    def _maybe_reload_endpoint(self, now: float | None = None, *, force: bool = False) -> bool:
        if (
            not getattr(self, "_follow_env_file", False)
            or getattr(self, "_env_file", None) is None
        ):
            return False
        if now is None:
            now = time.monotonic()
        if not force and now - self._last_env_check_at < 1.0:
            return False
        self._last_env_check_at = now

        mtime_ns = self._stat_env_mtime_ns()
        if mtime_ns == getattr(self, "_env_mtime_ns", None):
            return False
        try:
            values = load_dotenv_file(self._env_file)
            override = os.environ.get("RASHKO_PI_HOST_OVERRIDE", "").strip()
            if override.lower() in {"", "auto"}:
                validate_pi_host(values.get("RASHKO_PI_HOST", ""))
            endpoint = resolve_pi_endpoint(env_file=self._env_file, file_values=values)
            new_host = str(endpoint["pi_host"])
            new_port = int(endpoint["pi_port"])
        except (OSError, ValueError) as exc:
            self._begin_endpoint_change()
            self.get_logger().warning(f"Raspberry Pi target unresolved: {exc}")
            return False

        if (new_host, new_port) == (self.pi_host, self.pi_port):
            self._env_mtime_ns = mtime_ns
            self._endpoint_change_pending = False
            return False

        self._begin_endpoint_change()
        try:
            new_client = self._client_factory(
                new_host, new_port, secret=self._secret, allow_insecure=self.allow_insecure
            )
        except (OSError, ValueError) as exc:
            self.get_logger().warning(f"Could not connect to {new_host}:{new_port}: {exc}")
            return False

        self.client.close()
        self.client = new_client
        self.pi_host, self.pi_port = new_host, new_port
        self._env_mtime_ns = mtime_ns
        self._endpoint_change_pending = False
        # The stop sent to the former Pi must not spill over to an unverified target.
        self.estop_active = False
        # ROS consumers need monotonic status across UDP sessions.
        previous_sequence = self.highest_ack_sequence
        self._status_sequence_offset += 1 if previous_sequence is None else previous_sequence + 2
        self.highest_ack_sequence = None
        self.sent_anything = False
        self.first_sent_at = 0.0
        self.last_ack_at = 0.0
        self.last_valid_ack_angles = tuple(HOME_DEGREES)
        self._publish_status(-1, "TARGET_CHANGED", [], self.last_valid_ack_angles)
        self.get_logger().info(
            f"Selected Raspberry Pi {self.pi_host}:{self.pi_port}. A fresh ACK and reset are required."
        )
        return True

    def _begin_endpoint_change(self) -> None:
        if not self._endpoint_change_pending:
            self._endpoint_change_pending = True
            self._stop_motion()

    def _stop_motion(self) -> None:
        self.estop_active = True
        self._motion_armed = False
        self.reset_pending = False
        self.pending_reset_sequence = None
        try:
            self.client.send_emergency_stop()
        except (OSError, ProtocolError) as exc:
            self.get_logger().error(f"Failed to send Raspberry Pi E-stop: {exc}")

    def _link_is_fresh(self, now: float) -> bool:
        return bool(self.last_ack_at and now - self.last_ack_at <= self.ack_timeout_s)

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
        if not self._motion_armed or not self._link_is_fresh(time.monotonic()):
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
        if message.data:
            self._stop_motion()
            return

        now = time.monotonic()
        self._maybe_reload_endpoint(now, force=True)
        if self._endpoint_change_pending or not self._link_is_fresh(now):
            self.get_logger().warning("Reset refused until the selected Raspberry Pi acknowledges")
            return

        self._motion_armed = False
        try:
            self.pending_reset_sequence = self.client.send_reset_estop()
        except (OSError, ProtocolError) as exc:
            self.get_logger().error(f"Failed to send E-stop reset: {exc}")
            self._stop_motion()
            return
        self.estop_active = False
        self.reset_pending = True
        self._reset_requested_at = now

    def _heartbeat_callback(self) -> None:
        now = time.monotonic()
        self._maybe_reload_endpoint(now)
        if (
            (self._motion_armed and not self._link_is_fresh(now))
            or (self.reset_pending and now - self._reset_requested_at > self.ack_timeout_s)
        ):
            self._stop_motion()
            self.get_logger().warning("Raspberry Pi ACK timed out. An explicit reset is required.")
        try:
            if self.reset_pending:
                return
            if self.estop_active:
                self.client.send_emergency_stop()
            elif self._motion_armed:
                self.client.send_heartbeat()
            else:
                self.client.send_status()
            self.sent_anything = True
            if not self.first_sent_at:
                self.first_sent_at = now
        except (OSError, ProtocolError) as exc:
            self.get_logger().error(f"Failed to send Raspberry Pi packet: {exc}")

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
                "watchdog state is unconfirmed"
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
                self._stop_motion()
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
                status_code = "RESET_PENDING"
                if ack.sequence == self.pending_reset_sequence:
                    self.reset_pending = False
                    self.pending_reset_sequence = None
                    self._motion_armed = (
                        ack.code is AckCode.OK
                        and ControllerState.HARDWARE_READY in ack.state
                        and ControllerState.ESTOP_LATCHED not in ack.state
                        and ControllerState.OUTPUTS_ENABLED not in ack.state
                    )
                    status_code = "OK" if self._motion_armed else "RESET_REQUIRED"
            elif (
                ack.code in {AckCode.HARDWARE_ERROR, AckCode.CONTROL_BUSY}
                or ControllerState.ESTOP_LATCHED in ack.state
                or ControllerState.HARDWARE_READY not in ack.state
            ):
                self._motion_armed = False

            if self._endpoint_change_pending:
                status_code = "TARGET_UNRESOLVED"
            elif status_code == "OK" and not self._motion_armed:
                status_code = "ESTOP_PENDING" if self.estop_active and (
                    ControllerState.ESTOP_LATCHED not in ack.state
                    or ControllerState.OUTPUTS_ENABLED in ack.state
                ) else "RESET_REQUIRED"

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
            "sequence": sequence + self._status_sequence_offset,
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
