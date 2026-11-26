from __future__ import annotations

import socket
import threading
import unittest
from dataclasses import replace
from pathlib import Path

from rashko_communicator.config import ControllerConfig, default_hardware_config
from rashko_communicator.client import UdpArmClient
from rashko_communicator.controller import ArmController, UdpControllerServer
from rashko_communicator.hardware import DryRunHardware, HardwareError
from rashko_communicator.protocol import (
    AckCode,
    ControllerState,
    MessageType,
    Request,
    decode_ack,
    encode_request,
)


SECRET = "b" * 64
SESSION = "fedcba9876543210fedcba9876543210"
WALL_MS = 1_750_000_000_000


def make_config(*, allowed_clients: frozenset[str] = frozenset()) -> ControllerConfig:
    hardware = default_hardware_config()
    return ControllerConfig(
        bind_host="127.0.0.1",
        port=5005,
        shared_secret=SECRET,
        allow_insecure=False,
        allowed_clients=allowed_clients,
        watchdog_ms=500,
        freshness_ms=5000,
        update_hz=50.0,
        max_sessions=8,
        hardware_backend="dry-run",
        servo_config_path=Path("unused.json"),
        hardware=hardware,
    )


def request(
    kind: MessageType,
    sequence: int,
    *,
    angles: tuple[object, ...] | None = None,
    speed: object = None,
) -> Request:
    return Request(
        kind=kind,
        session_id=SESSION,
        sequence=sequence,
        sent_at_ms=WALL_MS,
        angles_deg=angles,
        speed_percent=speed,
    )


class FailingWriteHardware(DryRunHardware):
    def write_angles(self, angles_deg) -> None:  # type: ignore[no-untyped-def]
        raise HardwareError("injected write failure")


class ControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = make_config()
        self.hardware = DryRunHardware(self.config.hardware)
        self.controller = ArmController(self.config, self.hardware)

    def test_startup_reset_and_heartbeat_cannot_move_or_enable(self) -> None:
        self.assertFalse(self.hardware.outputs_enabled)
        self.assertEqual(self.hardware.write_count, 0)
        self.assertTrue(self.controller.state & ControllerState.ESTOP_LATCHED)
        self.assertTrue(self.controller.state & ControllerState.WATCHDOG_TRIPPED)

        rejected = self.controller.process(
            request(
                MessageType.COMMAND,
                0,
                angles=(100, 90, 90, 90, 90, 90),
                speed=10,
            ),
            "192.168.1.10",
            now_monotonic=1.0,
            now_wall_ms=WALL_MS,
        )
        self.assertEqual(rejected.code, AckCode.ESTOP_LATCHED)

        reset = self.controller.process(
            request(MessageType.RESET_ESTOP, 1),
            "192.168.1.10",
            now_monotonic=1.1,
            now_wall_ms=WALL_MS,
        )
        self.assertEqual(reset.code, AckCode.OK)
        self.assertFalse(self.hardware.outputs_enabled)
        self.assertEqual(self.hardware.write_count, 0)

        heartbeat = self.controller.process(
            request(MessageType.HEARTBEAT, 2),
            "192.168.1.10",
            now_monotonic=1.2,
            now_wall_ms=WALL_MS,
        )
        self.assertEqual(heartbeat.code, AckCode.OK)
        self.assertFalse(self.hardware.outputs_enabled)
        self.assertEqual(self.hardware.write_count, 0)

    def test_fresh_command_enables_then_moves_non_blocking_at_speed(self) -> None:
        target = (100, 80, 100, 80, 100, 80)
        self.controller.process(
            request(MessageType.RESET_ESTOP, 0),
            "192.168.1.10",
            now_monotonic=2.0,
            now_wall_ms=WALL_MS,
        )
        ack = self.controller.process(
            request(
                MessageType.COMMAND,
                1,
                angles=target,
                speed=10,
            ),
            "192.168.1.10",
            now_monotonic=2.1,
            now_wall_ms=WALL_MS,
        )
        self.assertEqual(ack.code, AckCode.OK)
        self.assertTrue(self.hardware.outputs_enabled)
        self.assertEqual(self.hardware.write_count, 1)
        self.assertEqual(
            self.hardware.angles_deg,
            self.config.hardware.home_degrees,
        )

        self.controller.tick(2.2)
        # 180 deg/s * 10% * 0.1s = 1.8 degree largest-joint step.
        largest_delta = max(
            abs(target_angle - home_angle)
            for target_angle, home_angle in zip(
                target,
                self.config.hardware.home_degrees,
            )
        )
        fraction = 1.8 / largest_delta
        self.assertAlmostEqual(
            self.controller.current_angles[0],
            self.config.hardware.home_degrees[0]
            + (target[0] - self.config.hardware.home_degrees[0]) * fraction,
        )
        self.assertAlmostEqual(
            self.controller.current_angles[1],
            self.config.hardware.home_degrees[1]
            + (target[1] - self.config.hardware.home_degrees[1]) * fraction,
        )
        self.assertEqual(self.hardware.write_count, 2)

    def test_watchdog_disables_and_relatches_estop(self) -> None:
        self.controller.process(
            request(MessageType.RESET_ESTOP, 0),
            "192.168.1.10",
            now_monotonic=10.0,
            now_wall_ms=WALL_MS,
        )
        self.controller.process(
            request(
                MessageType.COMMAND,
                1,
                angles=(100, 90, 90, 90, 90, 90),
                speed=25,
            ),
            "192.168.1.10",
            now_monotonic=10.1,
            now_wall_ms=WALL_MS,
        )
        self.controller.tick(10.7)
        self.assertFalse(self.hardware.outputs_enabled)
        self.assertTrue(self.controller.state & ControllerState.WATCHDOG_TRIPPED)
        self.assertTrue(self.controller.state & ControllerState.ESTOP_LATCHED)

        denied = self.controller.process(
            request(
                MessageType.COMMAND,
                2,
                angles=(100, 90, 90, 90, 90, 90),
                speed=25,
            ),
            "192.168.1.10",
            now_monotonic=10.8,
            now_wall_ms=WALL_MS,
        )
        self.assertEqual(denied.code, AckCode.ESTOP_LATCHED)

    def test_status_and_foreign_session_cannot_keep_motion_alive_or_take_over(self) -> None:
        ip = "192.168.1.10"
        reset = request(MessageType.RESET_ESTOP, 0)
        command = request(MessageType.COMMAND, 1, angles=(100, 90, 90, 90, 90, 90), speed=25)
        self.controller.process(reset, ip, now_monotonic=10.0)
        other_session = "a" * 32
        foreign_command = replace(command, session_id=other_session)
        denied = self.controller.process(foreign_command, ip, now_monotonic=10.05)
        self.assertEqual(denied.code, AckCode.CONTROL_BUSY)
        self.assertFalse(self.hardware.outputs_enabled)
        accepted = self.controller.process(command, ip, now_monotonic=10.1)
        self.assertEqual(accepted.code, AckCode.OK)

        heartbeat = request(MessageType.HEARTBEAT, 2)
        self.controller.process(heartbeat, ip, now_monotonic=10.3)
        foreign_heartbeat = replace(heartbeat, session_id=other_session)
        self.controller.process(foreign_heartbeat, ip, now_monotonic=10.6)
        self.controller.process(request(MessageType.STATUS, 3), ip, now_monotonic=10.7)
        self.assertEqual(self.controller.last_contact_at, 10.3)
        self.controller.process(request(MessageType.HEARTBEAT, 4), ip, now_monotonic=10.9)
        self.assertFalse(self.hardware.outputs_enabled)
        self.assertTrue(self.controller.estop_latched)

    def test_replay_is_rejected_without_refreshing_watchdog(self) -> None:
        first = self.controller.process(
            request(MessageType.HEARTBEAT, 5),
            "192.168.1.10",
            now_monotonic=20.0,
            now_wall_ms=WALL_MS,
        )
        replay = self.controller.process(
            request(MessageType.HEARTBEAT, 5),
            "192.168.1.10",
            now_monotonic=20.4,
            now_wall_ms=WALL_MS,
        )
        self.assertEqual(first.code, AckCode.OK)
        self.assertEqual(replay.code, AckCode.STALE_SEQUENCE)
        self.assertIsNone(self.controller.last_contact_at)

    def test_session_is_bound_to_first_authenticated_client_ip(self) -> None:
        first = self.controller.process(
            request(MessageType.HEARTBEAT, 0),
            "192.168.1.10",
            now_monotonic=20.0,
            now_wall_ms=WALL_MS,
        )
        wrong_source = self.controller.process(
            request(MessageType.EMERGENCY_STOP, 1),
            "192.168.1.11",
            now_monotonic=20.1,
            now_wall_ms=WALL_MS,
        )
        self.assertEqual(first.code, AckCode.OK)
        self.assertEqual(wrong_source.code, AckCode.STALE_SEQUENCE)
        self.assertIn("another client", wrong_source.detail)
        self.assertIsNone(self.controller.last_contact_at)

    def test_invalid_command_is_rejected_without_clamping(self) -> None:
        self.controller.process(
            request(MessageType.RESET_ESTOP, 0),
            "192.168.1.10",
            now_monotonic=1.0,
            now_wall_ms=WALL_MS,
        )
        ack = self.controller.process(
            request(
                MessageType.COMMAND,
                1,
                angles=(246, 90, 90, 90, 90, 90),
                speed=50,
            ),
            "192.168.1.10",
            now_monotonic=1.1,
            now_wall_ms=WALL_MS,
        )
        self.assertEqual(ack.code, AckCode.INVALID_COMMAND)
        self.assertFalse(self.hardware.outputs_enabled)
        self.assertEqual(self.hardware.write_count, 0)

    def test_narrower_calibrated_range_is_checked_before_enable(self) -> None:
        servos = list(self.config.hardware.servos)
        servos[0] = replace(servos[0], logical_max_deg=120.0)
        hardware_config = replace(self.config.hardware, servos=tuple(servos))
        config = replace(self.config, hardware=hardware_config)
        hardware = DryRunHardware(hardware_config)
        controller = ArmController(config, hardware)
        controller.process(
            request(MessageType.RESET_ESTOP, 0),
            "192.168.1.10",
            now_monotonic=1.0,
            now_wall_ms=WALL_MS,
        )
        ack = controller.process(
            request(
                MessageType.COMMAND,
                1,
                angles=(121, 90, 90, 90, 90, 90),
                speed=20,
            ),
            "192.168.1.10",
            now_monotonic=1.1,
            now_wall_ms=WALL_MS,
        )
        self.assertEqual(ack.code, AckCode.INVALID_COMMAND)
        self.assertFalse(hardware.outputs_enabled)
        self.assertEqual(hardware.write_count, 0)

    def test_hardware_failure_fails_closed_and_clears_ready(self) -> None:
        failing = FailingWriteHardware(self.config.hardware)
        controller = ArmController(self.config, failing)
        controller.process(
            request(MessageType.RESET_ESTOP, 0),
            "192.168.1.10",
            now_monotonic=1.0,
            now_wall_ms=WALL_MS,
        )
        ack = controller.process(
            request(
                MessageType.COMMAND,
                1,
                angles=(100, 90, 90, 90, 90, 90),
                speed=10,
            ),
            "192.168.1.10",
            now_monotonic=1.1,
            now_wall_ms=WALL_MS,
        )
        self.assertEqual(ack.code, AckCode.HARDWARE_ERROR)
        self.assertFalse(failing.outputs_enabled)
        self.assertFalse(ack.state & ControllerState.HARDWARE_READY)
        self.assertTrue(ack.state & ControllerState.ESTOP_LATCHED)

    def test_server_rejects_non_allowlisted_client_before_parsing(self) -> None:
        config = make_config(allowed_clients=frozenset({"192.168.1.10"}))
        controller = ArmController(config, DryRunHardware(config.hardware))

        class FakeSocket:
            def setblocking(self, _blocking: bool) -> None:
                pass

            def close(self) -> None:
                pass

        server = UdpControllerServer(config, controller, FakeSocket())  # type: ignore[arg-type]
        self.assertIsNone(
            server.handle_datagram(b"not even JSON", ("192.168.1.11", 12345))
        )

    def test_server_allows_cidr_subnet_clients_and_discards_outside(self) -> None:
        config = make_config(
            allowed_clients=frozenset({"192.168.100.0/24", "169.254.0.0/16"})
        )
        controller = ArmController(config, DryRunHardware(config.hardware))

        class FakeSocket:
            def setblocking(self, _blocking: bool) -> None:
                pass

            def close(self) -> None:
                pass

        server = UdpControllerServer(config, controller, FakeSocket())  # type: ignore[arg-type]

        # Outside subnets and malformed client IPs are discarded before parsing.
        for rejected_ip in ("192.168.101.15", "10.0.0.1", "not-an-ip", "::1"):
            self.assertIsNone(
                server.handle_datagram(b"not even JSON", (rejected_ip, 12345))
            )

        datagram_24 = encode_request(
            MessageType.HEARTBEAT,
            SESSION,
            1,
            secret=SECRET,
            sent_at_ms=WALL_MS,
        )
        outside_signed = server.handle_datagram(
            datagram_24,
            ("192.168.101.15", 12345),
            now_wall_ms=WALL_MS,
            now_monotonic=1.0,
        )
        self.assertIsNone(outside_signed)

        # Client inside /24 subnet is accepted and receives a signed ACK.
        response_24 = server.handle_datagram(
            datagram_24,
            ("192.168.100.42", 12345),
            now_wall_ms=WALL_MS,
            now_monotonic=1.0,
        )
        self.assertIsNotNone(response_24)
        ack_24 = decode_ack(response_24, secret=SECRET, now_ms=WALL_MS)  # type: ignore[arg-type]
        self.assertEqual(ack_24.code, AckCode.OK)
        self.assertEqual(ack_24.sequence, 1)

        # Session IP binding still rejects a different IP even inside the same /24 subnet.
        datagram_hijack = encode_request(
            MessageType.HEARTBEAT,
            SESSION,
            2,
            secret=SECRET,
            sent_at_ms=WALL_MS,
        )
        hijack_response = server.handle_datagram(
            datagram_hijack,
            ("192.168.100.99", 12345),
            now_wall_ms=WALL_MS,
            now_monotonic=1.1,
        )
        self.assertIsNotNone(hijack_response)
        hijack_ack = decode_ack(hijack_response, secret=SECRET, now_ms=WALL_MS)  # type: ignore[arg-type]
        self.assertEqual(hijack_ack.code, AckCode.STALE_SEQUENCE)
        self.assertIn("another client", hijack_ack.detail)

        # Client inside /16 link-local subnet is accepted on its own session.
        other_session = "0123456789abcdef0123456789abcdef"
        datagram_16 = encode_request(
            MessageType.HEARTBEAT,
            other_session,
            1,
            secret=SECRET,
            sent_at_ms=WALL_MS,
        )
        response_16 = server.handle_datagram(
            datagram_16,
            ("169.254.77.88", 12345),
            now_wall_ms=WALL_MS,
            now_monotonic=1.2,
        )
        self.assertIsNotNone(response_16)
        ack_16 = decode_ack(response_16, secret=SECRET, now_ms=WALL_MS)  # type: ignore[arg-type]
        self.assertEqual(ack_16.code, AckCode.OK)
        self.assertEqual(ack_16.sequence, 1)

    def test_server_authenticates_and_returns_signed_ack(self) -> None:
        class FakeSocket:
            def setblocking(self, _blocking: bool) -> None:
                pass

            def close(self) -> None:
                pass

        server = UdpControllerServer(self.config, self.controller, FakeSocket())  # type: ignore[arg-type]
        datagram = encode_request(
            MessageType.HEARTBEAT,
            SESSION,
            4,
            secret=SECRET,
            sent_at_ms=WALL_MS,
        )
        response = server.handle_datagram(
            datagram,
            ("192.168.1.10", 12345),
            now_wall_ms=WALL_MS,
            now_monotonic=1.0,
        )
        self.assertIsNotNone(response)
        ack = decode_ack(response, secret=SECRET, now_ms=WALL_MS)  # type: ignore[arg-type]
        self.assertEqual(ack.code, AckCode.OK)
        self.assertEqual(ack.sequence, 4)

    def test_real_udp_client_server_round_trip(self) -> None:
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind(("127.0.0.1", 0))
        port = udp_socket.getsockname()[1]
        server = UdpControllerServer(self.config, self.controller, udp_socket)
        stop_event = threading.Event()
        thread = threading.Thread(
            target=server.serve_forever,
            args=(stop_event,),
            daemon=True,
        )
        thread.start()
        try:
            with UdpArmClient("127.0.0.1", port, secret=SECRET) as client:
                sequence = client.send_heartbeat()
                ack = client.wait_for_ack(sequence, timeout_s=1.0)
                self.assertEqual(ack.code, AckCode.OK)
                self.assertTrue(ack.state & ControllerState.ESTOP_LATCHED)
        finally:
            stop_event.set()
            thread.join(timeout=1.0)
            server.close()
        self.assertFalse(thread.is_alive())


if __name__ == "__main__":
    unittest.main()
