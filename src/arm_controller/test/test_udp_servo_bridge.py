import json
import os
import sys
import tempfile
import types
from dataclasses import replace
import unittest
from pathlib import Path

try:
    from builtin_interfaces.msg import Time
    import rclpy  # noqa: F401
except ModuleNotFoundError:
    class _StubMsg:
        def __init__(self, **kwargs):
            for key, value in kwargs.items():
                setattr(self, key, value)

    class _Time(_StubMsg):
        sec = 0
        nanosec = 0

    class _Header(_StubMsg):
        def __init__(self):
            self.stamp = _Time()
            self.frame_id = ""

    class _JointState(_StubMsg):
        def __init__(self):
            self.header = _Header()
            self.name = []
            self.position = []

    class _Node:
        def __init__(self, *_args, **_kwargs):
            pass

    def _install_stub_module(name, attrs):
        if name in sys.modules:
            return sys.modules[name]
        module = types.ModuleType(name)
        for key, value in attrs.items():
            setattr(module, key, value)
        sys.modules[name] = module
        return module

    _install_stub_module("builtin_interfaces", {})
    _install_stub_module("builtin_interfaces.msg", {"Time": _Time})
    _install_stub_module("rclpy", {"ok": lambda: False})
    _install_stub_module("rclpy.node", {"Node": _Node})
    _install_stub_module("sensor_msgs", {})
    _install_stub_module("sensor_msgs.msg", {"JointState": _JointState})
    _install_stub_module("std_msgs", {})
    _install_stub_module(
        "std_msgs.msg",
        {
            "Bool": _StubMsg,
            "Float32": _StubMsg,
            "Float32MultiArray": _StubMsg,
            "String": _StubMsg,
        },
    )
    Time = _Time

from std_msgs.msg import Bool, Float32MultiArray

from arm_controller.joint_contract import HOME_DEGREES
from arm_controller.udp_servo_bridge import UdpServoBridge, ack_angles_to_joint_positions
from rashko_communicator import Ack, AckCode, ControllerState


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def debug(self, _message):
        pass

    def info(self, message):
        self.infos.append(message)

    def error(self, message):
        self.errors.append(message)

    def warning(self, message):
        self.warnings.append(message)


class _ClockInstant:
    @staticmethod
    def to_msg():
        return Time()


class _Clock:
    @staticmethod
    def now():
        return _ClockInstant()


class _Client:
    def __init__(self, acknowledgements, host="192.0.2.1", port=5005):
        self.acknowledgements = list(acknowledgements)
        self.host = host
        self.port = port
        self.stop_calls = 0
        self.status_calls = 0
        self.command_calls = 0
        self.heartbeat_calls = 0
        self.reset_calls = 0
        self.closed = False

    def receive_acks(self):
        result = self.acknowledgements
        self.acknowledgements = []
        return result

    def send_status(self):
        self.status_calls += 1
        return 400 + self.status_calls

    def send_command(self, _angles, _speed):
        self.command_calls += 1
        return 500 + self.command_calls

    def send_emergency_stop(self):
        self.stop_calls += 1
        return 100 + self.stop_calls

    def send_heartbeat(self):
        self.heartbeat_calls += 1
        return 200 + self.heartbeat_calls

    def send_reset_estop(self):
        self.reset_calls += 1
        return 300 + self.reset_calls

    def close(self):
        self.closed = True


class UdpServoBridgeTests(unittest.TestCase):
    @staticmethod
    def _ack(sequence, angles=HOME_DEGREES, code=AckCode.OK):
        return Ack(
            session_id="0123456789abcdef0123456789abcdef",
            sequence=sequence,
            code=code,
            state=(
                ControllerState.HARDWARE_READY
                | ControllerState.OUTPUTS_ENABLED
            ),
            angles_deg=tuple(angles),
            sent_at_ms=1,
        )

    @staticmethod
    def _bridge(acknowledgements):
        bridge = object.__new__(UdpServoBridge)
        bridge.client = _Client(acknowledgements)
        bridge.estop_active = False
        bridge._endpoint_change_pending = False
        bridge._motion_armed = False
        bridge._reset_requested_at = 0.0
        bridge.ack_timeout_s = 0.5
        bridge.speed_percent = 10.0
        bridge._status_sequence_offset = 0
        bridge.reset_pending = False
        bridge.pending_reset_sequence = None
        bridge.sent_anything = False
        bridge.last_ack_at = 0.0
        bridge.highest_ack_sequence = None
        bridge.last_valid_ack_angles = tuple(HOME_DEGREES)
        bridge.max_angles = (245.0, 180.0, 180.0, 180.0, 180.0, 0.0)
        bridge.pi_host = "192.0.2.1"
        bridge.pi_port = 5005
        bridge.joint_state_publisher = _Publisher()
        bridge.status_publisher = _Publisher()
        bridge.get_logger = lambda: _Logger()
        bridge.get_clock = lambda: _Clock()
        return bridge

    def test_pi_ack_home_becomes_urdf_joint_state_home(self):
        self.assertEqual(
            ack_angles_to_joint_positions(HOME_DEGREES),
            (0.0,) * 6,
        )

    def test_pi_ack_is_rejected_instead_of_clamped(self):
        with self.assertRaises(ValueError):
            ack_angles_to_joint_positions((246.0, 88.0, 90.0, 96.0, 98.0, 0.0))

    def test_non_monotonic_ack_is_discarded_before_both_publications(self):
        bridge = self._bridge(
            [
                self._ack(5),
                self._ack(4, (91.0, *HOME_DEGREES[1:])),
            ]
        )

        bridge._ack_callback()

        self.assertEqual(bridge.highest_ack_sequence, 5)
        self.assertEqual(len(bridge.joint_state_publisher.messages), 1)
        self.assertEqual(len(bridge.status_publisher.messages), 1)
        status = json.loads(bridge.status_publisher.messages[0].data)
        self.assertEqual(status["sequence"], 5)

    def test_invalid_ack_angles_publish_fault_not_healthy_status_and_stop(self):
        invalid = (246.0, *HOME_DEGREES[1:])
        bridge = self._bridge([self._ack(6, invalid)])

        bridge._ack_callback()

        self.assertTrue(bridge.estop_active)
        self.assertEqual(bridge.client.stop_calls, 1)
        self.assertEqual(bridge.joint_state_publisher.messages, [])
        self.assertEqual(len(bridge.status_publisher.messages), 1)
        status = json.loads(bridge.status_publisher.messages[0].data)
        self.assertEqual(status["code"], "HARDWARE_ERROR")
        self.assertEqual(status["angles_deg"], list(HOME_DEGREES))

    def test_ack_above_configured_bridge_limit_is_faulted(self):
        # This value is canonical but violates the default fail-closed gripper
        # calibration.  A Pi/Windows configuration mismatch must not report OK.
        angles = (*HOME_DEGREES[:5], 1.0)
        bridge = self._bridge([self._ack(7, angles)])

        bridge._ack_callback()

        self.assertTrue(bridge.estop_active)
        self.assertEqual(bridge.client.stop_calls, 1)
        self.assertEqual(bridge.joint_state_publisher.messages, [])
        status = json.loads(bridge.status_publisher.messages[0].data)
        self.assertEqual(status["code"], "HARDWARE_ERROR")

    def test_in_flight_ok_cannot_mask_locally_latched_ack_fault(self):
        invalid = (246.0, *HOME_DEGREES[1:])
        bridge = self._bridge([
            self._ack(8, invalid),
            self._ack(9),
        ])

        bridge._ack_callback()

        statuses = [
            json.loads(message.data)
            for message in bridge.status_publisher.messages
        ]
        self.assertEqual(
            [status["code"] for status in statuses],
            ["HARDWARE_ERROR", "ESTOP_PENDING"],
        )

    def test_reset_stays_pending_until_its_own_acknowledgement(self):
        bridge = self._bridge([
            self._ack(7),
            replace(self._ack(8), state=ControllerState.HARDWARE_READY),
        ])
        bridge.reset_pending = True
        bridge.pending_reset_sequence = 8

        bridge._ack_callback()

        statuses = [
            json.loads(message.data)
            for message in bridge.status_publisher.messages
        ]
        self.assertEqual([status["code"] for status in statuses], [
            "RESET_PENDING",
            "OK",
        ])
        self.assertFalse(bridge.reset_pending)
        self.assertIsNone(bridge.pending_reset_sequence)

    @staticmethod
    def _write_env_with_new_mtime(env_file: Path, content: str, prev_mtime_ns: int | None) -> int:
        env_file.write_text(content, encoding="utf-8")
        stat_ns = env_file.stat().st_mtime_ns
        if prev_mtime_ns is not None and stat_ns <= prev_mtime_ns:
            bumped = prev_mtime_ns + 1_000_000_000
            os.utime(env_file, ns=(bumped, bumped))
            stat_ns = env_file.stat().st_mtime_ns
        return stat_ns

    def test_reload_disarms_and_requires_a_new_ack_and_explicit_reset(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            env_file = Path(tmpdir) / ".env.windows"
            initial_mtime = self._write_env_with_new_mtime(
                env_file,
                "RASHKO_PI_HOST=192.0.2.1\nRASHKO_PI_PORT=5005\n",
                None,
            )

            bridge = self._bridge([])
            logger = _Logger()
            bridge.get_logger = lambda: logger
            old_client = bridge.client
            created_clients = []

            def fake_factory(host, port, *, secret="", allow_insecure=False):
                client = _Client([], host=host, port=port)
                client.secret = secret
                client.allow_insecure = allow_insecure
                created_clients.append(client)
                return client

            bridge._secret = "test-secret"
            bridge.allow_insecure = False
            bridge._client_factory = fake_factory
            bridge._env_file = env_file
            bridge._env_mtime_ns = initial_mtime
            bridge._last_env_check_at = 0.0
            bridge._follow_env_file = True
            bridge.ack_timeout_s = 1.0
            bridge.last_timeout_warning = 0.0

            bridge.estop_active = True
            bridge.reset_pending = True
            bridge.pending_reset_sequence = 42
            bridge.highest_ack_sequence = 99
            bridge.sent_anything = True
            bridge.first_sent_at = 12.0
            bridge.last_ack_at = 13.0

            self._write_env_with_new_mtime(
                env_file,
                "RASHKO_PI_HOST=198.51.100.77\nRASHKO_PI_PORT=6006\n",
                initial_mtime,
            )

            # Rate-limited before 1.0s has elapsed.
            self.assertFalse(bridge._maybe_reload_endpoint(now=0.5))
            self.assertIs(bridge.client, old_client)

            self.assertTrue(bridge._maybe_reload_endpoint(now=1.5))
            self.assertTrue(old_client.closed)
            self.assertEqual(len(created_clients), 1)
            new_client = created_clients[0]
            self.assertIs(bridge.client, new_client)
            self.assertEqual((bridge.pi_host, bridge.pi_port), ("198.51.100.77", 6006))
            self.assertEqual(new_client.secret, "test-secret")
            self.assertFalse(new_client.allow_insecure)
            self.assertIsNone(bridge.highest_ack_sequence)
            self.assertFalse(bridge.reset_pending)
            self.assertIsNone(bridge.pending_reset_sequence)
            self.assertFalse(bridge.sent_anything)
            self.assertEqual(bridge.first_sent_at, 0.0)
            self.assertEqual(bridge.last_ack_at, 0.0)
            self.assertFalse(bridge._motion_armed)
            self.assertTrue(any("198.51.100.77:6006" in msg for msg in logger.infos))

            bridge._heartbeat_callback()
            command = Float32MultiArray(data=list(HOME_DEGREES))
            bridge._joint_command_callback(command)
            bridge._estop_callback(Bool(data=False))
            self.assertEqual(new_client.command_calls, 0)
            self.assertEqual(new_client.reset_calls, 0)
            self.assertEqual(new_client.status_calls, 1)
            self.assertEqual(new_client.stop_calls, 0)
            self.assertEqual(old_client.stop_calls, 1)

            new_client.acknowledgements = [
                replace(self._ack(0), state=ControllerState.HARDWARE_READY | ControllerState.ESTOP_LATCHED)
            ]
            bridge._ack_callback()
            bridge._joint_command_callback(command)
            self.assertEqual(new_client.command_calls, 0)
            bridge._estop_callback(Bool(data=False))
            new_client.acknowledgements = [
                replace(self._ack(bridge.pending_reset_sequence), state=ControllerState.HARDWARE_READY)
            ]
            bridge._ack_callback()
            bridge._joint_command_callback(command)
            self.assertEqual(new_client.command_calls, 1)
            self.assertGreater(json.loads(bridge.status_publisher.messages[-1].data)["sequence"], 99)

    def test_maybe_reload_endpoint_keeps_client_on_error_or_unchanged_endpoint(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            env_file = Path(tmpdir) / ".env.windows"
            mtime1 = self._write_env_with_new_mtime(
                env_file,
                "RASHKO_PI_HOST=192.0.2.1\nRASHKO_PI_PORT=5005\n",
                None,
            )

            bridge = self._bridge([])
            logger = _Logger()
            bridge.get_logger = lambda: logger
            old_client = bridge.client
            factory_calls = []

            def failing_factory(host, port, **_kwargs):
                factory_calls.append((host, port))
                raise OSError("cannot bind or resolve")

            bridge._secret = "test-secret"
            bridge.allow_insecure = False
            bridge._client_factory = failing_factory
            bridge._env_file = env_file
            bridge._env_mtime_ns = mtime1
            bridge._last_env_check_at = 0.0
            bridge._follow_env_file = True
            bridge.highest_ack_sequence = 12

            # 1) Mtime changed (comment added) but (pi_host, pi_port) unchanged -> no swap.
            mtime2 = self._write_env_with_new_mtime(
                env_file,
                "# comment\nRASHKO_PI_HOST=192.0.2.1\nRASHKO_PI_PORT=5005\n",
                mtime1,
            )
            self.assertFalse(bridge._maybe_reload_endpoint(now=2.0))
            self.assertEqual(factory_calls, [])
            self.assertFalse(old_client.closed)
            self.assertIs(bridge.client, old_client)
            self.assertEqual(bridge.highest_ack_sequence, 12)

            # 2) Invalid host/port resolution error -> logs warning and keeps old client.
            mtime3 = self._write_env_with_new_mtime(
                env_file,
                "RASHKO_PI_HOST=bad host with space\nRASHKO_PI_PORT=5005\n",
                mtime2,
            )
            self.assertFalse(bridge._maybe_reload_endpoint(now=4.0))
            self.assertEqual(factory_calls, [])
            self.assertFalse(old_client.closed)
            self.assertIs(bridge.client, old_client)
            self.assertTrue(logger.warnings)

            # 3) Client factory failure -> logs warning and keeps old client.
            self._write_env_with_new_mtime(
                env_file,
                "RASHKO_PI_HOST=198.51.100.88\nRASHKO_PI_PORT=5005\n",
                mtime3,
            )
            self.assertFalse(bridge._maybe_reload_endpoint(now=6.0))
            self.assertEqual(factory_calls, [("198.51.100.88", 5005)])
            self.assertFalse(old_client.closed)
            self.assertIs(bridge.client, old_client)
            self.assertEqual((bridge.pi_host, bridge.pi_port), ("192.0.2.1", 5005))
            self.assertEqual(bridge.highest_ack_sequence, 12)
            self.assertEqual(old_client.stop_calls, 1)
            self.assertTrue(bridge._endpoint_change_pending)

            bridge._client_factory = lambda *args, **kwargs: _Client([])
            self.assertTrue(bridge._maybe_reload_endpoint(now=8.0))
            self.assertTrue(old_client.closed)
            self.assertFalse(bridge._endpoint_change_pending)
            self.assertFalse(bridge._motion_armed)

    def test_disarmed_bridge_only_requests_status(self):
        bridge = self._bridge([])
        bridge.first_sent_at = 0.0
        bridge.ack_timeout_s = 1.0
        bridge.last_timeout_warning = 0.0
        bridge._heartbeat_callback()
        self.assertEqual(bridge.client.heartbeat_calls, 0)
        self.assertEqual(bridge.client.status_calls, 1)


if __name__ == "__main__":
    unittest.main()
