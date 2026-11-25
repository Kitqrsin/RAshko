import json
import unittest

from builtin_interfaces.msg import Time

from arm_controller.joint_contract import HOME_DEGREES
from arm_controller.udp_servo_bridge import UdpServoBridge, ack_angles_to_joint_positions
from rashko_communicator import Ack, AckCode, ControllerState


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def debug(self, _message):
        pass

    def error(self, _message):
        pass

    def warning(self, _message):
        pass


class _ClockInstant:
    @staticmethod
    def to_msg():
        return Time()


class _Clock:
    @staticmethod
    def now():
        return _ClockInstant()


class _Client:
    def __init__(self, acknowledgements):
        self.acknowledgements = list(acknowledgements)
        self.stop_calls = 0

    def receive_acks(self):
        result = self.acknowledgements
        self.acknowledgements = []
        return result

    def send_emergency_stop(self):
        self.stop_calls += 1
        return 100 + self.stop_calls


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
        bridge = self._bridge([self._ack(7), self._ack(8)])
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


if __name__ == "__main__":
    unittest.main()
