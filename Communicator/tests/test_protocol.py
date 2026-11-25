from __future__ import annotations

import json
import unittest

from rashko_communicator.protocol import (
    Ack,
    AckCode,
    ControllerState,
    MessageType,
    ProtocolError,
    decode_ack,
    decode_request,
    encode_ack,
    encode_request,
    validate_joint_command,
)


SECRET = "a" * 64
SESSION = "0123456789abcdef0123456789abcdef"
NOW_MS = 1_750_000_000_000


class ProtocolTests(unittest.TestCase):
    def test_authenticated_command_round_trip(self) -> None:
        encoded = encode_request(
            MessageType.COMMAND,
            SESSION,
            7,
            secret=SECRET,
            sent_at_ms=NOW_MS,
            angles_deg=[105, 80, 110, 70, 100, 95],
            speed_percent=25,
        )
        request = decode_request(
            encoded,
            secret=SECRET,
            now_ms=NOW_MS + 100,
        )
        self.assertEqual(request.kind, MessageType.COMMAND)
        self.assertEqual(request.sequence, 7)
        self.assertEqual(request.angles_deg, (105, 80, 110, 70, 100, 95))
        self.assertEqual(request.speed_percent, 25)

    def test_tampering_is_rejected(self) -> None:
        encoded = encode_request(
            MessageType.HEARTBEAT,
            SESSION,
            8,
            secret=SECRET,
            sent_at_ms=NOW_MS,
        )
        packet = json.loads(encoded)
        packet["seq"] = 9
        tampered = json.dumps(packet, separators=(",", ":")).encode()
        with self.assertRaisesRegex(ProtocolError, "authentication failed"):
            decode_request(tampered, secret=SECRET, now_ms=NOW_MS)

    def test_unsigned_mode_must_be_explicit(self) -> None:
        with self.assertRaisesRegex(ValueError, "shared secret is required"):
            encode_request(
                MessageType.HEARTBEAT,
                SESSION,
                0,
                secret="",
                sent_at_ms=NOW_MS,
            )
        encoded = encode_request(
            MessageType.HEARTBEAT,
            SESSION,
            0,
            secret="",
            allow_insecure=True,
            sent_at_ms=NOW_MS,
        )
        request = decode_request(
            encoded,
            secret="",
            allow_insecure=True,
            now_ms=NOW_MS,
        )
        self.assertEqual(request.kind, MessageType.HEARTBEAT)

    def test_old_and_future_packets_are_rejected(self) -> None:
        for sent_at in (NOW_MS - 5001, NOW_MS + 5001):
            encoded = encode_request(
                MessageType.HEARTBEAT,
                SESSION,
                1,
                secret=SECRET,
                sent_at_ms=sent_at,
            )
            with self.assertRaises(ProtocolError) as context:
                decode_request(encoded, secret=SECRET, now_ms=NOW_MS, freshness_ms=5000)
            self.assertEqual(context.exception.code, AckCode.STALE_TIMESTAMP)

    def test_duplicate_json_keys_are_rejected(self) -> None:
        duplicate = (
            b'{"v":1,"v":1,"type":"heartbeat","session":"'
            + SESSION.encode()
            + b'","seq":0,"sent_at_ms":'
            + str(NOW_MS).encode()
            + b"}"
        )
        with self.assertRaisesRegex(ProtocolError, "strict JSON"):
            decode_request(
                duplicate,
                secret="",
                allow_insecure=True,
                now_ms=NOW_MS,
            )

    def test_ack_round_trip_preserves_state_and_estimate(self) -> None:
        ack = Ack(
            session_id=SESSION,
            sequence=12,
            code=AckCode.OK,
            state=ControllerState.HARDWARE_READY | ControllerState.OUTPUTS_ENABLED,
            angles_deg=(90, 91, 92, 93, 94, 95),
            sent_at_ms=NOW_MS,
            detail="accepted",
        )
        decoded = decode_ack(
            encode_ack(ack, secret=SECRET),
            secret=SECRET,
            now_ms=NOW_MS,
        )
        self.assertEqual(decoded, ack)

    def test_command_validation_rejects_shape_limits_and_speed(self) -> None:
        invalid = (
            ([0, 0, 0, 0, 0], 50),
            ([246, 0, 0, 0, 0, 0], 50),
            ([0, 0, 0, 0, 0, 0], 0),
            ([0, 0, 0, 0, 0, 0], 101),
            ([0, 0, float("nan"), 0, 0, 0], 50),
        )
        for angles, speed in invalid:
            with self.subTest(angles=angles, speed=speed):
                with self.assertRaises(ValueError):
                    validate_joint_command(angles, speed)


if __name__ == "__main__":
    unittest.main()

