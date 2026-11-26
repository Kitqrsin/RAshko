"""Small authenticated UDP wire protocol shared by the laptop and Raspberry Pi.

Packets are canonical JSON followed by an HMAC-SHA256 authentication field.
Every request contains a random client-session identifier, a monotonically
increasing sequence number, and a wall-clock timestamp.  The controller checks
all three: session/sequence prevents in-session replay and timestamp freshness
prevents a captured packet from becoming valid again after a Pi reboot.
"""

from __future__ import annotations

import hashlib
import hmac
import json
import math
import time
from dataclasses import dataclass
from enum import Enum, IntEnum, IntFlag
from typing import Any, Mapping, Sequence


PROTOCOL_VERSION = 1
MAX_DATAGRAM_BYTES = 4096
MAX_SEQUENCE = (1 << 63) - 1
DEFAULT_FRESHNESS_MS = 5_000
JOINT_NAMES = (
    "base",
    "shoulder",
    "elbow",
    "wrist",
    "wrist_rotate",
    "gripper",
)
MIN_ANGLES_DEG = (0.0,) * 6
MAX_ANGLES_DEG = (245.0, 180.0, 180.0, 180.0, 180.0, 180.0)


class MessageType(str, Enum):
    COMMAND = "command"
    HEARTBEAT = "heartbeat"
    STATUS = "status"
    EMERGENCY_STOP = "estop"
    RESET_ESTOP = "reset_estop"
    ACK = "ack"


class AckCode(IntEnum):
    OK = 0
    BAD_PACKET = 1
    AUTH_FAILED = 2
    STALE_SEQUENCE = 3
    INVALID_COMMAND = 4
    ESTOP_LATCHED = 5
    HARDWARE_ERROR = 6
    STALE_TIMESTAMP = 7
    CONTROL_BUSY = 8


class ControllerState(IntFlag):
    NONE = 0
    ESTOP_LATCHED = 1 << 0
    WATCHDOG_TRIPPED = 1 << 1
    OUTPUTS_ENABLED = 1 << 2
    HARDWARE_READY = 1 << 3
    INSECURE_MODE = 1 << 4


class ProtocolError(ValueError):
    """A packet failed structural, authentication, or freshness validation."""

    def __init__(self, message: str, code: AckCode = AckCode.BAD_PACKET) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True, slots=True)
class Request:
    kind: MessageType
    session_id: str
    sequence: int
    sent_at_ms: int
    angles_deg: tuple[Any, ...] | None = None
    speed_percent: Any = None


@dataclass(frozen=True, slots=True)
class Ack:
    session_id: str
    sequence: int
    code: AckCode
    state: ControllerState
    angles_deg: tuple[float, ...]
    sent_at_ms: int
    detail: str = ""


def validate_secret(secret: str | bytes | None, allow_insecure: bool) -> bytes:
    """Return a byte secret, refusing accidental unauthenticated operation."""

    if isinstance(secret, str):
        value = secret.encode("utf-8")
    elif isinstance(secret, bytes):
        value = secret
    elif secret is None:
        value = b""
    else:
        raise TypeError("secret must be text or bytes")
    if value and len(value) < 32:
        raise ValueError("shared secret must contain at least 32 bytes")
    if not value and not allow_insecure:
        raise ValueError("shared secret is required unless insecure mode is explicitly enabled")
    return value


def validate_joint_command(
    values: Sequence[Any],
    speed_percent: Any,
) -> tuple[tuple[float, ...], float]:
    if (
        not isinstance(values, Sequence)
        or isinstance(values, (str, bytes))
        or len(values) != len(JOINT_NAMES)
    ):
        raise ValueError(f"expected exactly {len(JOINT_NAMES)} joint angles")
    angles: list[float] = []
    for index, raw_value in enumerate(values):
        if isinstance(raw_value, bool):
            raise ValueError(f"{JOINT_NAMES[index]} angle must be numeric")
        try:
            value = float(raw_value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{JOINT_NAMES[index]} angle must be numeric") from exc
        if not math.isfinite(value):
            raise ValueError(f"{JOINT_NAMES[index]} angle must be finite")
        if not MIN_ANGLES_DEG[index] <= value <= MAX_ANGLES_DEG[index]:
            raise ValueError(
                f"{JOINT_NAMES[index]}={value} is outside "
                f"{MIN_ANGLES_DEG[index]}..{MAX_ANGLES_DEG[index]} degrees"
            )
        angles.append(value)
    if isinstance(speed_percent, bool):
        raise ValueError("speed must be numeric")
    try:
        speed = float(speed_percent)
    except (TypeError, ValueError) as exc:
        raise ValueError("speed must be numeric") from exc
    if not math.isfinite(speed) or not 1.0 <= speed <= 100.0:
        raise ValueError("speed must be finite and in the range 1..100 percent")
    return tuple(angles), speed


def encode_request(
    kind: MessageType,
    session_id: str,
    sequence: int,
    *,
    secret: str | bytes | None,
    allow_insecure: bool = False,
    sent_at_ms: int | None = None,
    angles_deg: Sequence[float] | None = None,
    speed_percent: float | None = None,
) -> bytes:
    if kind is MessageType.ACK:
        raise ValueError("use encode_ack for acknowledgements")
    _validate_session_and_sequence(session_id, sequence)
    packet: dict[str, Any] = {
        "v": PROTOCOL_VERSION,
        "type": kind.value,
        "session": session_id,
        "seq": sequence,
        "sent_at_ms": _wall_time_ms() if sent_at_ms is None else sent_at_ms,
    }
    if kind is MessageType.COMMAND:
        if angles_deg is None or speed_percent is None:
            raise ValueError("command requires angles and speed")
        angles, speed = validate_joint_command(angles_deg, speed_percent)
        packet["angles_deg"] = list(angles)
        packet["speed_percent"] = speed
    elif angles_deg is not None or speed_percent is not None:
        raise ValueError(f"{kind.value} does not accept angles or speed")
    return _encode_signed(packet, secret, allow_insecure)


def decode_request(
    datagram: bytes,
    *,
    secret: str | bytes | None,
    allow_insecure: bool = False,
    now_ms: int | None = None,
    freshness_ms: int = DEFAULT_FRESHNESS_MS,
) -> Request:
    packet = _decode_signed(datagram, secret, allow_insecure)
    _require_exact_keys(
        packet,
        {"v", "type", "session", "seq", "sent_at_ms"},
        {"angles_deg", "speed_percent", "auth"},
    )
    _validate_version(packet)
    try:
        kind = MessageType(packet["type"])
    except (ValueError, TypeError) as exc:
        raise ProtocolError("unknown request type") from exc
    if kind is MessageType.ACK:
        raise ProtocolError("ACK is not a request")
    session_id = packet["session"]
    sequence = packet["seq"]
    sent_at_ms = packet["sent_at_ms"]
    _validate_session_and_sequence(session_id, sequence)
    _validate_timestamp(sent_at_ms, now_ms, freshness_ms)
    if kind is MessageType.COMMAND:
        if "angles_deg" not in packet or "speed_percent" not in packet:
            raise ProtocolError("command is missing angles or speed")
        angles = packet["angles_deg"]
        if not isinstance(angles, list):
            raise ProtocolError("angles_deg must be an array")
        return Request(
            kind,
            session_id,
            sequence,
            sent_at_ms,
            tuple(angles),
            packet["speed_percent"],
        )
    if "angles_deg" in packet or "speed_percent" in packet:
        raise ProtocolError(f"{kind.value} cannot contain angles or speed")
    return Request(kind, session_id, sequence, sent_at_ms)


def encode_ack(
    ack: Ack,
    *,
    secret: str | bytes | None,
    allow_insecure: bool = False,
) -> bytes:
    _validate_session_and_sequence(ack.session_id, ack.sequence)
    angles, _ = validate_joint_command(ack.angles_deg, 100.0)
    detail = str(ack.detail)
    if len(detail) > 160:
        detail = detail[:160]
    packet = {
        "v": PROTOCOL_VERSION,
        "type": MessageType.ACK.value,
        "session": ack.session_id,
        "seq": ack.sequence,
        "sent_at_ms": ack.sent_at_ms,
        "code": int(ack.code),
        "state": int(ack.state),
        "angles_deg": list(angles),
        "detail": detail,
    }
    return _encode_signed(packet, secret, allow_insecure)


def decode_ack(
    datagram: bytes,
    *,
    secret: str | bytes | None,
    allow_insecure: bool = False,
    now_ms: int | None = None,
    freshness_ms: int = DEFAULT_FRESHNESS_MS * 2,
) -> Ack:
    packet = _decode_signed(datagram, secret, allow_insecure)
    _require_exact_keys(
        packet,
        {
            "v",
            "type",
            "session",
            "seq",
            "sent_at_ms",
            "code",
            "state",
            "angles_deg",
            "detail",
        },
        {"auth"},
    )
    _validate_version(packet)
    if packet["type"] != MessageType.ACK.value:
        raise ProtocolError("response is not an ACK")
    _validate_session_and_sequence(packet["session"], packet["seq"])
    _validate_timestamp(packet["sent_at_ms"], now_ms, freshness_ms)
    try:
        code = AckCode(_strict_int(packet["code"], "code"))
        state_value = _strict_int(packet["state"], "state")
        known_state_mask = 0
        for flag in ControllerState:
            known_state_mask |= int(flag)
        if state_value < 0 or state_value & ~known_state_mask:
            raise ValueError("unknown state flag")
        state = ControllerState(state_value)
        angles, _ = validate_joint_command(packet["angles_deg"], 100.0)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"invalid ACK fields: {exc}") from exc
    if not isinstance(packet["detail"], str):
        raise ProtocolError("ACK detail must be text")
    return Ack(
        packet["session"],
        packet["seq"],
        code,
        state,
        angles,
        packet["sent_at_ms"],
        packet["detail"],
    )


def _encode_signed(
    packet: Mapping[str, Any],
    secret: str | bytes | None,
    allow_insecure: bool,
) -> bytes:
    key = validate_secret(secret, allow_insecure)
    body = dict(packet)
    if key:
        body["auth"] = hmac.new(key, _canonical_json(body), hashlib.sha256).hexdigest()
    encoded = _canonical_json(body)
    if len(encoded) > MAX_DATAGRAM_BYTES:
        raise ValueError("packet is too large")
    return encoded


def _decode_signed(
    datagram: bytes,
    secret: str | bytes | None,
    allow_insecure: bool,
) -> dict[str, Any]:
    key = validate_secret(secret, allow_insecure)
    if not isinstance(datagram, bytes):
        raise ProtocolError("datagram must be bytes")
    if not datagram or len(datagram) > MAX_DATAGRAM_BYTES:
        raise ProtocolError("datagram is empty or too large")
    try:
        packet = json.loads(
            datagram.decode("utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)),
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise ProtocolError("datagram is not strict JSON") from exc
    if not isinstance(packet, dict):
        raise ProtocolError("packet must be a JSON object")
    supplied_auth = packet.pop("auth", None)
    if key:
        if not isinstance(supplied_auth, str) or len(supplied_auth) != 64:
            raise ProtocolError("missing authentication", AckCode.AUTH_FAILED)
        expected = hmac.new(key, _canonical_json(packet), hashlib.sha256).hexdigest()
        if not hmac.compare_digest(supplied_auth, expected):
            raise ProtocolError("authentication failed", AckCode.AUTH_FAILED)
        packet["auth"] = supplied_auth
    elif supplied_auth is not None:
        raise ProtocolError("authenticated packet cannot be checked without a secret")
    return packet


def _canonical_json(value: Mapping[str, Any]) -> bytes:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("ascii")


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate key: {key}")
        result[key] = value
    return result


def _require_exact_keys(
    packet: Mapping[str, Any],
    required: set[str],
    optional: set[str],
) -> None:
    keys = set(packet)
    missing = required - keys
    unexpected = keys - required - optional
    if missing:
        raise ProtocolError(f"missing packet fields: {sorted(missing)}")
    if unexpected:
        raise ProtocolError(f"unexpected packet fields: {sorted(unexpected)}")


def _validate_version(packet: Mapping[str, Any]) -> None:
    if _strict_int(packet.get("v"), "v") != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")


def _validate_session_and_sequence(session_id: Any, sequence: Any) -> None:
    if (
        not isinstance(session_id, str)
        or len(session_id) != 32
        or any(character not in "0123456789abcdef" for character in session_id)
    ):
        raise ProtocolError("session must be 16 random bytes encoded as lowercase hex")
    sequence_value = _strict_int(sequence, "sequence")
    if not 0 <= sequence_value <= MAX_SEQUENCE:
        raise ProtocolError("sequence is outside the supported range")


def _validate_timestamp(sent_at_ms: Any, now_ms: int | None, freshness_ms: int) -> None:
    timestamp = _strict_int(sent_at_ms, "sent_at_ms")
    if freshness_ms < 1:
        raise ValueError("freshness_ms must be positive")
    current = _wall_time_ms() if now_ms is None else now_ms
    if abs(current - timestamp) > freshness_ms:
        raise ProtocolError(
            "packet timestamp is outside the freshness window",
            AckCode.STALE_TIMESTAMP,
        )


def _strict_int(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError(f"{field} must be an integer")
    return value


def _wall_time_ms() -> int:
    return time.time_ns() // 1_000_000
