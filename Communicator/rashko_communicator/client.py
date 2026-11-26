"""Non-blocking authenticated UDP client used by the laptop ROS bridge."""

from __future__ import annotations

import secrets
import select
import socket
import threading
import time
from collections.abc import Sequence

from .protocol import (
    DEFAULT_FRESHNESS_MS,
    Ack,
    MessageType,
    ProtocolError,
    decode_ack,
    encode_request,
    validate_joint_command,
    validate_secret,
)


class UdpArmClient:
    """Send commands and drain signed ACKs without blocking a ROS executor."""

    def __init__(
        self,
        host: str,
        port: int = 5005,
        *,
        secret: str | bytes | None,
        allow_insecure: bool = False,
        ack_freshness_ms: int = DEFAULT_FRESHNESS_MS * 2,
    ) -> None:
        if not isinstance(host, str) or not host.strip():
            raise ValueError("host is required")
        if isinstance(port, bool) or not 1 <= int(port) <= 65535:
            raise ValueError("port must be in 1..65535")
        self._secret = validate_secret(secret, allow_insecure)
        self._allow_insecure = bool(allow_insecure)
        self._ack_freshness_ms = int(ack_freshness_ms)
        if self._ack_freshness_ms < 1:
            raise ValueError("ack_freshness_ms must be positive")

        addresses = socket.getaddrinfo(
            host,
            int(port),
            family=socket.AF_INET,
            type=socket.SOCK_DGRAM,
        )
        if not addresses:
            raise OSError(f"could not resolve {host}")
        family, socket_type, protocol, _, server_address = addresses[0]
        self._socket = socket.socket(family, socket_type, protocol)
        self._socket.connect(server_address)
        self._socket.setblocking(False)
        self._session_id = secrets.token_hex(16)
        self._sequence = -1
        self._send_lock = threading.Lock()
        self._closed = False

    @property
    def session_id(self) -> str:
        return self._session_id

    def send_command(
        self,
        angles_deg: Sequence[float],
        speed_percent: float,
    ) -> int:
        angles, speed = validate_joint_command(angles_deg, speed_percent)
        return self._send(
            MessageType.COMMAND,
            angles_deg=angles,
            speed_percent=speed,
        )

    def send_heartbeat(self) -> int:
        return self._send(MessageType.HEARTBEAT)

    def send_status(self) -> int:
        return self._send(MessageType.STATUS)

    def send_emergency_stop(self) -> int:
        return self._send(MessageType.EMERGENCY_STOP)

    def send_reset_estop(self) -> int:
        return self._send(MessageType.RESET_ESTOP)

    def receive_acks(self, max_packets: int = 64) -> list[Ack]:
        if self._closed:
            return []
        if not 1 <= max_packets <= 1024:
            raise ValueError("max_packets must be in 1..1024")
        acknowledgements: list[Ack] = []
        for _ in range(max_packets):
            try:
                datagram = self._socket.recv(4097)
            except (BlockingIOError, InterruptedError):
                break
            ack = decode_ack(
                datagram,
                secret=self._secret,
                allow_insecure=self._allow_insecure,
                freshness_ms=self._ack_freshness_ms,
            )
            if ack.session_id != self._session_id:
                raise ProtocolError("ACK belongs to a different client session")
            if ack.sequence > self._sequence:
                raise ProtocolError("ACK sequence was never sent by this client")
            acknowledgements.append(ack)
        return acknowledgements

    def wait_for_ack(self, sequence: int, timeout_s: float = 1.0) -> Ack:
        if timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f"no ACK received for sequence {sequence}")
            readable, _, _ = select.select([self._socket], [], [], remaining)
            if not readable:
                raise TimeoutError(f"no ACK received for sequence {sequence}")
            for ack in self.receive_acks():
                if ack.sequence == sequence:
                    return ack

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            self._socket.close()

    def __enter__(self) -> "UdpArmClient":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def _send(
        self,
        kind: MessageType,
        *,
        angles_deg: Sequence[float] | None = None,
        speed_percent: float | None = None,
    ) -> int:
        if self._closed:
            raise OSError("UDP client is closed")
        with self._send_lock:
            self._sequence += 1
            sequence = self._sequence
            datagram = encode_request(
                kind,
                self._session_id,
                sequence,
                secret=self._secret,
                allow_insecure=self._allow_insecure,
                angles_deg=angles_deg,
                speed_percent=speed_percent,
            )
            sent = self._socket.send(datagram)
            if sent != len(datagram):
                raise OSError(f"partial UDP send: {sent}/{len(datagram)} bytes")
            return sequence

