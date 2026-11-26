"""Safety state machine and UDP server for the Raspberry Pi arm controller."""

from __future__ import annotations

import logging
import select
import signal
import socket
import threading
import time
from collections import OrderedDict

from .config import (
    ConfigError,
    ControllerConfig,
    compile_allowed_clients,
    load_controller_config,
    matches_allowed_clients,
)
from .hardware import HardwareError, ServoHardware, create_hardware
from .protocol import (
    MAX_DATAGRAM_BYTES,
    Ack,
    AckCode,
    ControllerState,
    MessageType,
    ProtocolError,
    Request,
    decode_request,
    encode_ack,
    validate_joint_command,
)


LOGGER = logging.getLogger("rashko_controller")


class ArmController:
    """Pure safety/motion core; call :meth:`tick` frequently from the UDP loop."""

    def __init__(
        self,
        config: ControllerConfig,
        hardware: ServoHardware,
    ) -> None:
        self.config = config
        self.hardware = hardware
        self.estop_latched = True
        self.watchdog_tripped = True
        self.hardware_ready = hardware.ready
        self.current_angles = tuple(config.hardware.home_degrees)
        self.target_angles = self.current_angles
        self.speed_percent = 1.0
        self.last_contact_at: float | None = None
        self.last_update_at: float | None = None
        self._motion_owner: tuple[str, str] | None = None
        self._sessions: OrderedDict[str, tuple[str, int]] = OrderedDict()
        # Constructor has no PWM write or enable call: startup cannot move the
        # arm. Explicitly assert OE HIGH even if a future driver forgot to.
        try:
            self.hardware.disable_outputs()
        except Exception:
            self.hardware_ready = False

    @property
    def state(self) -> ControllerState:
        state = ControllerState.NONE
        if self.estop_latched:
            state |= ControllerState.ESTOP_LATCHED
        if self.watchdog_tripped:
            state |= ControllerState.WATCHDOG_TRIPPED
        if self.hardware.outputs_enabled:
            state |= ControllerState.OUTPUTS_ENABLED
        if self.hardware_ready and self.hardware.ready:
            state |= ControllerState.HARDWARE_READY
        if self.config.allow_insecure and not self.config.shared_secret:
            state |= ControllerState.INSECURE_MODE
        return state

    def process(
        self,
        request: Request,
        client_ip: str,
        *,
        now_monotonic: float | None = None,
        now_wall_ms: int | None = None,
    ) -> Ack:
        now = time.monotonic() if now_monotonic is None else now_monotonic
        if request.kind is MessageType.STATUS:
            # Diagnostics must not change watchdog timing or evict motion sessions.
            return self._ack(request, AckCode.OK, "status", now_wall_ms)
        self._expire_watchdog(now)
        sender = (request.session_id, client_ip)
        previous = self._sessions.get(request.session_id)
        if previous is not None:
            previous_ip, previous_sequence = previous
            if client_ip != previous_ip or request.sequence <= previous_sequence:
                self._sessions.move_to_end(request.session_id)
                detail = (
                    "session is bound to another client"
                    if client_ip != previous_ip
                    else "sequence was already processed"
                )
                return self._ack(
                    request,
                    AckCode.STALE_SEQUENCE,
                    detail,
                    now_wall_ms,
                )
        self._sessions[request.session_id] = (client_ip, request.sequence)
        self._sessions.move_to_end(request.session_id)
        while len(self._sessions) > self.config.max_sessions:
            oldest = next(iter(self._sessions))
            if self._motion_owner is not None and oldest == self._motion_owner[0]:
                self._sessions.move_to_end(oldest)
            self._sessions.popitem(last=False)

        if request.kind is MessageType.EMERGENCY_STOP:
            self._motion_owner = None
            self.last_contact_at = None
            self.estop_latched = True
            self.watchdog_tripped = True
            self.target_angles = self.current_angles
            if not self._disable_outputs():
                return self._ack(
                    request, AckCode.HARDWARE_ERROR, "failed to disable outputs", now_wall_ms
                )
            return self._ack(request, AckCode.OK, "emergency stop latched", now_wall_ms)

        if request.kind is MessageType.RESET_ESTOP:
            if self.hardware.outputs_enabled and self._motion_owner != sender:
                return self._ack(request, AckCode.CONTROL_BUSY, "another session owns motion", now_wall_ms)
            self._motion_owner = None
            self.last_contact_at = None
            self.target_angles = self.current_angles
            # Reset never enables or writes PWM.  Keep OE HIGH until a subsequent
            # fresh, valid command has been received.
            if not self._disable_outputs():
                self.estop_latched = True
                return self._ack(
                    request, AckCode.HARDWARE_ERROR, "failed to hold outputs disabled", now_wall_ms
                )
            self._motion_owner = sender
            self.estop_latched = False
            self.watchdog_tripped = True
            return self._ack(
                request,
                AckCode.OK,
                "E-stop reset; fresh motion command required",
                now_wall_ms,
            )

        if request.kind is MessageType.HEARTBEAT:
            if self._motion_owner == sender:
                self.last_contact_at = now
            elif self._motion_owner is not None:
                return self._ack(request, AckCode.CONTROL_BUSY, "another session owns motion", now_wall_ms)
            return self._ack(request, AckCode.OK, "heartbeat", now_wall_ms)

        if request.kind is not MessageType.COMMAND:
            return self._ack(request, AckCode.BAD_PACKET, "unsupported request", now_wall_ms)

        try:
            angles, speed = validate_joint_command(
                request.angles_deg, request.speed_percent  # type: ignore[arg-type]
            )
        except (TypeError, ValueError) as exc:
            return self._ack(request, AckCode.INVALID_COMMAND, str(exc), now_wall_ms)
        for servo, angle in zip(self.config.hardware.servos, angles):
            if not servo.logical_min_deg <= angle <= servo.logical_max_deg:
                return self._ack(
                    request,
                    AckCode.INVALID_COMMAND,
                    (
                        f"{servo.name}={angle} is outside calibrated range "
                        f"{servo.logical_min_deg}..{servo.logical_max_deg} degrees"
                    ),
                    now_wall_ms,
                )
        if self.estop_latched:
            return self._ack(
                request,
                AckCode.ESTOP_LATCHED,
                "reset emergency stop before commanding motion",
                now_wall_ms,
            )
        if not self.hardware_ready or not self.hardware.ready:
            self._hardware_failure()
            return self._ack(
                request, AckCode.HARDWARE_ERROR, "servo hardware is not ready", now_wall_ms
            )

        if self._motion_owner != sender:
            return self._ack(request, AckCode.CONTROL_BUSY, "reset is required in this motion session", now_wall_ms)
        self.last_contact_at = now
        self.target_angles = angles
        self.speed_percent = speed
        self.last_update_at = now
        if not self.hardware.outputs_enabled:
            try:
                # Prepare the configured startup estimate while OE remains HIGH,
                # then lower OE. The commissioning procedure requires the arm to
                # be physically placed at this startup pose before first enable.
                self.hardware.write_angles(self.current_angles)
                self.hardware.enable_outputs()
            except (HardwareError, OSError, RuntimeError, ValueError):
                self._hardware_failure()
                return self._ack(
                    request,
                    AckCode.HARDWARE_ERROR,
                    "failed to prepare or enable servo outputs",
                    now_wall_ms,
                )
        self.watchdog_tripped = False
        return self._ack(request, AckCode.OK, "motion target accepted", now_wall_ms)

    def _expire_watchdog(self, now: float) -> None:
        if (
            self.hardware.outputs_enabled
            and (
                self.last_contact_at is None
                or now - self.last_contact_at > self.config.watchdog_ms / 1000.0
            )
        ):
            self.watchdog_tripped = True
            self.estop_latched = True
            self.target_angles = self.current_angles
            self._motion_owner = None
            self.last_contact_at = None
            self._disable_outputs()
            self.last_update_at = now

    def tick(self, now_monotonic: float | None = None) -> None:
        now = time.monotonic() if now_monotonic is None else now_monotonic
        self._expire_watchdog(now)
        if (
            not self.hardware.outputs_enabled
            or self.estop_latched
            or not self.hardware_ready
        ):
            self.last_update_at = now
            return
        if self.last_update_at is None:
            self.last_update_at = now
            return

        # Cap the step after scheduling delays; this prevents a stalled process
        # from issuing one large catch-up jump when it resumes.
        elapsed = max(0.0, min(now - self.last_update_at, 0.1))
        self.last_update_at = now
        if elapsed <= 0.0:
            return
        largest_delta = max(
            abs(target - current)
            for target, current in zip(self.target_angles, self.current_angles)
        )
        if largest_delta <= 1e-9:
            return
        degrees_per_second = (
            self.config.hardware.max_speed_deg_per_second * self.speed_percent / 100.0
        )
        fraction = min(1.0, degrees_per_second * elapsed / largest_delta)
        next_angles = tuple(
            current + (target - current) * fraction
            for target, current in zip(self.target_angles, self.current_angles)
        )
        try:
            self.hardware.write_angles(next_angles)
        except (HardwareError, OSError, RuntimeError, ValueError):
            self._hardware_failure()
            return
        self.current_angles = next_angles

    def close(self) -> None:
        self.estop_latched = True
        self.watchdog_tripped = True
        self.target_angles = self.current_angles
        try:
            self.hardware.close()
        except Exception:
            self.hardware_ready = False
            LOGGER.exception("hardware close failed; verify OE is electrically HIGH")

    def _disable_outputs(self) -> bool:
        try:
            self.hardware.disable_outputs()
            return True
        except Exception:
            self._hardware_failure()
            return False

    def _hardware_failure(self) -> None:
        self._motion_owner = None
        self.hardware_ready = False
        self.estop_latched = True
        self.watchdog_tripped = True
        self.target_angles = self.current_angles
        try:
            self.hardware.disable_outputs()
        except Exception:
            LOGGER.exception("could not confirm OE HIGH after hardware failure")

    def _ack(
        self,
        request: Request,
        code: AckCode,
        detail: str,
        now_wall_ms: int | None,
    ) -> Ack:
        timestamp = time.time_ns() // 1_000_000 if now_wall_ms is None else now_wall_ms
        return Ack(
            session_id=request.session_id,
            sequence=request.sequence,
            code=code,
            state=self.state,
            angles_deg=self.current_angles,
            sent_at_ms=timestamp,
            detail=detail,
        )


class UdpControllerServer:
    def __init__(
        self,
        config: ControllerConfig,
        controller: ArmController,
        udp_socket: socket.socket | None = None,
    ) -> None:
        self.config = config
        self.controller = controller
        self._allowed_exact_ips, self._allowed_networks = compile_allowed_clients(
            config.allowed_clients
        )
        self.socket = udp_socket or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if udp_socket is None:
            self.socket.bind((config.bind_host, config.port))
        self.socket.setblocking(False)

    def handle_datagram(
        self,
        datagram: bytes,
        address: tuple[str, int],
        *,
        now_wall_ms: int | None = None,
        now_monotonic: float | None = None,
    ) -> bytes | None:
        client_ip = address[0]
        if not matches_allowed_clients(
            client_ip,
            self._allowed_exact_ips,
            self._allowed_networks,
        ):
            LOGGER.warning("discarded packet from non-allowlisted client %s", client_ip)
            return None
        try:
            request = decode_request(
                datagram,
                secret=self.config.shared_secret,
                allow_insecure=self.config.allow_insecure,
                now_ms=now_wall_ms,
                freshness_ms=self.config.freshness_ms,
            )
        except ProtocolError as exc:
            LOGGER.warning("rejected UDP packet from %s: %s", client_ip, exc)
            # Authentication/freshness/structure failures receive no response.
            # This avoids an unauthenticated reflection surface.
            return None
        ack = self.controller.process(
            request,
            client_ip,
            now_monotonic=now_monotonic,
            now_wall_ms=now_wall_ms,
        )
        return encode_ack(
            ack,
            secret=self.config.shared_secret,
            allow_insecure=self.config.allow_insecure,
        )

    def serve_forever(self, stop_event: threading.Event | None = None) -> None:
        stop = stop_event or threading.Event()
        period = 1.0 / self.config.update_hz
        next_tick = time.monotonic()
        while not stop.is_set():
            now = time.monotonic()
            timeout = max(0.0, min(period, next_tick - now))
            try:
                readable, _, _ = select.select([self.socket], [], [], timeout)
            except (OSError, ValueError):
                if stop.is_set():
                    break
                raise
            if readable:
                # Bound each drain so packet floods cannot starve the watchdog.
                for _ in range(64):
                    try:
                        datagram, address = self.socket.recvfrom(MAX_DATAGRAM_BYTES + 1)
                    except (BlockingIOError, InterruptedError):
                        break
                    response = self.handle_datagram(datagram, address)
                    if response is not None:
                        try:
                            self.socket.sendto(response, address)
                        except OSError:
                            LOGGER.exception("failed to send ACK to %s", address[0])
            now = time.monotonic()
            if now >= next_tick:
                self.controller.tick(now)
                next_tick = now + period

    def close(self) -> None:
        try:
            self.controller.close()
        finally:
            self.socket.close()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    controller: ArmController | None = None
    hardware: ServoHardware | None = None
    try:
        config = load_controller_config()
        hardware = create_hardware(config.hardware_backend, config.hardware)
        controller = ArmController(config, hardware)
        server = UdpControllerServer(config, controller)
    except (ConfigError, HardwareError, OSError, ValueError) as exc:
        if controller is not None:
            controller.close()
        elif hardware is not None:
            try:
                hardware.close()
            except Exception:
                LOGGER.exception("hardware cleanup failed during refused startup")
        LOGGER.critical("controller startup refused: %s", exc)
        raise SystemExit(2) from exc

    stop_event = threading.Event()

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    security_mode = (
        "INSECURE"
        if config.allow_insecure and not config.shared_secret
        else "authenticated"
    )
    LOGGER.info(
        "listening on %s:%d (%s, backend=%s); OE remains HIGH until reset + command",
        config.bind_host,
        config.port,
        security_mode,
        config.hardware_backend,
    )
    try:
        server.serve_forever(stop_event)
    finally:
        server.close()


if __name__ == "__main__":
    main()
