"""Command-line diagnostic client for the Raspberry Pi controller."""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections.abc import Sequence

from .client import UdpArmClient
from .protocol import AckCode, ControllerState, ProtocolError


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send one authenticated command and wait for its Raspberry Pi ACK"
    )
    parser.add_argument("host", help="Raspberry Pi IPv4 address or hostname")
    parser.add_argument(
        "angles",
        nargs="*",
        type=float,
        help="six logical degrees: base shoulder elbow wrist wrist_rotate gripper",
    )
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--speed", type=float, default=25.0, help="motion speed, 1..100")
    parser.add_argument("--timeout", type=float, default=1.5, help="ACK timeout in seconds")
    parser.add_argument(
        "--allow-insecure",
        action="store_true",
        help="explicitly permit unsigned packets (commissioning only)",
    )
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--heartbeat", action="store_true", help="send a motion-session heartbeat")
    action.add_argument("--status", action="store_true", help="request status without renewing the motion watchdog")
    action.add_argument("--estop", action="store_true", help="latch emergency stop")
    action.add_argument(
        "--reset-estop",
        action="store_true",
        help="clear the latch; a later fresh command is still required to enable outputs",
    )
    return parser


def run(argv: Sequence[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    is_control = arguments.status or arguments.heartbeat or arguments.estop or arguments.reset_estop
    if is_control and arguments.angles:
        raise SystemExit("angles cannot be supplied with a control-only option")
    if not is_control and len(arguments.angles) != 6:
        raise SystemExit("a motion command requires exactly six angles")

    secret = os.environ.get("RASHKO_SHARED_SECRET", "")
    try:
        with UdpArmClient(
            arguments.host,
            arguments.port,
            secret=secret,
            allow_insecure=arguments.allow_insecure,
        ) as client:
            if arguments.status:
                sequence = client.send_status()
            elif arguments.heartbeat:
                sequence = client.send_heartbeat()
            elif arguments.estop:
                sequence = client.send_emergency_stop()
            elif arguments.reset_estop:
                sequence = client.send_reset_estop()
            else:
                sequence = client.send_command(arguments.angles, arguments.speed)
            ack = client.wait_for_ack(sequence, arguments.timeout)
    except (OSError, ProtocolError, TimeoutError, ValueError) as exc:
        print(f"communication failed: {exc}", file=sys.stderr)
        return 2

    state_names = [flag.name for flag in ControllerState if flag in ack.state]
    print(
        json.dumps(
            {
                "sequence": ack.sequence,
                "code": ack.code.name,
                "state": state_names,
                "angles_deg": [round(value, 3) for value in ack.angles_deg],
                "detail": ack.detail,
            },
            separators=(",", ":"),
        )
    )
    return 0 if ack.code is AckCode.OK else 1


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
