"""Pure validation helpers for RAshko joint trajectories.

The ROS action server uses these helpers before publishing any servo target.
Keeping the validation independent of :mod:`rclpy` also makes the safety
contract straightforward to unit test on Windows.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence

from arm_controller.joint_contract import (
    HOME_DEGREES,
    JOINT_NAMES,
    merge_joint_radians,
    validate_servo_degrees,
)


class TrajectoryValidationError(ValueError):
    """Raised when a trajectory cannot be sent through the servo contract."""


_ARM_JOINTS = tuple(JOINT_NAMES[:5])
_ALL_JOINTS = tuple(JOINT_NAMES)
_GRIPPER_JOINT = (JOINT_NAMES[5],)
_ALLOWED_ORDERS = frozenset((_ARM_JOINTS, _ALL_JOINTS, _GRIPPER_JOINT))


def validate_trajectory_rows(
    joint_names: Sequence[str],
    position_rows: Sequence[Sequence[float]],
    times_from_start_s: Sequence[float],
    current_degrees: Sequence[float],
    maximum_horizon_s: float = 120.0,
) -> tuple[tuple[float, ...], ...]:
    """Validate and convert a complete trajectory to six logical degrees.

    Only MoveIt's five pose joints, the gripper joint alone, or all six
    canonical joints are accepted.
    This deliberately rejects arbitrary partial/unknown joint vectors while
    still allowing normal plans for the SRDF ``arm`` group to preserve the
    current gripper position.
    """

    names = tuple(str(name) for name in joint_names)
    if names not in _ALLOWED_ORDERS:
        raise TrajectoryValidationError(
            "joint names must be the five canonical arm joints, the canonical "
            "gripper alone, or all six "
            f"canonical joints in order; got {list(names)!r}"
        )
    if len(position_rows) == 0:
        raise TrajectoryValidationError("trajectory contains no points")
    if len(position_rows) != len(times_from_start_s):
        raise TrajectoryValidationError("trajectory point/time counts differ")
    if len(current_degrees) != len(JOINT_NAMES):
        raise TrajectoryValidationError("current state must contain six joints")
    if any(not math.isfinite(float(value)) for value in current_degrees):
        raise TrajectoryValidationError("current state contains a non-finite value")
    if (
        isinstance(maximum_horizon_s, bool)
        or not math.isfinite(float(maximum_horizon_s))
        or float(maximum_horizon_s) <= 0.0
    ):
        raise TrajectoryValidationError("maximum trajectory horizon is invalid")

    converted = []
    previous_time = -1.0
    seed = tuple(float(value) for value in current_degrees)
    for index, (positions, point_time) in enumerate(
        zip(position_rows, times_from_start_s)
    ):
        time_s = float(point_time)
        if not math.isfinite(time_s) or time_s < 0.0:
            raise TrajectoryValidationError(
                f"trajectory point {index} has an invalid time_from_start"
            )
        if time_s > float(maximum_horizon_s):
            raise TrajectoryValidationError(
                f"trajectory exceeds the {maximum_horizon_s:.1f}s horizon"
            )
        if index > 0 and time_s <= previous_time:
            raise TrajectoryValidationError(
                "trajectory time_from_start values must be strictly increasing"
            )
        previous_time = time_s

        row = tuple(float(value) for value in positions)
        if len(row) != len(names):
            raise TrajectoryValidationError(
                f"trajectory point {index} has {len(row)} positions for "
                f"{len(names)} joints"
            )
        if any(not math.isfinite(value) for value in row):
            raise TrajectoryValidationError(
                f"trajectory point {index} contains a non-finite position"
            )
        try:
            seed = merge_joint_radians(names, row, seed)
        except ValueError as exc:
            raise TrajectoryValidationError(
                f"trajectory point {index} violates the joint contract: {exc}"
            ) from exc
        converted.append(seed)

    return tuple(converted)


def max_target_error_degrees(
    current_degrees: Sequence[float],
    target_degrees: Sequence[float],
    controlled_joint_names: Iterable[str],
) -> float:
    """Return the largest absolute error across the controlled joints."""

    if len(current_degrees) != len(JOINT_NAMES) or len(target_degrees) != len(
        JOINT_NAMES
    ):
        raise ValueError("current and target states must contain six joints")
    indexes = {name: index for index, name in enumerate(JOINT_NAMES)}
    errors = []
    for name in controlled_joint_names:
        if name not in indexes:
            raise ValueError(f"unknown joint name: {name}")
        index = indexes[name]
        current = float(current_degrees[index])
        target = float(target_degrees[index])
        if not math.isfinite(current) or not math.isfinite(target):
            raise ValueError("joint state contains a non-finite value")
        errors.append(abs(current - target))
    return max(errors, default=0.0)


def home_seed() -> tuple[float, ...]:
    """Expose the canonical home only for tests and explicit initialization."""

    return tuple(HOME_DEGREES)


def estimate_motion_timeout_seconds(
    current_degrees: Sequence[float],
    target_degrees: Sequence[float],
    controlled_joint_names: Iterable[str],
    *,
    speed_percent: float,
    maximum_speed_deg_per_second: float,
    scale: float = 1.5,
    margin_s: float = 1.0,
    maximum_timeout_s: float = 30.0,
) -> float:
    """Bound a checkpoint timeout from the Pi's configured slew rate."""

    parameters = (
        speed_percent,
        maximum_speed_deg_per_second,
        scale,
        maximum_timeout_s,
    )
    if any(
        isinstance(value, bool)
        or not math.isfinite(float(value))
        or float(value) <= 0.0
        for value in parameters
    ) or isinstance(margin_s, bool) or not math.isfinite(float(margin_s)) or margin_s < 0.0:
        raise ValueError("motion timeout parameters must be finite and positive")
    if not 1.0 <= float(speed_percent) <= 100.0:
        raise ValueError("speed percent must be in 1..100")
    rate = float(maximum_speed_deg_per_second) * float(speed_percent) / 100.0
    delta = max_target_error_degrees(
        current_degrees, target_degrees, controlled_joint_names
    )
    timeout = delta / rate * float(scale) + float(margin_s)
    if timeout > float(maximum_timeout_s):
        raise ValueError(
            f"required checkpoint timeout {timeout:.2f}s exceeds the safety cap"
        )
    return timeout


def validate_hardware_status_payload(
    payload: Mapping[str, object],
    *,
    previous_sequence: int | None = None,
) -> tuple[int, str, tuple[str, ...], tuple[float, ...]]:
    """Validate one authenticated bridge status and its monotonic ACK sequence."""

    if not isinstance(payload, Mapping):
        raise TrajectoryValidationError("hardware status is not an object")
    sequence = payload.get("sequence")
    if type(sequence) is not int or sequence < 0:
        raise TrajectoryValidationError("hardware status sequence is invalid")
    if previous_sequence is not None and sequence <= previous_sequence:
        raise TrajectoryValidationError("hardware status sequence is not newer")
    code = payload.get("code")
    if not isinstance(code, str) or not code:
        raise TrajectoryValidationError("hardware status code is invalid")
    states_raw = payload.get("state")
    if not isinstance(states_raw, list) or any(
        not isinstance(item, str) or not item for item in states_raw
    ):
        raise TrajectoryValidationError("hardware status state list is invalid")
    if len(set(states_raw)) != len(states_raw):
        raise TrajectoryValidationError("hardware status states contain duplicates")
    angles_raw = payload.get("angles_deg")
    if not isinstance(angles_raw, list) or len(angles_raw) != len(JOINT_NAMES):
        raise TrajectoryValidationError("hardware status must contain six angles")
    try:
        # Status is safety feedback, so it must pass the same canonical logical
        # ranges (including the base joint's 245-degree range) as commands.
        angles = validate_servo_degrees(angles_raw)
    except (TypeError, ValueError) as exc:
        raise TrajectoryValidationError(
            f"hardware status angles violate the joint contract: {exc}"
        ) from exc
    host = payload.get("host")
    port = payload.get("port")
    if not isinstance(host, str) or not host or type(port) is not int or not 1 <= port <= 65535:
        raise TrajectoryValidationError("hardware status endpoint is invalid")
    return sequence, code, tuple(states_raw), angles
