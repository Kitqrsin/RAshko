"""Canonical mapping between ROS/URDF radians and logical servo degrees."""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence


# LLM HANDOFF: This is the single source of truth shared by simulation,
# trajectory execution, the Windows UDP bridge, and the Raspberry Pi config.
JOINT_NAMES = (
    "base",
    "shoulder",
    "elbow",
    "wrist",
    "wrist_rotate",
    "gripper_base",
)
SERVO_MIN_DEGREES = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
SERVO_MAX_DEGREES = (245.0, 180.0, 180.0, 180.0, 180.0, 180.0)
INVERTED_URDF_INDICES = frozenset((1, 2, 3))
HOME_DEGREES = (90.0, 88.0, 90.0, 96.0, 98.0, 0.0)
GRIPPER_OPEN_DEGREES = HOME_DEGREES[5]

# Trigonometric/radian conversions can produce values such as
# ``-1.4e-14`` at an exact 0-degree joint limit.  Snap only that numerical
# round-off to the boundary; real out-of-range commands are still rejected.
SERVO_LIMIT_EPSILON_DEGREES = 1e-6


def _normalize_limit_roundoff(index: int, angle: float) -> float:
    minimum = SERVO_MIN_DEGREES[index]
    maximum = SERVO_MAX_DEGREES[index]
    if minimum - SERVO_LIMIT_EPSILON_DEGREES <= angle < minimum:
        return minimum
    if maximum < angle <= maximum + SERVO_LIMIT_EPSILON_DEGREES:
        return maximum
    return angle


def validate_servo_degrees(values: Iterable[float]) -> tuple[float, ...]:
    angles = tuple(float(value) for value in values)
    if len(angles) != len(JOINT_NAMES):
        raise ValueError(f"expected {len(JOINT_NAMES)} angles, got {len(angles)}")
    normalized = []
    for index, angle in enumerate(angles):
        if not math.isfinite(angle):
            raise ValueError(f"{JOINT_NAMES[index]} is not finite")
        angle = _normalize_limit_roundoff(index, angle)
        if not SERVO_MIN_DEGREES[index] <= angle <= SERVO_MAX_DEGREES[index]:
            raise ValueError(
                f"{JOINT_NAMES[index]}={angle:.3f} is outside "
                f"{SERVO_MIN_DEGREES[index]:.1f}..{SERVO_MAX_DEGREES[index]:.1f} degrees"
            )
        normalized.append(angle)
    return tuple(normalized)


def servo_degrees_to_radians(values: Iterable[float]) -> tuple[float, ...]:
    angles = validate_servo_degrees(values)
    result = []
    for index, angle in enumerate(angles):
        radians = math.radians(angle - HOME_DEGREES[index])
        result.append(-radians if index in INVERTED_URDF_INDICES else radians)
    return tuple(result)


def radians_to_servo_degrees(index: int, radians: float) -> float:
    offset = math.degrees(float(radians))
    home = HOME_DEGREES[index]
    angle = home - offset if index in INVERTED_URDF_INDICES else home + offset
    return _normalize_limit_roundoff(index, angle)


def merge_joint_radians(
    names: Sequence[str],
    positions: Sequence[float],
    current_degrees: Sequence[float] = HOME_DEGREES,
) -> tuple[float, ...]:
    """Merge a named, possibly partial ROS joint vector into six servo angles."""
    if len(names) != len(positions):
        raise ValueError("joint name and position counts differ")
    result = list(validate_servo_degrees(current_degrees))
    indexes = {name: index for index, name in enumerate(JOINT_NAMES)}
    for name, position in zip(names, positions):
        if name not in indexes:
            continue
        index = indexes[name]
        angle = radians_to_servo_degrees(index, position)
        if not SERVO_MIN_DEGREES[index] <= angle <= SERVO_MAX_DEGREES[index]:
            raise ValueError(f"trajectory places {name} outside its servo range: {angle:.3f} degrees")
        result[index] = angle
    return tuple(result)
