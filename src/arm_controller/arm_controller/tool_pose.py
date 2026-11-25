"""Orientation contract for the physical gripper approach axis."""

from __future__ import annotations

import math
from collections.abc import Sequence


TOOL_LINK = "tool_tip"
ARM_LINKS = (
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_link",
    "gripper_base",
    TOOL_LINK,
)


def tool_quaternion(
    azimuth: float,
    elevation: float,
    roll: float = 0.0,
) -> tuple[float, float, float, float]:
    """Return XYZW orientation for the gripper's physical approach direction.

    The CAD model extends from ``gripper_base`` along local -Z.  Elevation is
    therefore expressed for local -Z in the base frame: +90 degrees is up,
    zero is horizontal/outward, and -90 degrees is down.  Roll rotates the
    gripper about that approach direction without changing it.
    """
    cos_elevation = math.cos(elevation)
    direction = (
        cos_elevation * math.cos(azimuth),
        cos_elevation * math.sin(azimuth),
        math.sin(elevation),
    )

    # Rotation-matrix columns are the tool frame axes in the base frame.
    z_axis = tuple(-value for value in direction)
    y_axis = (-math.sin(azimuth), math.cos(azimuth), 0.0)
    x_axis = _cross(y_axis, z_axis)

    cos_roll = math.cos(roll)
    sin_roll = math.sin(roll)
    rolled_x = tuple(
        cos_roll * x_axis[index] + sin_roll * y_axis[index]
        for index in range(3)
    )
    rolled_y = tuple(
        -sin_roll * x_axis[index] + cos_roll * y_axis[index]
        for index in range(3)
    )

    matrix = (
        (rolled_x[0], rolled_y[0], z_axis[0]),
        (rolled_x[1], rolled_y[1], z_axis[1]),
        (rolled_x[2], rolled_y[2], z_axis[2]),
    )
    return _matrix_to_quaternion(matrix)


def tool_direction(quaternion: Sequence[float]) -> tuple[float, float, float]:
    """Return the base-frame direction of the tool's local -Z axis."""
    x, y, z, w = (float(value) for value in quaternion)
    return (
        -2.0 * (x * z + w * y),
        -2.0 * (y * z - w * x),
        -(1.0 - 2.0 * (x * x + y * y)),
    )


def tool_elevation(quaternion: Sequence[float]) -> float:
    """Return physical tool elevation in radians from an XYZW quaternion."""
    return math.asin(max(-1.0, min(1.0, tool_direction(quaternion)[2])))


def _cross(
    left: Sequence[float],
    right: Sequence[float],
) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _matrix_to_quaternion(
    matrix: Sequence[Sequence[float]],
) -> tuple[float, float, float, float]:
    """Convert a proper 3x3 rotation matrix to a normalized XYZW quaternion."""
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m21 - m12) / scale
        y = (m02 - m20) / scale
        z = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / scale
        x = 0.25 * scale
        y = (m01 + m10) / scale
        z = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / scale
        x = (m01 + m10) / scale
        y = 0.25 * scale
        z = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / scale
        x = (m02 + m20) / scale
        y = (m12 + m21) / scale
        z = 0.25 * scale

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)
