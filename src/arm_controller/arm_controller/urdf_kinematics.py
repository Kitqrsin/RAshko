"""Exact URDF forward kinematics and bounded numerical Cartesian IK."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from arm_controller.joint_contract import JOINT_NAMES


ARM_JOINT_NAMES = JOINT_NAMES[:5]
TOOL_DIRECTION_SCALE = 0.10


@dataclass(frozen=True)
class ForwardResult:
    position: np.ndarray
    rotation: np.ndarray
    link_positions: tuple[tuple[float, float, float], ...]
    joint_positions: tuple[tuple[float, float, float], ...]

    @property
    def tool_direction(self) -> np.ndarray:
        """Physical gripper direction; the CAD gripper extends along local -Z."""
        return -self.rotation[:, 2]


@dataclass(frozen=True)
class IKSolution:
    radians: tuple[float, ...]
    position_error: float
    direction_error: float


@dataclass(frozen=True)
class _Joint:
    name: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


def _chain_elements(robot_description, base_link, tip_link):
    root = ET.fromstring(robot_description)
    child_to_joint = {
        joint.find("child").attrib["link"]: joint for joint in root.findall("joint")
    }
    elements = []
    current_link = tip_link
    while current_link != base_link:
        if current_link not in child_to_joint:
            raise ValueError(f"URDF has no joint chain from {base_link} to {tip_link}")
        joint_element = child_to_joint[current_link]
        elements.append(joint_element)
        current_link = joint_element.find("parent").attrib["link"]
    return list(reversed(elements))


def compact_tool_chain(robot_description: str) -> str:
    """Extract joint transforms and limits for the GUI's lightweight schematic."""
    chain = ET.Element("robot", name="cartesian_chain")
    chain.extend(_chain_elements(robot_description, "base_link", "tool_tip"))
    return ET.tostring(chain, encoding="unicode")


class UrdfKinematics:
    """Kinematics for the exact ``base_link`` to ``tool_tip`` URDF chain."""

    def __init__(
        self,
        robot_description: str,
        base_link: str = "base_link",
        tip_link: str = "tool_tip",
    ):
        self._segments = []
        active_names = []
        for element in _chain_elements(robot_description, base_link, tip_link):
            name = element.attrib["name"]
            joint_type = element.attrib.get("type", "fixed")
            origin_element = element.find("origin")
            xyz = _vector_attribute(origin_element, "xyz")
            rpy = _vector_attribute(origin_element, "rpy")
            origin = _origin_matrix(xyz, rpy)

            if joint_type == "fixed":
                self._segments.append((name, origin, None))
                continue
            if joint_type not in ("revolute", "continuous"):
                raise ValueError(f"unsupported arm joint type {joint_type!r}")

            axis_element = element.find("axis")
            axis = _vector_attribute(axis_element, "xyz", default=(1.0, 0.0, 0.0))
            axis_array = np.asarray(axis, dtype=float)
            axis_array /= np.linalg.norm(axis_array)
            limit_element = element.find("limit")
            lower = float(limit_element.attrib.get("lower", -math.pi))
            upper = float(limit_element.attrib.get("upper", math.pi))
            joint = _Joint(name, origin, axis_array, lower, upper)
            self._segments.append((name, origin, joint))
            active_names.append(name)

        if tuple(active_names) != tuple(ARM_JOINT_NAMES):
            raise ValueError(
                f"unexpected URDF arm chain {active_names}; expected {ARM_JOINT_NAMES}"
            )
        self.lower_limits = np.asarray(
            [segment[2].lower for segment in self._segments if segment[2]],
            dtype=float,
        )
        self.upper_limits = np.asarray(
            [segment[2].upper for segment in self._segments if segment[2]],
            dtype=float,
        )

    def forward(self, radians: Sequence[float]) -> ForwardResult:
        values = np.asarray(radians, dtype=float)
        if values.shape != (len(ARM_JOINT_NAMES),):
            raise ValueError(f"expected {len(ARM_JOINT_NAMES)} joint radians")

        transform = np.eye(4, dtype=float)
        positions = [(0.0, 0.0, 0.0)]
        joint_positions = []
        active_index = 0
        for _, origin, joint in self._segments:
            transform = transform @ origin
            if joint is not None:
                joint_positions.append(tuple(float(value) for value in transform[:3, 3]))
                transform = transform @ _axis_rotation(joint.axis, values[active_index])
                active_index += 1
            positions.append(tuple(float(value) for value in transform[:3, 3]))

        return ForwardResult(
            position=transform[:3, 3].copy(),
            rotation=transform[:3, :3].copy(),
            link_positions=tuple(positions),
            joint_positions=tuple(joint_positions),
        )

    def solve(
        self,
        target_position: Sequence[float],
        tool_elevation: Optional[float],
        seed_radians: Sequence[float],
        wrist_roll: float = 0.0,
        max_iterations: int = 120,
    ) -> Optional[IKSolution]:
        """Solve a tool-tip position, optionally with physical tool elevation.

        The first four joints position and aim the gripper. ``wrist_rotate`` is
        kept at the requested roll because it rotates around the tool direction.
        Passing ``None`` for ``tool_elevation`` leaves pitch free, which gives
        mouse point-following the arm's full position workspace. Supplying an
        elevation retains strict pose control for tasks that require it.
        Multiple deterministic starts avoid getting trapped in the folded-arm
        local minimum that caused the old Cartesian solver to flip the gripper.
        """
        target = np.asarray(target_position, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("target_position must contain three finite values")

        desired_direction = None
        if tool_elevation is not None:
            elevation = float(tool_elevation)
            if not math.isfinite(elevation):
                raise ValueError("tool_elevation must be finite or None")
            azimuth = math.atan2(float(target[1]), float(target[0]))
            desired_direction = np.asarray(
                (
                    math.cos(elevation) * math.cos(azimuth),
                    math.cos(elevation) * math.sin(azimuth),
                    math.sin(elevation),
                ),
                dtype=float,
            )
        seed = np.clip(
            np.asarray(seed_radians, dtype=float),
            self.lower_limits,
            self.upper_limits,
        )
        if seed.shape != (len(ARM_JOINT_NAMES),):
            raise ValueError(f"expected {len(ARM_JOINT_NAMES)} seed radians")
        seed[4] = np.clip(wrist_roll, self.lower_limits[4], self.upper_limits[4])

        starts = [seed]
        for candidate in (
            (0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, math.pi / 2.0, -math.pi / 2.0),
            (0.0, 0.0, -math.pi / 2.0, math.pi / 2.0),
            (0.0, math.pi / 2.0, 0.0, -math.pi / 2.0),
            (0.0, -math.pi / 2.0, 0.0, math.pi / 2.0),
            (0.0, 0.0, math.pi / 2.0, math.pi / 2.0),
            (0.0, math.pi / 2.0, math.pi / 2.0, -math.pi / 2.0),
        ):
            start = np.asarray((*candidate, seed[4]), dtype=float)
            starts.append(np.clip(start, self.lower_limits, self.upper_limits))

        best = None
        best_score = float("inf")
        for start in starts:
            candidate = self._solve_from_start(
                target,
                desired_direction,
                start,
                seed,
                max_iterations,
            )
            forward = self.forward(candidate)
            position_error = float(np.linalg.norm(target - forward.position))
            direction_error = (
                0.0
                if desired_direction is None
                else _angle_between(desired_direction, forward.tool_direction)
            )
            jump = float(np.linalg.norm(candidate[:4] - seed[:4]))
            score = position_error + 0.0005 * jump
            if desired_direction is not None:
                score += 0.10 * direction_error
            if score < best_score:
                best_score = score
                best = IKSolution(
                    tuple(float(value) for value in candidate),
                    position_error,
                    direction_error,
                )

        if (
            best is not None
            and best.position_error <= 0.008
            and (
                desired_direction is None
                or best.direction_error <= math.radians(6.0)
            )
        ):
            return best
        return None

    def _solve_from_start(
        self,
        target_position: np.ndarray,
        target_direction: Optional[np.ndarray],
        start: np.ndarray,
        seed: np.ndarray,
        max_iterations: int,
    ) -> np.ndarray:
        values = start.copy()
        values[4] = seed[4]
        epsilon = 1e-4
        damping = 2e-3
        regularization = 2e-5

        for _ in range(max_iterations):
            include_direction = target_direction is not None
            current = self._task_vector(values, include_direction)
            desired = target_position
            if include_direction:
                desired = np.concatenate(
                    (target_position, TOOL_DIRECTION_SCALE * target_direction)
                )
            residual = desired - current
            position_tolerance = 0.0015 if include_direction else 0.0003
            if np.linalg.norm(residual[:3]) < position_tolerance:
                if not include_direction:
                    break
                current_direction = current[3:] / TOOL_DIRECTION_SCALE
                if _angle_between(current_direction, target_direction) < math.radians(1.5):
                    break

            jacobian = np.zeros((len(desired), len(values)), dtype=float)
            for index in range(4):
                plus = values.copy()
                minus = values.copy()
                plus[index] = min(self.upper_limits[index], plus[index] + epsilon)
                minus[index] = max(self.lower_limits[index], minus[index] - epsilon)
                denominator = plus[index] - minus[index]
                if denominator > 0.0:
                    jacobian[:, index] = (
                        self._task_vector(plus, include_direction)
                        - self._task_vector(minus, include_direction)
                    ) / denominator

            normal = jacobian.T @ jacobian
            normal += (damping * damping + regularization) * np.eye(len(values))
            right = jacobian.T @ residual + regularization * (seed - values)
            try:
                delta = np.linalg.solve(normal, right)
            except np.linalg.LinAlgError:
                break
            delta[4] = 0.0
            delta_norm = float(np.linalg.norm(delta[:4]))
            if delta_norm > 0.25:
                delta *= 0.25 / delta_norm

            current_cost = float(residual @ residual)
            accepted = False
            for scale in (1.0, 0.5, 0.25, 0.1):
                trial = np.clip(
                    values + scale * delta,
                    self.lower_limits,
                    self.upper_limits,
                )
                trial[4] = seed[4]
                trial_residual = desired - self._task_vector(trial, include_direction)
                if float(trial_residual @ trial_residual) < current_cost:
                    values = trial
                    accepted = True
                    break
            if not accepted:
                break

        return values

    def _task_vector(
        self, radians: np.ndarray, include_direction: bool = True
    ) -> np.ndarray:
        forward = self.forward(radians)
        if not include_direction:
            return forward.position
        return np.concatenate(
            (forward.position, TOOL_DIRECTION_SCALE * forward.tool_direction)
        )


def _vector_attribute(element, name, default=(0.0, 0.0, 0.0)):
    if element is None or name not in element.attrib:
        return default
    return tuple(float(value) for value in element.attrib[name].split())


def _origin_matrix(xyz, rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=float,
    )
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = xyz
    return transform


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus = 1.0 - cosine
    rotation = np.asarray(
        (
            (cosine + x * x * one_minus, x * y * one_minus - z * sine, x * z * one_minus + y * sine),
            (y * x * one_minus + z * sine, cosine + y * y * one_minus, y * z * one_minus - x * sine),
            (z * x * one_minus - y * sine, z * y * one_minus + x * sine, cosine + z * z * one_minus),
        ),
        dtype=float,
    )
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    return transform


def _angle_between(left: np.ndarray, right: np.ndarray) -> float:
    cosine = float(np.dot(left, right))
    cosine /= float(np.linalg.norm(left) * np.linalg.norm(right))
    return math.acos(max(-1.0, min(1.0, cosine)))
