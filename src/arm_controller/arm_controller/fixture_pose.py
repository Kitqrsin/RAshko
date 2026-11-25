"""Metric camera pose estimation for the marked planar cap fixture."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from numbers import Real
from typing import Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class FixturePoseReference:
    """Commissioned camera pose expressed in the fixture frame."""

    position_fixture_m: tuple[float, float, float]
    orientation_fixture_from_camera_xyzw: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        """Reject incomplete, non-finite, or non-unit reference poses."""

        position = _finite_vector(
            self.position_fixture_m,
            3,
            "camera reference position",
        )
        quaternion = _finite_vector(
            self.orientation_fixture_from_camera_xyzw,
            4,
            "camera reference orientation",
        )
        norm = float(np.linalg.norm(quaternion))
        if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1.0e-3):
            raise ValueError("camera reference quaternion must be unit length")
        quaternion = quaternion / norm
        if position[2] <= 0.0:
            raise ValueError("camera reference must be above the fixture plane")
        object.__setattr__(
            self,
            "position_fixture_m",
            tuple(float(value) for value in position),
        )
        object.__setattr__(
            self,
            "orientation_fixture_from_camera_xyzw",
            tuple(float(value) for value in quaternion),
        )


@dataclass(frozen=True)
class FixturePoseEstimate:
    """One PnP estimate of the camera relative to the fixture plane."""

    position_fixture_m: tuple[float, float, float]
    orientation_fixture_from_camera_xyzw: tuple[float, float, float, float]
    camera_optical_axis_fixture: tuple[float, float, float]
    optical_axis_tilt_deg: float
    reprojection_rms_px: float
    minimum_depth_m: float
    reference_translation_error_m: float | None = None
    reference_rotation_error_deg: float | None = None


def _finite_vector(values, length: int, name: str) -> np.ndarray:
    """Return one finite vector with strict scalar typing."""

    if not isinstance(values, (tuple, list, np.ndarray)):
        raise ValueError(f"{name} must contain {length} numbers")
    if len(values) != length:
        raise ValueError(f"{name} must contain {length} numbers")
    if any(isinstance(value, bool) or not isinstance(value, Real) for value in values):
        raise ValueError(f"{name} must contain unquoted finite numbers")
    vector = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} contains a non-finite value")
    return vector


def _as_planar_points(points: Sequence[Sequence[float]]) -> np.ndarray:
    """Validate planar fixture coordinates for PnP."""

    result = np.asarray(points, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 2 or len(result) < 4:
        raise ValueError("fixture pose needs at least four planar points")
    if not np.all(np.isfinite(result)):
        raise ValueError("fixture pose points contain a non-finite value")
    if abs(float(cv2.contourArea(cv2.convexHull(result.astype(np.float32))))) <= 1e-8:
        raise ValueError("fixture pose points are degenerate")
    return result


def _as_image_points(points: Sequence[Sequence[float]], count: int) -> np.ndarray:
    """Validate image coordinates paired with fixture points."""

    result = np.asarray(points, dtype=np.float64)
    if result.shape != (count, 2) or not np.all(np.isfinite(result)):
        raise ValueError("fixture image points must be finite paired XY points")
    return result


def _as_camera_matrix(matrix: Sequence[Sequence[float]]) -> np.ndarray:
    """Validate one pinhole intrinsic matrix."""

    result = np.asarray(matrix, dtype=np.float64)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError("camera matrix must be a finite 3x3 matrix")
    if result[0, 0] <= 0.0 or result[1, 1] <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    if not math.isclose(float(result[2, 2]), 1.0, abs_tol=1e-9):
        raise ValueError("camera matrix has an invalid homogeneous scale")
    return result


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> tuple[float, ...]:
    """Convert a proper rotation matrix to a canonical XYZW quaternion."""

    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    # This eigen decomposition is stable around 180-degree rotations, where
    # trace-based branches are especially easy to implement incorrectly.
    k = np.array(
        [
            [
                matrix[0, 0] - matrix[1, 1] - matrix[2, 2],
                matrix[0, 1] + matrix[1, 0],
                matrix[0, 2] + matrix[2, 0],
                matrix[2, 1] - matrix[1, 2],
            ],
            [
                matrix[0, 1] + matrix[1, 0],
                matrix[1, 1] - matrix[0, 0] - matrix[2, 2],
                matrix[1, 2] + matrix[2, 1],
                matrix[0, 2] - matrix[2, 0],
            ],
            [
                matrix[0, 2] + matrix[2, 0],
                matrix[1, 2] + matrix[2, 1],
                matrix[2, 2] - matrix[0, 0] - matrix[1, 1],
                matrix[1, 0] - matrix[0, 1],
            ],
            [
                matrix[2, 1] - matrix[1, 2],
                matrix[0, 2] - matrix[2, 0],
                matrix[1, 0] - matrix[0, 1],
                matrix[0, 0] + matrix[1, 1] + matrix[2, 2],
            ],
        ],
        dtype=np.float64,
    ) / 3.0
    eigenvalues, eigenvectors = np.linalg.eigh(k)
    quaternion = eigenvectors[:, int(np.argmax(eigenvalues))]
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def quaternion_rotation_distance_deg(
    first_xyzw: Sequence[float],
    second_xyzw: Sequence[float],
) -> float:
    """Return the shortest three-dimensional orientation difference."""

    first = _finite_vector(first_xyzw, 4, "first quaternion")
    second = _finite_vector(second_xyzw, 4, "second quaternion")
    first /= np.linalg.norm(first)
    second /= np.linalg.norm(second)
    cosine = float(np.clip(abs(np.dot(first, second)), 0.0, 1.0))
    return math.degrees(2.0 * math.acos(cosine))


def estimate_fixture_pose(
    plane_points_m: Sequence[Sequence[float]],
    image_points_px: Sequence[Sequence[float]],
    camera_matrix: Sequence[Sequence[float]],
    *,
    max_reprojection_rms_px: float,
) -> FixturePoseEstimate:
    """Estimate camera-in-fixture pose with planar IPPE and fail closed."""

    plane = _as_planar_points(plane_points_m)
    image = _as_image_points(image_points_px, len(plane))
    intrinsics = _as_camera_matrix(camera_matrix)
    if (
        isinstance(max_reprojection_rms_px, bool)
        or not isinstance(max_reprojection_rms_px, Real)
        or not math.isfinite(float(max_reprojection_rms_px))
        or max_reprojection_rms_px <= 0.0
    ):
        raise ValueError("PnP reprojection limit must be a positive number")

    object_points = np.column_stack(
        (plane, np.zeros(len(plane), dtype=np.float64))
    )
    try:
        result = cv2.solvePnPGeneric(
            object_points,
            image,
            intrinsics,
            np.zeros(5, dtype=np.float64),
            flags=cv2.SOLVEPNP_IPPE,
        )
    except cv2.error as exc:
        raise ValueError(f"fixture planar pose estimation failed: {exc}") from exc
    if len(result) < 3 or not result[0]:
        raise ValueError("fixture planar pose estimation produced no solution")

    candidates = []
    for rotation_vector, translation_vector in zip(result[1], result[2]):
        # IPPE resolves the two planar pose branches. Refine each branch over
        # every distributed marker corner before applying the physical and
        # reprojection gates; raw IPPE can retain sub-pixel algebraic error.
        try:
            rotation_vector, translation_vector = cv2.solvePnPRefineLM(
                object_points,
                image,
                intrinsics,
                np.zeros(5, dtype=np.float64),
                np.asarray(rotation_vector, dtype=np.float64).reshape(3, 1),
                np.asarray(translation_vector, dtype=np.float64).reshape(3, 1),
            )
        except cv2.error:
            continue
        rotation_fixture_to_camera, _ = cv2.Rodrigues(rotation_vector)
        translation = np.asarray(translation_vector, dtype=np.float64).reshape(3)
        camera_points = (
            rotation_fixture_to_camera @ object_points.T
        ).T + translation
        minimum_depth = float(np.min(camera_points[:, 2]))
        if minimum_depth <= 0.0:
            continue
        rotation_camera_to_fixture = rotation_fixture_to_camera.T
        camera_position = -rotation_camera_to_fixture @ translation
        if camera_position[2] <= 0.0:
            continue
        projected, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            translation_vector,
            intrinsics,
            np.zeros(5, dtype=np.float64),
        )
        errors = np.linalg.norm(projected.reshape(-1, 2) - image, axis=1)
        rms = float(np.sqrt(np.mean(np.square(errors))))
        optical_axis = rotation_camera_to_fixture @ np.asarray(
            (0.0, 0.0, 1.0), dtype=np.float64
        )
        tilt = math.degrees(
            math.acos(float(np.clip(abs(optical_axis[2]), 0.0, 1.0)))
        )
        candidates.append(
            FixturePoseEstimate(
                position_fixture_m=tuple(float(value) for value in camera_position),
                orientation_fixture_from_camera_xyzw=(
                    rotation_matrix_to_quaternion_xyzw(
                        rotation_camera_to_fixture
                    )
                ),
                camera_optical_axis_fixture=tuple(
                    float(value) for value in optical_axis
                ),
                optical_axis_tilt_deg=tilt,
                reprojection_rms_px=rms,
                minimum_depth_m=minimum_depth,
            )
        )
    if not candidates:
        raise ValueError("fixture pose has no physically valid camera-above-plane solution")
    estimate = min(candidates, key=lambda item: item.reprojection_rms_px)
    if estimate.reprojection_rms_px > max_reprojection_rms_px:
        raise ValueError(
            "fixture PnP reprojection RMS "
            f"{estimate.reprojection_rms_px:.2f}px exceeds limit"
        )
    return estimate


def compare_fixture_pose_to_reference(
    estimate: FixturePoseEstimate,
    reference: FixturePoseReference,
    *,
    max_translation_error_m: float,
    max_rotation_error_deg: float,
    enforce_limits: bool = True,
) -> FixturePoseEstimate:
    """Attach reference errors and reject a displaced observation camera."""

    translation_error = float(
        np.linalg.norm(
            np.asarray(estimate.position_fixture_m)
            - np.asarray(reference.position_fixture_m)
        )
    )
    rotation_error = quaternion_rotation_distance_deg(
        estimate.orientation_fixture_from_camera_xyzw,
        reference.orientation_fixture_from_camera_xyzw,
    )
    result = replace(
        estimate,
        reference_translation_error_m=translation_error,
        reference_rotation_error_deg=rotation_error,
    )
    if enforce_limits and translation_error > max_translation_error_m:
        raise ValueError(
            "camera pose translation differs from its commissioned reference "
            f"by {translation_error * 1000.0:.2f}mm"
        )
    if enforce_limits and rotation_error > max_rotation_error_deg:
        raise ValueError(
            "camera pose rotation differs from its commissioned reference "
            f"by {rotation_error:.3f}deg"
        )
    return result
