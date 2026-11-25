"""Fail-closed red-cap detection and image-based XY alignment primitives."""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import math
from numbers import Integral, Real
from typing import Mapping, Sequence

import cv2
import numpy as np


class VisualServoError(ValueError):
    """Raised when a camera observation cannot safely guide alignment."""


@dataclass(frozen=True)
class RedBlob:
    """One geometrically validated red component in camera pixels."""

    center_px: tuple[float, float]
    area_px2: float
    equivalent_radius_px: float
    circularity: float
    solidity: float


@dataclass(frozen=True)
class RedBlobDetectorConfig:
    """Conservative close-up red-cap segmentation thresholds."""

    min_area_px2: float = 80.0
    # The current close gripper-camera view renders an 8.3 mm cap at roughly
    # 5,400..5,900 px2. Keep margin for distance variation while circularity and
    # solidity continue rejecting broad red background regions.
    max_area_px2: float = 12000.0
    min_circularity: float = 0.55
    min_solidity: float = 0.85
    morphology_radius_px: int = 2
    red_low_1: tuple[int, int, int] = (0, 110, 80)
    red_high_1: tuple[int, int, int] = (12, 255, 255)
    red_low_2: tuple[int, int, int] = (168, 110, 80)
    red_high_2: tuple[int, int, int] = (180, 255, 255)

    def __post_init__(self) -> None:
        """Reject thresholds which could silently accept arbitrary red noise."""

        values = (
            self.min_area_px2,
            self.max_area_px2,
            self.min_circularity,
            self.min_solidity,
        )
        if any(
            isinstance(value, bool)
            or not isinstance(value, Real)
            or not math.isfinite(float(value))
            for value in values
        ):
            raise ValueError("red-blob thresholds must be finite numbers")
        if not 0.0 < self.min_area_px2 < self.max_area_px2:
            raise ValueError("red-blob area limits are invalid")
        if not 0.50 <= self.min_circularity <= 1.0:
            raise ValueError("red-blob circularity must be in [0.50, 1.00]")
        if not 0.70 <= self.min_solidity <= 1.0:
            raise ValueError("red-blob solidity must be in [0.70, 1.00]")
        if (
            isinstance(self.morphology_radius_px, bool)
            or not isinstance(self.morphology_radius_px, Integral)
            or not 1 <= self.morphology_radius_px <= 8
        ):
            raise ValueError("morphology_radius_px must be an integer in [1, 8]")


@dataclass(frozen=True)
class FixedJigReference:
    """A fixed fixture-plane to undistorted-image homography."""

    plane_to_image: tuple[float, ...]
    image_size: tuple[int, int]

    def __post_init__(self) -> None:
        """Require a finite invertible homography and exact image size."""

        if len(self.plane_to_image) != 9:
            raise ValueError("fixed-jig homography must contain nine values")
        matrix = np.asarray(self.plane_to_image, dtype=np.float64).reshape(3, 3)
        if not np.all(np.isfinite(matrix)):
            raise ValueError("fixed-jig homography contains a non-finite value")
        if abs(float(np.linalg.det(matrix))) <= 1e-12:
            raise ValueError("fixed-jig homography is singular")
        if abs(float(matrix[2, 2])) <= 1e-12:
            raise ValueError("fixed-jig homography cannot be normalized")
        if (
            len(self.image_size) != 2
            or any(type(value) is not int or value <= 0 for value in self.image_size)
        ):
            raise ValueError("fixed-jig image size must contain positive integers")

    @property
    def matrix(self) -> np.ndarray:
        """Return the normalized 3-by-3 homography."""

        matrix = np.asarray(self.plane_to_image, dtype=np.float64).reshape(3, 3)
        return matrix / matrix[2, 2]


@dataclass(frozen=True)
class FixedJigCandidate:
    """One seven-cap-derived fixed-jig calibration candidate."""

    reference: FixedJigReference
    slot_pixels: tuple[tuple[str, tuple[float, float]], ...]
    reprojection_rms_px: float
    second_best_rms_px: float


@dataclass(frozen=True)
class FixedCapRowCandidate:
    """Ordered one-dimensional reference for seven collinear cap centres."""

    slot_pixels: tuple[tuple[str, tuple[float, float]], ...]
    image_size: tuple[int, int]
    row_direction_px: tuple[float, float]
    row_straightness_rms_px: float
    minimum_neighbor_spacing_px: float


@dataclass(frozen=True)
class VisualServoConfig:
    """Commissioned mapping from close-up pixel error to tool XY correction."""

    target_pixel: tuple[float, float]
    correction_matrix_m_per_px: tuple[float, float, float, float]
    max_step_m: float = 0.003
    centered_tolerance_px: float = 2.0
    acquisition_radius_px: float = 100.0
    ambiguity_margin_px: float = 25.0
    required_stable_frames: int = 3
    max_iterations: int = 12

    def __post_init__(self) -> None:
        """Reject an unbounded, singular, or weak alignment configuration."""

        target = np.asarray(self.target_pixel, dtype=np.float64)
        mapping = np.asarray(
            self.correction_matrix_m_per_px, dtype=np.float64
        ).reshape(2, 2)
        scalars = (
            self.max_step_m,
            self.centered_tolerance_px,
            self.acquisition_radius_px,
            self.ambiguity_margin_px,
        )
        if not np.all(np.isfinite(target)) or not np.all(np.isfinite(mapping)):
            raise ValueError("visual-servo target and correction matrix must be finite")
        if abs(float(np.linalg.det(mapping))) <= 1e-12:
            raise ValueError("visual-servo correction matrix must be invertible")
        if any(
            isinstance(value, bool)
            or not isinstance(value, Real)
            or not math.isfinite(float(value))
            or value <= 0.0
            for value in scalars
        ):
            raise ValueError("visual-servo limits must be positive finite numbers")
        if not 0.0001 <= self.max_step_m <= 0.010:
            raise ValueError("visual-servo max step must be in [0.1, 10] mm")
        if self.centered_tolerance_px > 5.0:
            raise ValueError("visual-servo centering tolerance cannot exceed 5 pixels")
        for name in ("required_stable_frames", "max_iterations"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Integral):
                raise ValueError(f"{name} must be an integer")
        if self.required_stable_frames < 2:
            raise ValueError("at least two centered frames are required")
        if self.max_iterations < self.required_stable_frames:
            raise ValueError("max_iterations cannot be below the stable-frame count")


@dataclass(frozen=True)
class VisualServoCommand:
    """One bounded XY correction decision from a close-up observation."""

    error_px: tuple[float, float]
    correction_xy_m: tuple[float, float]
    centered: bool
    alignment_verified: bool
    iteration: int


def detect_red_blobs(
    image_bgr: np.ndarray,
    config: RedBlobDetectorConfig | None = None,
) -> tuple[RedBlob, ...]:
    """Detect only finite, circular, solid red components in a BGR frame."""

    settings = config or RedBlobDetectorConfig()
    if (
        not isinstance(image_bgr, np.ndarray)
        or image_bgr.dtype != np.uint8
        or image_bgr.ndim != 3
        or image_bgr.shape[2] != 3
        or min(image_bgr.shape[:2]) < 4
    ):
        raise VisualServoError("red-cap detector needs a non-empty uint8 BGR image")
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, settings.red_low_1, settings.red_high_1)
    mask |= cv2.inRange(hsv, settings.red_low_2, settings.red_high_2)
    radius = int(settings.morphology_radius_px)
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1)
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    blobs = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not settings.min_area_px2 <= area <= settings.max_area_px2:
            continue
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        solidity = 0.0 if hull_area <= 0.0 else area / hull_area
        if circularity < settings.min_circularity or solidity < settings.min_solidity:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] <= 0.0:
            continue
        center = (
            float(moments["m10"] / moments["m00"]),
            float(moments["m01"] / moments["m00"]),
        )
        blobs.append(
            RedBlob(
                center_px=center,
                area_px2=area,
                equivalent_radius_px=math.sqrt(area / math.pi),
                circularity=circularity,
                solidity=solidity,
            )
        )
    return tuple(sorted(blobs, key=lambda item: item.center_px))


def select_alignment_blob(
    blobs: Sequence[RedBlob],
    target_pixel: Sequence[float],
    *,
    acquisition_radius_px: float,
    ambiguity_margin_px: float,
) -> RedBlob:
    """Select one nearby cap while rejecting loss and neighbor ambiguity."""

    target = np.asarray(target_pixel, dtype=np.float64)
    ranked = sorted(
        [
            (
                float(np.linalg.norm(np.asarray(blob.center_px) - target)),
                blob,
            )
            for blob in blobs
        ],
        key=lambda item: item[0],
    )
    if not ranked or ranked[0][0] > acquisition_radius_px:
        raise VisualServoError("no validated red cap is inside the acquisition ROI")
    if len(ranked) > 1 and ranked[1][0] - ranked[0][0] < ambiguity_margin_px:
        raise VisualServoError("two red caps are ambiguous inside the acquisition ROI")
    return ranked[0][1]


def derive_fixed_jig_reference(
    slot_centers_m: Mapping[str, Sequence[float]],
    blobs: Sequence[RedBlob],
    image_size: tuple[int, int],
    *,
    max_reprojection_rms_px: float = 2.0,
    minimum_second_best_gap_px: float = 1.0,
    require_fixture_x_right_y_up: bool = False,
) -> FixedJigCandidate:
    """Assign exactly seven red caps and derive a fixed observation homography.

    The optional axis convention is suitable only for a keyed mechanical jig:
    fixture +X must point toward image right and fixture +Y toward image up.
    It removes mirror-equivalent assignments without pretending the red pattern
    alone has an orientation.
    """

    if len(slot_centers_m) != 7 or len(blobs) != 7:
        raise VisualServoError("fixed-jig teaching requires exactly seven caps")
    slot_ids = tuple(sorted(slot_centers_m))
    plane = np.asarray([slot_centers_m[item] for item in slot_ids], dtype=np.float64)
    if plane.shape != (7, 2) or not np.all(np.isfinite(plane)):
        raise VisualServoError("fixed-jig slot coordinates must be seven finite XY points")
    image_points = tuple(np.asarray(item.center_px, dtype=np.float64) for item in blobs)
    scored = []
    for permutation in itertools.permutations(image_points):
        image = np.asarray(permutation, dtype=np.float64)
        homography, _ = cv2.findHomography(plane, image, method=0)
        if homography is None or not np.all(np.isfinite(homography)):
            continue
        if require_fixture_x_right_y_up and not _fixture_axes_match_image(
            homography, plane
        ):
            continue
        projected = cv2.perspectiveTransform(
            plane.astype(np.float64).reshape(-1, 1, 2), homography
        ).reshape(-1, 2)
        rms = float(
            np.sqrt(np.mean(np.square(np.linalg.norm(projected - image, axis=1))))
        )
        scored.append((rms, homography, image))
    if len(scored) < 2:
        raise VisualServoError("fixed-jig pattern produced too few pose candidates")
    scored.sort(key=lambda item: item[0])
    best, second = scored[0], scored[1]
    if best[0] > max_reprojection_rms_px:
        raise VisualServoError(
            f"fixed-jig red pattern RMS {best[0]:.2f}px exceeds limit"
        )
    if second[0] - best[0] < minimum_second_best_gap_px:
        raise VisualServoError(
            "fixed-jig red pattern is geometrically ambiguous; use a keyed jig"
        )
    normalized = best[1] / best[1][2, 2]
    reference = FixedJigReference(
        tuple(float(value) for value in normalized.reshape(-1)),
        image_size,
    )
    return FixedJigCandidate(
        reference=reference,
        slot_pixels=tuple(
            (slot_id, tuple(float(value) for value in point))
            for slot_id, point in zip(slot_ids, best[2])
        ),
        reprojection_rms_px=best[0],
        second_best_rms_px=second[0],
    )


def derive_fixed_cap_row_candidate(
    slot_centers_m: Mapping[str, Sequence[float]],
    blobs: Sequence[RedBlob],
    image_size: tuple[int, int],
    *,
    max_row_straightness_rms_px: float = 2.0,
) -> FixedCapRowCandidate:
    """Order a seven-cap row without claiming a two-dimensional pose."""

    if len(slot_centers_m) != 7 or len(blobs) != 7:
        raise VisualServoError("fixed-row teaching requires exactly seven caps")
    if (
        len(image_size) != 2
        or any(type(value) is not int or value <= 0 for value in image_size)
    ):
        raise VisualServoError("fixed-row image size must be positive integers")
    slot_ids = tuple(sorted(slot_centers_m))
    plane = np.asarray([slot_centers_m[item] for item in slot_ids], dtype=np.float64)
    if plane.shape != (7, 2) or not np.all(np.isfinite(plane)):
        raise VisualServoError("fixed-row slot coordinates must be finite XY")
    plane_centered = plane - np.mean(plane, axis=0)
    _u, singular, plane_axes = np.linalg.svd(plane_centered, full_matrices=False)
    if singular[0] <= 0.0 or singular[1] > singular[0] * 1e-3:
        raise VisualServoError("configured cap slots are not one straight row")
    plane_axis = plane_axes[0]
    if plane_axis[0] < 0.0:
        plane_axis = -plane_axis
    if plane_axis[0] < math.sqrt(0.5):
        raise VisualServoError("configured cap row must follow fixture +X")
    plane_coordinate = plane_centered @ plane_axis
    plane_order = np.argsort(plane_coordinate)

    image = np.asarray([item.center_px for item in blobs], dtype=np.float64)
    image_center = np.mean(image, axis=0)
    image_centered = image - image_center
    covariance = image_centered.T @ image_centered
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    row_axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    if row_axis[0] < 0.0:
        row_axis = -row_axis
    if row_axis[0] < math.sqrt(0.5):
        raise VisualServoError(
            "cap row must run left-to-right in the commissioned image"
        )
    along = image_centered @ row_axis
    perpendicular_axis = np.asarray((-row_axis[1], row_axis[0]))
    perpendicular = image_centered @ perpendicular_axis
    straightness = float(np.sqrt(np.mean(np.square(perpendicular))))
    if straightness > max_row_straightness_rms_px:
        raise VisualServoError(
            f"cap row straightness RMS {straightness:.2f}px exceeds limit"
        )
    image_order = np.argsort(along)
    ordered_along = along[image_order]
    spacings = np.diff(ordered_along)
    if np.any(spacings <= 0.0):
        raise VisualServoError("cap row pixel order is not unique")
    ordered_slot_ids = [slot_ids[index] for index in plane_order]
    ordered_image = image[image_order]
    return FixedCapRowCandidate(
        slot_pixels=tuple(
            (slot_id, (float(point[0]), float(point[1])))
            for slot_id, point in zip(ordered_slot_ids, ordered_image)
        ),
        image_size=image_size,
        row_direction_px=(float(row_axis[0]), float(row_axis[1])),
        row_straightness_rms_px=straightness,
        minimum_neighbor_spacing_px=float(np.min(spacings)),
    )


def _fixture_axes_match_image(
    homography: np.ndarray, plane_points: np.ndarray
) -> bool:
    """Check a coarse, explicit fixture-X/right and fixture-Y/up convention."""

    minimum = np.min(plane_points, axis=0)
    maximum = np.max(plane_points, axis=0)
    extent = maximum - minimum
    if np.any(extent <= 0.0):
        return False
    center = (minimum + maximum) / 2.0
    probes = np.asarray(
        (
            center,
            center + (extent[0] * 0.1, 0.0),
            center + (0.0, extent[1] * 0.1),
        ),
        dtype=np.float64,
    )
    projected = cv2.perspectiveTransform(
        probes.reshape(-1, 1, 2), homography
    ).reshape(-1, 2)
    image_x = projected[1] - projected[0]
    image_y = projected[2] - projected[0]
    norm_x = float(np.linalg.norm(image_x))
    norm_y = float(np.linalg.norm(image_y))
    if min(norm_x, norm_y) <= 1e-9:
        return False
    return (
        float(image_x[0] / norm_x) >= math.sqrt(0.5)
        and float(-image_y[1] / norm_y) >= math.sqrt(0.5)
    )


class VisualServoController:
    """Bound and stabilize image-based XY corrections; never command Z."""

    def __init__(self, config: VisualServoConfig) -> None:
        self.config = config
        self.iteration = 0
        self.centered_frames = 0
        self.failed = False

    def observe(self, blob: RedBlob) -> VisualServoCommand:
        """Convert one selected cap centroid into a bounded XY correction."""

        if self.failed:
            raise VisualServoError("visual-servo controller is failure-latched")
        if self.iteration >= self.config.max_iterations:
            self.failed = True
            raise VisualServoError("visual-servo iteration limit was exceeded")
        self.iteration += 1
        error = np.asarray(blob.center_px) - np.asarray(self.config.target_pixel)
        error_norm = float(np.linalg.norm(error))
        centered = error_norm <= self.config.centered_tolerance_px
        if centered:
            self.centered_frames += 1
            correction = np.zeros(2, dtype=np.float64)
        else:
            self.centered_frames = 0
            mapping = np.asarray(
                self.config.correction_matrix_m_per_px, dtype=np.float64
            ).reshape(2, 2)
            correction = mapping @ error
            norm = float(np.linalg.norm(correction))
            if norm > self.config.max_step_m:
                correction *= self.config.max_step_m / norm
        verified = self.centered_frames >= self.config.required_stable_frames
        return VisualServoCommand(
            error_px=tuple(float(value) for value in error),
            correction_xy_m=tuple(float(value) for value in correction),
            centered=centered,
            alignment_verified=verified,
            iteration=self.iteration,
        )


def require_vertical_motion(
    start_xyz_m: Sequence[float],
    target_xyz_m: Sequence[float],
    *,
    max_xy_drift_m: float = 0.0005,
) -> None:
    """Reject any nominal descent/retreat containing lateral translation."""

    start = np.asarray(start_xyz_m, dtype=np.float64)
    target = np.asarray(target_xyz_m, dtype=np.float64)
    if start.shape != (3,) or target.shape != (3,) or not np.all(
        np.isfinite(np.concatenate((start, target)))
    ):
        raise VisualServoError("vertical-motion endpoints must be finite XYZ points")
    drift = float(np.linalg.norm(target[:2] - start[:2]))
    if drift > max_xy_drift_m:
        raise VisualServoError(
            f"low motion contains {drift * 1000.0:.2f}mm lateral travel"
        )
