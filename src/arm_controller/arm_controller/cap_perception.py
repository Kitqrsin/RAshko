"""Fail-closed planar perception for the seven-cap battery fixture.

This module deliberately has no ROS, robot-control, or inverse-kinematics
dependencies.  A caller supplies one BGR image and the seven cap slots in a
fixture plane.  Every frame gets its own plane-to-image homography, either
from detected ArUco marker corners or from explicit correspondences.

The module never authorizes motion from a single frame.  Feed consecutive
``FrameObservation`` objects to ``ObservationStabilizer`` and act only when
its returned ``StableObservation.safe_to_move`` value is true.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import hashlib
import math
from numbers import Integral, Real
from types import MappingProxyType
from typing import Iterable, Mapping, Optional, Sequence

import cv2
import numpy as np

from arm_controller.fixture_pose import (
    FixturePoseEstimate,
    FixturePoseReference,
    compare_fixture_pose_to_reference,
    estimate_fixture_pose,
    quaternion_rotation_distance_deg,
)


Point2 = tuple[float, float]
_ARUCO = getattr(cv2, "aruco", None)
_DEFAULT_ARUCO_DICTIONARY = (
    getattr(_ARUCO, "DICT_4X4_50", 0) if _ARUCO is not None else 0
)

__all__ = (
    "CapPerception",
    "CapSlot",
    "FixtureLayout",
    "FixturePoseEstimate",
    "FixturePoseReference",
    "FrameObservation",
    "HomographyObservation",
    "InspectorConfig",
    "ObservationStabilizer",
    "PerceptionError",
    "SlotObservation",
    "SlotOccupancy",
    "StableObservation",
    "detect_aruco_correspondences",
    "estimate_planar_homography",
    "image_content_digest",
    "marker_correspondences_from_detections",
)


class PerceptionError(ValueError):
    """Raised when fixture geometry cannot be trusted."""


class SlotOccupancy(str, Enum):
    """Classification of one known cap slot."""

    PRESENT = "present"
    ABSENT = "absent"
    AMBIGUOUS = "ambiguous"


def image_content_digest(image_bgr: np.ndarray) -> bytes:
    """Fingerprint decoded pixels so repeated transport frames are detectable."""

    image = np.asarray(image_bgr)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("image content digest requires an HxWx3 uint8 image")
    contiguous = np.ascontiguousarray(image)
    return hashlib.blake2s(contiguous.data, digest_size=16).digest()


@dataclass(frozen=True)
class CapSlot:
    """One stable cap identity and its expected fixture-plane geometry."""

    slot_id: str
    x_m: float
    y_m: float
    search_radius_m: float
    expected_radius_m: float = 0.005
    max_center_error_m: float = 0.003

    @property
    def center_m(self) -> Point2:
        """Return the nominal cap center in the fixture plane."""

        return (self.x_m, self.y_m)


@dataclass(frozen=True)
class FixtureLayout:
    """Seven cap slots, workspace bounds, and optional ArUco geometry.

    ``marker_corners_m`` maps an ArUco ID to its four physical corners in
    marker order: top-left, top-right, bottom-right, bottom-left.  Coordinates
    are metres in the same planar fixture frame as the cap slots.
    """

    workspace_bounds_m: tuple[float, float, float, float]
    slots: tuple[CapSlot, ...]
    marker_corners_m: Mapping[int, Sequence[Sequence[float]]]

    def __post_init__(self) -> None:
        """Reject layouts which could assign one blob to multiple slots."""

        slots = tuple(self.slots)
        bounds_tuple = tuple(float(value) for value in self.workspace_bounds_m)
        marker_layout = {
            marker_id: tuple(
                (float(point[0]), float(point[1])) for point in corners
            )
            for marker_id, corners in self.marker_corners_m.items()
        }
        object.__setattr__(self, "slots", slots)
        object.__setattr__(self, "workspace_bounds_m", bounds_tuple)
        object.__setattr__(
            self,
            "marker_corners_m",
            MappingProxyType(marker_layout),
        )

        x_min, y_min, x_max, y_max = bounds_tuple
        bounds = np.asarray(bounds_tuple, dtype=np.float64)
        if not np.all(np.isfinite(bounds)) or x_max <= x_min or y_max <= y_min:
            raise ValueError("workspace_bounds_m must contain finite min/max bounds")
        if len(self.slots) != 7:
            raise ValueError("the battery fixture must define exactly seven cap slots")

        identifiers = [slot.slot_id for slot in self.slots]
        if any(not identifier for identifier in identifiers):
            raise ValueError("every cap slot needs a non-empty stable ID")
        if len(set(identifiers)) != len(identifiers):
            raise ValueError("cap slot IDs must be unique")

        for slot in self.slots:
            values = np.asarray(
                (
                    slot.x_m,
                    slot.y_m,
                    slot.search_radius_m,
                    slot.expected_radius_m,
                    slot.max_center_error_m,
                ),
                dtype=np.float64,
            )
            if not np.all(np.isfinite(values)):
                raise ValueError(f"slot {slot.slot_id!r} contains non-finite values")
            if min(values[2:]) <= 0.0:
                raise ValueError(f"slot {slot.slot_id!r} radii must be positive")
            if slot.expected_radius_m >= slot.search_radius_m:
                raise ValueError(
                    f"slot {slot.slot_id!r} expected radius must fit its search ROI"
                )
            if slot.max_center_error_m + slot.expected_radius_m > slot.search_radius_m:
                raise ValueError(
                    f"slot {slot.slot_id!r} cap plus center tolerance exceeds its ROI"
                )
            if not (
                x_min + slot.search_radius_m <= slot.x_m <= x_max - slot.search_radius_m
                and y_min + slot.search_radius_m
                <= slot.y_m
                <= y_max - slot.search_radius_m
            ):
                raise ValueError(f"slot {slot.slot_id!r} search ROI leaves the workspace")

        for index, first in enumerate(self.slots):
            for second in self.slots[index + 1:]:
                separation = math.hypot(first.x_m - second.x_m, first.y_m - second.y_m)
                if separation <= first.search_radius_m + second.search_radius_m:
                    raise ValueError(
                        f"search ROIs for {first.slot_id!r} and {second.slot_id!r} overlap"
                    )

        for marker_id, corners in self.marker_corners_m.items():
            if isinstance(marker_id, bool) or not isinstance(marker_id, int):
                raise ValueError("ArUco marker IDs must be integers")
            points = np.asarray(corners, dtype=np.float64)
            if points.shape != (4, 2) or not np.all(np.isfinite(points)):
                raise ValueError(
                    f"marker {marker_id} must define four finite planar corners"
                )
            if abs(cv2.contourArea(points.astype(np.float32))) <= 1.0e-10:
                raise ValueError(f"marker {marker_id} corners are degenerate")


@dataclass(frozen=True)
class InspectorConfig:
    """Geometric, colour, shape, and temporal safety thresholds."""

    pixels_per_meter: float = 4000.0
    max_rectified_pixels: int = 5_000_000
    aruco_dictionary_id: int = _DEFAULT_ARUCO_DICTIONARY
    min_visible_markers: int = 2
    ransac_threshold_px: float = 2.0
    min_homography_inlier_ratio: float = 0.80
    max_reprojection_rms_px: float = 1.5
    max_reprojection_error_px: float = 3.0
    max_homography_condition: float = 1.0e10
    min_image_correspondence_hull_fraction: float = 0.08
    min_plane_correspondence_hull_fraction: float = 0.20
    image_boundary_margin_px: float = 2.0
    min_valid_roi_fraction: float = 0.98
    min_grayscale_mean: float = 10.0
    max_grayscale_mean: float = 250.0
    min_laplacian_variance: float = 2.0
    red_low_1: tuple[int, int, int] = (0, 110, 80)
    red_high_1: tuple[int, int, int] = (12, 255, 255)
    red_low_2: tuple[int, int, int] = (168, 110, 80)
    red_high_2: tuple[int, int, int] = (180, 255, 255)
    morphology_radius_m: float = 0.0005
    min_cap_area_ratio: float = 0.40
    max_cap_area_ratio: float = 1.80
    min_cap_radius_ratio: float = 0.62
    max_cap_radius_ratio: float = 1.45
    min_circularity: float = 0.50
    min_solidity: float = 0.80
    max_total_red_area_ratio: float = 2.10
    max_absent_red_area_ratio: float = 0.16
    min_present_confidence: float = 0.62
    min_absent_confidence: float = 0.70
    stability_frames: int = 5
    max_homography_jitter_px: float = 2.5
    max_cap_center_jitter_m: float = 0.0010
    min_stable_slot_confidence: float = 0.68
    require_fixture_pose: bool = False
    max_fixture_pose_reprojection_rms_px: float = 1.0
    max_camera_translation_jitter_m: float = 0.0015
    max_camera_rotation_jitter_deg: float = 0.50
    max_camera_reference_translation_error_m: float = 0.003
    max_camera_reference_rotation_error_deg: float = 1.0

    def __post_init__(self) -> None:
        """Reject nonsensical thresholds rather than silently weakening checks."""

        integer_fields = (
            "max_rectified_pixels",
            "aruco_dictionary_id",
            "min_visible_markers",
            "stability_frames",
        )
        for name in integer_fields:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Integral):
                raise ValueError(f"{name} must be an integer, not a boolean")
        if not isinstance(self.require_fixture_pose, bool):
            raise ValueError("require_fixture_pose must be an unquoted boolean")

        real_fields = (
            "pixels_per_meter",
            "ransac_threshold_px",
            "min_homography_inlier_ratio",
            "max_reprojection_rms_px",
            "max_reprojection_error_px",
            "max_homography_condition",
            "min_image_correspondence_hull_fraction",
            "min_plane_correspondence_hull_fraction",
            "image_boundary_margin_px",
            "min_valid_roi_fraction",
            "min_grayscale_mean",
            "max_grayscale_mean",
            "min_laplacian_variance",
            "morphology_radius_m",
            "min_cap_area_ratio",
            "max_cap_area_ratio",
            "min_cap_radius_ratio",
            "max_cap_radius_ratio",
            "min_circularity",
            "min_solidity",
            "max_total_red_area_ratio",
            "max_absent_red_area_ratio",
            "min_present_confidence",
            "min_absent_confidence",
            "max_homography_jitter_px",
            "max_cap_center_jitter_m",
            "min_stable_slot_confidence",
            "max_fixture_pose_reprojection_rms_px",
            "max_camera_translation_jitter_m",
            "max_camera_rotation_jitter_deg",
            "max_camera_reference_translation_error_m",
            "max_camera_reference_rotation_error_deg",
        )
        for name in real_fields:
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, Real)
                or not math.isfinite(float(value))
            ):
                raise ValueError(f"{name} must be a finite number, not a boolean")

        positive_fields = (
            "pixels_per_meter",
            "ransac_threshold_px",
            "max_reprojection_rms_px",
            "max_reprojection_error_px",
            "max_homography_condition",
            "morphology_radius_m",
            "min_cap_area_ratio",
            "max_cap_area_ratio",
            "min_cap_radius_ratio",
            "max_cap_radius_ratio",
            "max_total_red_area_ratio",
            "max_absent_red_area_ratio",
            "max_homography_jitter_px",
            "max_cap_center_jitter_m",
            "min_laplacian_variance",
            "max_fixture_pose_reprojection_rms_px",
            "max_camera_translation_jitter_m",
            "max_camera_rotation_jitter_deg",
            "max_camera_reference_translation_error_m",
            "max_camera_reference_rotation_error_deg",
        )
        if any(float(getattr(self, name)) <= 0.0 for name in positive_fields):
            raise ValueError("positive perception thresholds must be above zero")

        if self.max_rectified_pixels < 1:
            raise ValueError("max_rectified_pixels must be positive")
        if self.aruco_dictionary_id < 0:
            raise ValueError("aruco_dictionary_id cannot be negative")
        if self.min_visible_markers < 2:
            raise ValueError("min_visible_markers must be at least two")
        if self.stability_frames < 3:
            raise ValueError("stability_frames must be at least three")
        if self.ransac_threshold_px > 5.0:
            raise ValueError("ransac_threshold_px cannot exceed 5 pixels")
        if self.max_reprojection_rms_px > 3.0:
            raise ValueError("max_reprojection_rms_px cannot exceed 3 pixels")
        if self.max_reprojection_error_px > 6.0:
            raise ValueError("max_reprojection_error_px cannot exceed 6 pixels")
        if self.max_reprojection_rms_px > self.max_reprojection_error_px:
            raise ValueError("reprojection RMS limit cannot exceed the maximum limit")
        if self.max_homography_jitter_px > 6.0:
            raise ValueError("max_homography_jitter_px cannot exceed 6 pixels")
        if self.max_fixture_pose_reprojection_rms_px > 3.0:
            raise ValueError(
                "max_fixture_pose_reprojection_rms_px cannot exceed 3 pixels"
            )
        if self.max_camera_translation_jitter_m > 0.010:
            raise ValueError("camera translation jitter cannot exceed 10 mm")
        if self.max_camera_rotation_jitter_deg > 3.0:
            raise ValueError("camera rotation jitter cannot exceed 3 degrees")
        if self.max_camera_reference_translation_error_m > 0.020:
            raise ValueError("camera reference translation error cannot exceed 20 mm")
        if self.max_camera_reference_rotation_error_deg > 5.0:
            raise ValueError("camera reference rotation error cannot exceed 5 degrees")

        safe_fraction_ranges = (
            ("min_homography_inlier_ratio", 0.50),
            ("min_image_correspondence_hull_fraction", 0.01),
            ("min_plane_correspondence_hull_fraction", 0.10),
            ("min_valid_roi_fraction", 0.90),
            ("min_circularity", 0.30),
            ("min_solidity", 0.50),
            ("min_present_confidence", 0.50),
            ("min_absent_confidence", 0.50),
            ("min_stable_slot_confidence", 0.50),
        )
        for name, minimum in safe_fraction_ranges:
            value = float(getattr(self, name))
            if not minimum <= value <= 1.0:
                raise ValueError(f"{name} must be in [{minimum:.2f}, 1.00]")
        if self.min_stable_slot_confidence < min(
            self.min_present_confidence,
            self.min_absent_confidence,
        ):
            raise ValueError(
                "min_stable_slot_confidence cannot weaken per-frame confidence"
            )
        if not 0.0 < self.min_cap_area_ratio < self.max_cap_area_ratio:
            raise ValueError("cap area ratios are invalid")
        if not 0.0 < self.min_cap_radius_ratio < self.max_cap_radius_ratio:
            raise ValueError("cap radius ratios are invalid")
        if self.max_absent_red_area_ratio >= self.min_cap_area_ratio:
            raise ValueError("absence and presence area thresholds must not overlap")
        if self.max_total_red_area_ratio < self.max_cap_area_ratio:
            raise ValueError("total red area limit cannot be below the cap area limit")
        if self.image_boundary_margin_px < 0.0:
            raise ValueError("image boundary margin must not be negative")
        if not (
            math.isfinite(self.min_grayscale_mean)
            and math.isfinite(self.max_grayscale_mean)
            and 0.0 <= self.min_grayscale_mean < self.max_grayscale_mean <= 255.0
        ):
            raise ValueError("grayscale quality limits are invalid")
        for low, high in (
            (self.red_low_1, self.red_high_1),
            (self.red_low_2, self.red_high_2),
        ):
            if len(low) != 3 or len(high) != 3:
                raise ValueError("HSV bounds must each have three components")
            limits = (180, 255, 255)
            for lower, upper, limit in zip(low, high, limits):
                if (
                    isinstance(lower, bool)
                    or isinstance(upper, bool)
                    or not isinstance(lower, Integral)
                    or not isinstance(upper, Integral)
                ):
                    raise ValueError("HSV bounds must contain integers")
                if not 0 <= lower <= upper <= limit:
                    raise ValueError("HSV lower/upper bounds are invalid")

    def validate_layout_raster(self, layout: FixtureLayout) -> None:
        """Prove the rectified raster keeps all seven slot ROIs distinct."""

        scale = float(self.pixels_per_meter)
        x_min, y_min, x_max, y_max = layout.workspace_bounds_m
        width = int(math.ceil((x_max - x_min) * scale)) + 1
        height = int(math.ceil((y_max - y_min) * scale)) + 1
        if width < 32 or height < 32:
            raise ValueError("pixels_per_meter makes the rectified fixture too small")
        if width * height > self.max_rectified_pixels:
            raise ValueError("rectified fixture image exceeds max_rectified_pixels")
        if layout.marker_corners_m and self.min_visible_markers > len(
            layout.marker_corners_m
        ):
            raise ValueError("min_visible_markers exceeds the configured marker count")

        raster_slots = []
        if self.morphology_radius_m * scale < 1.0:
            raise ValueError(
                "pixels_per_meter gives morphology_radius_m less than one pixel"
            )
        for slot in layout.slots:
            expected_radius_px = slot.expected_radius_m * scale
            center_tolerance_px = slot.max_center_error_m * scale
            if expected_radius_px < 4.0:
                raise ValueError(
                    f"pixels_per_meter gives {slot.slot_id} fewer than four "
                    "pixels of expected cap radius"
                )
            if center_tolerance_px < 2.0:
                raise ValueError(
                    f"pixels_per_meter gives {slot.slot_id} fewer than two "
                    "pixels of center tolerance"
                )
            center = np.asarray(
                (
                    round((slot.x_m - x_min) * scale),
                    round((slot.y_m - y_min) * scale),
                ),
                dtype=np.float64,
            )
            radius = int(math.ceil(slot.search_radius_m * scale))
            raster_slots.append((slot.slot_id, center, radius))

        for index, (first_id, first_center, first_radius) in enumerate(
            raster_slots
        ):
            for second_id, second_center, second_radius in raster_slots[index + 1:]:
                separation = float(np.linalg.norm(first_center - second_center))
                if separation <= first_radius + second_radius + 2.0:
                    raise ValueError(
                        f"rectified search ROIs for {first_id!r} and "
                        f"{second_id!r} are not separated by two pixels"
                    )
        if self.max_cap_center_jitter_m > min(
            slot.max_center_error_m for slot in layout.slots
        ):
            raise ValueError(
                "max_cap_center_jitter_m exceeds a slot center-error limit"
            )


@dataclass(frozen=True)
class HomographyObservation:
    """Validated per-frame fixture-plane projection."""

    plane_to_image: np.ndarray
    image_to_plane: np.ndarray
    reprojection_rms_px: float
    reprojection_max_px: float
    inlier_count: int
    correspondence_count: int
    image_hull_fraction: float
    plane_hull_fraction: float
    visible_marker_ids: tuple[int, ...]
    workspace_corners_image_px: np.ndarray


@dataclass(frozen=True)
class SlotObservation:
    """Result for one stable fixture slot ID."""

    slot_id: str
    occupancy: SlotOccupancy
    confidence: float
    centroid_m: Optional[Point2]
    center_error_m: Optional[float]
    contour_area_m2: float
    red_area_m2: float
    circularity: float
    solidity: float


@dataclass(frozen=True)
class FrameObservation:
    """One frame result; never sufficient by itself to authorize motion."""

    frame_sequence: int
    fixture_valid: bool
    classification_valid: bool
    expectation_matches: bool
    reason: str
    expected_present_ids: tuple[str, ...]
    slots: tuple[SlotObservation, ...]
    homography: Optional[HomographyObservation]
    fixture_pose: Optional[FixturePoseEstimate]
    rectified_bgr: Optional[np.ndarray] = None
    annotated_rectified_bgr: Optional[np.ndarray] = None

    @property
    def ready_for_stability(self) -> bool:
        """Return whether this frame may enter the temporal safety gate."""

        return (
            self.fixture_valid
            and self.classification_valid
            and self.expectation_matches
            and self.homography is not None
        )

    @property
    def reprojection_error_px(self) -> float:
        """Expose pixel RMS directly, returning infinity for an invalid fixture."""

        if self.homography is None:
            return math.inf
        return self.homography.reprojection_rms_px

    @property
    def present_ids(self) -> tuple[str, ...]:
        """Return the stable IDs classified present in this frame."""

        return tuple(
            slot.slot_id
            for slot in self.slots
            if slot.occupancy is SlotOccupancy.PRESENT
        )


@dataclass(frozen=True)
class StableObservation:
    """Temporally verified result and the only motion-authorization output."""

    safe_to_move: bool
    reason: str
    frame_sequences: tuple[int, ...]
    expected_present_ids: tuple[str, ...]
    slots: tuple[SlotObservation, ...]
    worst_reprojection_rms_px: float
    minimum_slot_confidence: float
    fixture_pose: Optional[FixturePoseEstimate] = None


def _as_points(points: Sequence[Sequence[float]], name: str) -> np.ndarray:
    """Convert point input to a finite N-by-2 float64 array."""

    result = np.asarray(points, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 2 or len(result) < 4:
        raise PerceptionError(f"{name} must contain at least four 2D points")
    if not np.all(np.isfinite(result)):
        raise PerceptionError(f"{name} contains non-finite values")
    return result


def _project(points: np.ndarray, homography: np.ndarray) -> np.ndarray:
    """Project N two-dimensional points through a homography."""

    shaped = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(shaped, homography)
    result = projected.reshape(-1, 2)
    if not np.all(np.isfinite(result)):
        raise PerceptionError("homography projection produced non-finite coordinates")
    return result


def _convex_hull_area(points: np.ndarray) -> float:
    """Return the absolute convex-hull area of a point set."""

    hull = cv2.convexHull(np.asarray(points, dtype=np.float32))
    return abs(float(cv2.contourArea(hull)))


def marker_correspondences_from_detections(
    detected_corners_px: Sequence[np.ndarray],
    detected_ids: Sequence[int] | np.ndarray,
    marker_corners_m: Mapping[int, Sequence[Sequence[float]]],
) -> tuple[np.ndarray, np.ndarray, tuple[int, ...]]:
    """Match unordered ArUco detections to known physical marker corners.

    Unknown markers are ignored.  A duplicate known ID is rejected because it
    makes the fixture pose ambiguous.
    """

    ids = np.asarray(detected_ids).reshape(-1)
    if len(ids) != len(detected_corners_px):
        raise PerceptionError("ArUco ID and corner counts differ")

    image_points: list[tuple[float, float]] = []
    plane_points: list[tuple[float, float]] = []
    visible: list[int] = []
    seen: set[int] = set()
    for marker_id_value, corners in zip(ids, detected_corners_px):
        marker_id = int(marker_id_value)
        if marker_id not in marker_corners_m:
            continue
        if marker_id in seen:
            raise PerceptionError(f"duplicate known ArUco marker ID {marker_id}")
        image_corners = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        plane_corners = np.asarray(marker_corners_m[marker_id], dtype=np.float64)
        if image_corners.shape != (4, 2) or not np.all(np.isfinite(image_corners)):
            raise PerceptionError(f"marker {marker_id} has invalid detected corners")
        if plane_corners.shape != (4, 2) or not np.all(np.isfinite(plane_corners)):
            raise PerceptionError(f"marker {marker_id} has invalid fixture corners")
        image_points.extend(map(tuple, image_corners))
        plane_points.extend(map(tuple, plane_corners))
        visible.append(marker_id)
        seen.add(marker_id)

    if not visible:
        raise PerceptionError("no known ArUco fixture markers were detected")
    return (
        np.asarray(image_points, dtype=np.float64),
        np.asarray(plane_points, dtype=np.float64),
        tuple(sorted(visible)),
    )


def detect_aruco_correspondences(
    image_bgr: np.ndarray,
    layout: FixtureLayout,
    config: InspectorConfig,
) -> tuple[np.ndarray, np.ndarray, tuple[int, ...]]:
    """Detect known fixture markers and return image/plane correspondences."""

    if not layout.marker_corners_m:
        raise PerceptionError("the fixture has no ArUco marker layout")
    if _ARUCO is None:
        raise PerceptionError(
            "this OpenCV build has no ArUco support; supply marker correspondences"
        )
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    dictionary = _ARUCO.getPredefinedDictionary(config.aruco_dictionary_id)
    if hasattr(_ARUCO, "DetectorParameters"):
        parameters = _ARUCO.DetectorParameters()
    else:  # pragma: no cover - OpenCV 4.6 compatibility
        parameters = _ARUCO.DetectorParameters_create()
    if hasattr(parameters, "cornerRefinementMethod") and hasattr(
        _ARUCO, "CORNER_REFINE_SUBPIX"
    ):
        parameters.cornerRefinementMethod = _ARUCO.CORNER_REFINE_SUBPIX
    if hasattr(_ARUCO, "ArucoDetector"):
        detector = _ARUCO.ArucoDetector(dictionary, parameters)
        corners, ids, _rejected = detector.detectMarkers(gray)
    else:  # pragma: no cover - compatibility with older ROS OpenCV builds
        corners, ids, _rejected = _ARUCO.detectMarkers(
            gray,
            dictionary,
            parameters=parameters,
        )
    if ids is None:
        raise PerceptionError("no ArUco markers were detected")
    image_points, plane_points, visible = marker_correspondences_from_detections(
        corners,
        ids,
        layout.marker_corners_m,
    )
    if len(visible) < config.min_visible_markers:
        raise PerceptionError(
            f"need {config.min_visible_markers} fixture markers; saw {len(visible)}"
        )
    return image_points, plane_points, visible


def estimate_planar_homography(
    plane_points_m: Sequence[Sequence[float]],
    image_points_px: Sequence[Sequence[float]],
    image_size_wh: tuple[int, int],
    workspace_bounds_m: tuple[float, float, float, float],
    config: Optional[InspectorConfig] = None,
    visible_marker_ids: Sequence[int] = (),
) -> HomographyObservation:
    """Estimate and validate a plane-to-image homography for one frame.

    RANSAC is deliberately solved in the plane-to-image direction, making its
    rejection threshold pixel based.  The inverse is computed only after the
    pixel-domain fit passes every quality check.
    """

    settings = config or InspectorConfig()
    plane = _as_points(plane_points_m, "plane_points_m")
    image = _as_points(image_points_px, "image_points_px")
    if plane.shape != image.shape:
        raise PerceptionError("plane and image correspondence counts differ")

    width_px, height_px = image_size_wh
    if width_px < 2 or height_px < 2:
        raise PerceptionError("image dimensions are invalid")
    if np.any(image[:, 0] < 0.0) or np.any(image[:, 0] > width_px - 1):
        raise PerceptionError("an image correspondence lies outside the frame")
    if np.any(image[:, 1] < 0.0) or np.any(image[:, 1] > height_px - 1):
        raise PerceptionError("an image correspondence lies outside the frame")

    x_min, y_min, x_max, y_max = workspace_bounds_m
    plane_area = (x_max - x_min) * (y_max - y_min)
    if not math.isfinite(plane_area) or plane_area <= 0.0:
        raise PerceptionError("workspace bounds are invalid")

    plane_to_image, inlier_mask = cv2.findHomography(
        plane,
        image,
        cv2.RANSAC,
        settings.ransac_threshold_px,
    )
    if plane_to_image is None or inlier_mask is None:
        raise PerceptionError("OpenCV could not estimate the fixture homography")
    plane_to_image = np.asarray(plane_to_image, dtype=np.float64)
    if not np.all(np.isfinite(plane_to_image)):
        raise PerceptionError("fixture homography contains non-finite values")
    if abs(plane_to_image[2, 2]) > 1.0e-15:
        plane_to_image = plane_to_image / plane_to_image[2, 2]
    if np.linalg.matrix_rank(plane_to_image) != 3:
        raise PerceptionError("fixture homography is singular")
    condition = float(np.linalg.cond(plane_to_image))
    if not math.isfinite(condition) or condition > settings.max_homography_condition:
        raise PerceptionError("fixture homography is ill-conditioned")

    inliers = np.asarray(inlier_mask, dtype=np.uint8).reshape(-1).astype(bool)
    inlier_count = int(np.count_nonzero(inliers))
    if inlier_count < 4:
        raise PerceptionError("fixture homography has fewer than four inliers")
    inlier_ratio = inlier_count / len(plane)
    if inlier_ratio < settings.min_homography_inlier_ratio:
        raise PerceptionError("too many fixture correspondences were rejected")

    image_hull_fraction = _convex_hull_area(image[inliers]) / float(
        width_px * height_px
    )
    plane_hull_fraction = _convex_hull_area(plane[inliers]) / plane_area
    if image_hull_fraction < settings.min_image_correspondence_hull_fraction:
        raise PerceptionError(
            "fixture inliers cover too little of the camera image"
        )
    if plane_hull_fraction < settings.min_plane_correspondence_hull_fraction:
        raise PerceptionError("fixture inliers are too narrowly distributed")

    projected = _project(plane, plane_to_image)
    pixel_errors = np.linalg.norm(projected - image, axis=1)[inliers]
    rms_error = float(np.sqrt(np.mean(np.square(pixel_errors))))
    maximum_error = float(np.max(pixel_errors))
    if rms_error > settings.max_reprojection_rms_px:
        raise PerceptionError(
            f"fixture reprojection RMS {rms_error:.2f}px exceeds limit"
        )
    if maximum_error > settings.max_reprojection_error_px:
        raise PerceptionError(
            f"fixture reprojection maximum {maximum_error:.2f}px exceeds limit"
        )

    try:
        image_to_plane = np.linalg.inv(plane_to_image)
    except np.linalg.LinAlgError as exc:
        raise PerceptionError("fixture homography cannot be inverted") from exc
    image_to_plane = image_to_plane / image_to_plane[2, 2]

    workspace_corners = np.asarray(
        (
            (x_min, y_min),
            (x_max, y_min),
            (x_max, y_max),
            (x_min, y_max),
        ),
        dtype=np.float64,
    )
    corners_image = _project(workspace_corners, plane_to_image)
    if not cv2.isContourConvex(corners_image.astype(np.float32)):
        raise PerceptionError("projected workspace is folded or non-convex")
    margin = settings.image_boundary_margin_px
    if (
        np.any(corners_image[:, 0] < margin)
        or np.any(corners_image[:, 0] > width_px - 1 - margin)
        or np.any(corners_image[:, 1] < margin)
        or np.any(corners_image[:, 1] > height_px - 1 - margin)
    ):
        raise PerceptionError("the complete fixture workspace is not visible")

    for matrix in (plane_to_image, image_to_plane, corners_image):
        matrix.setflags(write=False)
    return HomographyObservation(
        plane_to_image=plane_to_image,
        image_to_plane=image_to_plane,
        reprojection_rms_px=rms_error,
        reprojection_max_px=maximum_error,
        inlier_count=inlier_count,
        correspondence_count=len(plane),
        image_hull_fraction=image_hull_fraction,
        plane_hull_fraction=plane_hull_fraction,
        visible_marker_ids=tuple(sorted(int(value) for value in visible_marker_ids)),
        workspace_corners_image_px=corners_image,
    )


@dataclass(frozen=True)
class _Candidate:
    """Internal accepted red contour metrics."""

    confidence: float
    centroid_m: Point2
    center_error_m: float
    area_m2: float
    circularity: float
    solidity: float


class CapPerception:
    """Inspect one calibrated view of the marked planar battery fixture."""

    def __init__(
        self,
        layout: FixtureLayout,
        config: Optional[InspectorConfig] = None,
        camera_pose_reference: Optional[FixturePoseReference] = None,
    ) -> None:
        """Store a validated immutable fixture description and thresholds."""

        self.layout = layout
        self.config = config or InspectorConfig()
        self.config.validate_layout_raster(layout)
        self.camera_pose_reference = camera_pose_reference
        self._slot_by_id = {slot.slot_id: slot for slot in layout.slots}

    def inspect_bgr(
        self,
        image_bgr: np.ndarray,
        *,
        frame_sequence: int,
        expected_present_ids: Iterable[str],
        image_points_px: Optional[Sequence[Sequence[float]]] = None,
        plane_points_m: Optional[Sequence[Sequence[float]]] = None,
        source_valid_mask: Optional[np.ndarray] = None,
        camera_matrix: Optional[Sequence[Sequence[float]]] = None,
        include_debug_images: bool = False,
    ) -> FrameObservation:
        """Inspect one BGR frame using ArUco or explicit correspondences.

        Explicit correspondences are useful for calibrated upstream fiducial
        detectors and deterministic tests.  If omitted, ArUco markers are
        detected from the frame.  Supplying only one correspondence array is
        an error.  The expected-present set is mandatory so a missing or extra
        cap cannot silently authorize motion.
        """

        expected, expected_error = self._validate_expected_ids(expected_present_ids)
        if expected_error:
            return self._invalid_frame(frame_sequence, (), expected_error)
        if isinstance(frame_sequence, bool) or not isinstance(frame_sequence, int):
            return self._invalid_frame(-1, expected, "frame_sequence must be an integer")
        if frame_sequence < 0:
            return self._invalid_frame(
                frame_sequence,
                expected,
                "frame_sequence must be non-negative",
            )
        if (
            not isinstance(image_bgr, np.ndarray)
            or image_bgr.dtype != np.uint8
            or image_bgr.ndim != 3
            or image_bgr.shape[2] != 3
            or image_bgr.shape[0] < 2
            or image_bgr.shape[1] < 2
        ):
            return self._invalid_frame(
                frame_sequence,
                expected,
                "image must be a non-empty uint8 BGR array",
            )
        if (image_points_px is None) != (plane_points_m is None):
            return self._invalid_frame(
                frame_sequence,
                expected,
                "image and plane correspondences must be supplied together",
            )

        try:
            pose_gate_reason = None
            source_valid = self._normalize_source_valid_mask(
                image_bgr, source_valid_mask
            )
            gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
            quality_mask = cv2.erode(
                source_valid,
                np.ones((3, 3), dtype=np.uint8),
                iterations=1,
            )
            valid_pixels = quality_mask > 0
            if int(np.count_nonzero(valid_pixels)) < 16:
                raise PerceptionError("camera has too few valid undistorted pixels")
            grayscale_mean = float(np.mean(gray[valid_pixels]))
            laplacian = cv2.Laplacian(gray, cv2.CV_64F)
            sharpness = float(laplacian[valid_pixels].var())
            if not (
                self.config.min_grayscale_mean
                <= grayscale_mean
                <= self.config.max_grayscale_mean
            ):
                raise PerceptionError(
                    f"image brightness {grayscale_mean:.1f} is outside "
                    "the commissioned range"
                )
            if sharpness < self.config.min_laplacian_variance:
                raise PerceptionError(
                    f"image sharpness {sharpness:.1f} is below the minimum"
                )
            if image_points_px is None:
                image_points, plane_points, visible = detect_aruco_correspondences(
                    image_bgr,
                    self.layout,
                    self.config,
                )
            else:
                image_points = image_points_px
                plane_points = plane_points_m
                visible = ()
            homography = estimate_planar_homography(
                plane_points,
                image_points,
                (image_bgr.shape[1], image_bgr.shape[0]),
                self.layout.workspace_bounds_m,
                self.config,
                visible,
            )
            fixture_pose = None
            if camera_matrix is not None:
                fixture_pose = estimate_fixture_pose(
                    plane_points,
                    image_points,
                    camera_matrix,
                    max_reprojection_rms_px=(
                        self.config.max_fixture_pose_reprojection_rms_px
                    ),
                )
                if self.camera_pose_reference is not None:
                    fixture_pose = compare_fixture_pose_to_reference(
                        fixture_pose,
                        self.camera_pose_reference,
                        max_translation_error_m=(
                            self.config.max_camera_reference_translation_error_m
                        ),
                        max_rotation_error_deg=(
                            self.config.max_camera_reference_rotation_error_deg
                        ),
                        enforce_limits=False,
                    )
                    if (
                        fixture_pose.reference_translation_error_m
                        > self.config.max_camera_reference_translation_error_m
                    ):
                        pose_gate_reason = (
                            "camera pose translation differs from its "
                            "commissioned reference by "
                            f"{fixture_pose.reference_translation_error_m * 1000.0:.2f}mm"
                        )
                    elif (
                        fixture_pose.reference_rotation_error_deg
                        > self.config.max_camera_reference_rotation_error_deg
                    ):
                        pose_gate_reason = (
                            "camera pose rotation differs from its "
                            "commissioned reference by "
                            f"{fixture_pose.reference_rotation_error_deg:.3f}deg"
                        )
            elif self.config.require_fixture_pose:
                raise PerceptionError(
                    "metric fixture pose requires calibrated camera intrinsics"
                )
            rectified, valid_mask = self._rectify(
                image_bgr,
                homography,
                source_valid,
            )
            slots, annotated = self._classify_slots(
                rectified,
                valid_mask,
                include_debug_images,
            )
        except (PerceptionError, cv2.error, np.linalg.LinAlgError, ValueError) as exc:
            return self._invalid_frame(frame_sequence, expected, str(exc))

        fixture_valid = pose_gate_reason is None
        classification_valid = all(
            slot.occupancy is not SlotOccupancy.AMBIGUOUS for slot in slots
        )
        observed_present = {
            slot.slot_id
            for slot in slots
            if slot.occupancy is SlotOccupancy.PRESENT
        }
        expectation_matches = classification_valid and observed_present == set(expected)
        if pose_gate_reason is not None:
            reason = pose_gate_reason
        elif not classification_valid:
            reason = "one or more cap slots are ambiguous"
        elif not expectation_matches:
            missing = sorted(set(expected) - observed_present)
            extra = sorted(observed_present - set(expected))
            reason = f"cap occupancy differs from expectation; missing={missing}, extra={extra}"
        else:
            reason = "frame valid; temporal stabilization still required"

        debug_rectified = rectified if include_debug_images else None
        if debug_rectified is not None:
            debug_rectified.setflags(write=False)
        if annotated is not None:
            annotated.setflags(write=False)
        return FrameObservation(
            frame_sequence=frame_sequence,
            fixture_valid=fixture_valid,
            classification_valid=classification_valid,
            expectation_matches=expectation_matches,
            reason=reason,
            expected_present_ids=expected,
            slots=slots,
            homography=homography,
            fixture_pose=fixture_pose,
            rectified_bgr=debug_rectified,
            annotated_rectified_bgr=annotated,
        )

    def _validate_expected_ids(
        self,
        expected_present_ids: Iterable[str],
    ) -> tuple[tuple[str, ...], Optional[str]]:
        """Canonicalize and validate the caller's expected occupancy set."""

        if expected_present_ids is None:
            return (), "expected_present_ids is required"
        try:
            values = tuple(expected_present_ids)
        except TypeError:
            return (), "expected_present_ids must be iterable"
        if len(values) != len(set(values)):
            return (), "expected_present_ids contains duplicates"
        unknown = sorted(set(values) - set(self._slot_by_id))
        if unknown:
            return (), f"unknown expected cap slot IDs: {unknown}"
        return tuple(sorted(values)), None

    def _invalid_frame(
        self,
        frame_sequence: int,
        expected: tuple[str, ...],
        reason: str,
    ) -> FrameObservation:
        """Return all seven stable IDs as ambiguous on any validation failure."""

        slots = tuple(
            SlotObservation(
                slot_id=slot.slot_id,
                occupancy=SlotOccupancy.AMBIGUOUS,
                confidence=0.0,
                centroid_m=None,
                center_error_m=None,
                contour_area_m2=0.0,
                red_area_m2=0.0,
                circularity=0.0,
                solidity=0.0,
            )
            for slot in self.layout.slots
        )
        return FrameObservation(
            frame_sequence=frame_sequence,
            fixture_valid=False,
            classification_valid=False,
            expectation_matches=False,
            reason=reason or "fixture validation failed",
            expected_present_ids=expected,
            slots=slots,
            homography=None,
            fixture_pose=None,
        )

    def _rectification_geometry(self) -> tuple[np.ndarray, tuple[int, int]]:
        """Return plane-to-rectified-pixel transform and image dimensions."""

        x_min, y_min, x_max, y_max = self.layout.workspace_bounds_m
        scale = self.config.pixels_per_meter
        plane_to_rectified = np.asarray(
            (
                (scale, 0.0, -x_min * scale),
                (0.0, scale, -y_min * scale),
                (0.0, 0.0, 1.0),
            ),
            dtype=np.float64,
        )
        width = int(math.ceil((x_max - x_min) * scale)) + 1
        height = int(math.ceil((y_max - y_min) * scale)) + 1
        if width * height > self.config.max_rectified_pixels:
            raise PerceptionError("rectified fixture image exceeds the configured limit")
        return plane_to_rectified, (width, height)

    @staticmethod
    def _normalize_source_valid_mask(
        image_bgr: np.ndarray,
        source_valid_mask: Optional[np.ndarray],
    ) -> np.ndarray:
        """Return a binary mask for real pixels after camera undistortion."""

        if source_valid_mask is None:
            return np.full(image_bgr.shape[:2], 255, dtype=np.uint8)
        if (
            not isinstance(source_valid_mask, np.ndarray)
            or source_valid_mask.shape != image_bgr.shape[:2]
            or source_valid_mask.dtype not in (np.dtype(np.uint8), np.dtype(bool))
        ):
            raise PerceptionError(
                "source_valid_mask must match the image and be uint8 or bool"
            )
        return np.where(source_valid_mask, 255, 0).astype(np.uint8)

    def _rectify(
        self,
        image_bgr: np.ndarray,
        homography: HomographyObservation,
        source_valid: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Warp the visible fixture plane and validate every cap ROI coverage."""

        plane_to_rectified, size = self._rectification_geometry()
        image_to_rectified = plane_to_rectified @ homography.image_to_plane
        rectified = cv2.warpPerspective(
            image_bgr,
            image_to_rectified,
            size,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
        valid_mask = cv2.warpPerspective(
            source_valid,
            image_to_rectified,
            size,
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        for slot in self.layout.slots:
            center_px = self._plane_to_rectified_px(slot.center_m)
            radius_px = max(1, int(round(slot.search_radius_m * self.config.pixels_per_meter)))
            roi_mask = np.zeros(valid_mask.shape, dtype=np.uint8)
            cv2.circle(roi_mask, center_px, radius_px, 255, -1)
            pixel_count = cv2.countNonZero(roi_mask)
            if pixel_count <= 0:
                raise PerceptionError(f"slot {slot.slot_id} has an empty rectified ROI")
            valid_fraction = cv2.countNonZero(cv2.bitwise_and(valid_mask, roi_mask)) / pixel_count
            if valid_fraction < self.config.min_valid_roi_fraction:
                raise PerceptionError(
                    f"slot {slot.slot_id} is not completely visible in the camera frame"
                )
        return rectified, valid_mask

    def _plane_to_rectified_px(self, point_m: Point2) -> tuple[int, int]:
        """Convert one fixture coordinate to a rectified integer pixel."""

        x_min, y_min, _x_max, _y_max = self.layout.workspace_bounds_m
        scale = self.config.pixels_per_meter
        return (
            int(round((point_m[0] - x_min) * scale)),
            int(round((point_m[1] - y_min) * scale)),
        )

    def _classify_slots(
        self,
        rectified_bgr: np.ndarray,
        valid_mask: np.ndarray,
        include_debug_images: bool,
    ) -> tuple[tuple[SlotObservation, ...], Optional[np.ndarray]]:
        """Classify red circular candidates independently in seven fixed ROIs."""

        hsv = cv2.cvtColor(rectified_bgr, cv2.COLOR_BGR2HSV)
        first = cv2.inRange(hsv, self.config.red_low_1, self.config.red_high_1)
        second = cv2.inRange(hsv, self.config.red_low_2, self.config.red_high_2)
        red_mask = cv2.bitwise_and(cv2.bitwise_or(first, second), valid_mask)
        kernel_radius = max(
            1,
            int(round(self.config.morphology_radius_m * self.config.pixels_per_meter)),
        )
        kernel_size = kernel_radius * 2 + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size),
        )
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        annotated = rectified_bgr.copy() if include_debug_images else None
        observations: list[SlotObservation] = []
        for slot in self.layout.slots:
            observation = self._classify_slot(slot, red_mask)
            observations.append(observation)
            if annotated is not None:
                self._annotate_slot(annotated, slot, observation)
        return tuple(observations), annotated

    def _classify_slot(
        self,
        slot: CapSlot,
        red_mask: np.ndarray,
    ) -> SlotObservation:
        """Apply conservative area, radius, shape, and position gates to one ROI."""

        scale = self.config.pixels_per_meter
        center_px = self._plane_to_rectified_px(slot.center_m)
        search_px = max(1, int(round(slot.search_radius_m * scale)))
        x0 = max(0, center_px[0] - search_px)
        y0 = max(0, center_px[1] - search_px)
        x1 = min(red_mask.shape[1], center_px[0] + search_px + 1)
        y1 = min(red_mask.shape[0], center_px[1] + search_px + 1)
        crop = red_mask[y0:y1, x0:x1].copy()
        local_center = (center_px[0] - x0, center_px[1] - y0)
        roi_mask = np.zeros(crop.shape, dtype=np.uint8)
        cv2.circle(roi_mask, local_center, search_px, 255, -1)
        crop = cv2.bitwise_and(crop, roi_mask)

        total_red_area_m2 = cv2.countNonZero(crop) / (scale * scale)
        expected_area_m2 = math.pi * slot.expected_radius_m**2
        total_red_ratio = total_red_area_m2 / expected_area_m2
        contours, _hierarchy = cv2.findContours(
            crop,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        candidates: list[_Candidate] = []
        for contour in contours:
            candidate = self._candidate_from_contour(slot, contour, x0, y0)
            if candidate is not None:
                candidates.append(candidate)

        if len(candidates) == 1 and total_red_ratio <= self.config.max_total_red_area_ratio:
            candidate = candidates[0]
            if candidate.confidence >= self.config.min_present_confidence:
                return SlotObservation(
                    slot_id=slot.slot_id,
                    occupancy=SlotOccupancy.PRESENT,
                    confidence=candidate.confidence,
                    centroid_m=candidate.centroid_m,
                    center_error_m=candidate.center_error_m,
                    contour_area_m2=candidate.area_m2,
                    red_area_m2=total_red_area_m2,
                    circularity=candidate.circularity,
                    solidity=candidate.solidity,
                )

        if not candidates and total_red_ratio <= self.config.max_absent_red_area_ratio:
            absence_confidence = 1.0 - (
                total_red_ratio / self.config.max_absent_red_area_ratio
            )
            absence_confidence = float(np.clip(absence_confidence, 0.0, 1.0))
            if absence_confidence >= self.config.min_absent_confidence:
                return SlotObservation(
                    slot_id=slot.slot_id,
                    occupancy=SlotOccupancy.ABSENT,
                    confidence=absence_confidence,
                    centroid_m=None,
                    center_error_m=None,
                    contour_area_m2=0.0,
                    red_area_m2=total_red_area_m2,
                    circularity=0.0,
                    solidity=0.0,
                )

        best = max(candidates, key=lambda item: item.confidence, default=None)
        return SlotObservation(
            slot_id=slot.slot_id,
            occupancy=SlotOccupancy.AMBIGUOUS,
            confidence=0.0 if best is None else min(best.confidence, 0.59),
            centroid_m=None if best is None else best.centroid_m,
            center_error_m=None if best is None else best.center_error_m,
            contour_area_m2=0.0 if best is None else best.area_m2,
            red_area_m2=total_red_area_m2,
            circularity=0.0 if best is None else best.circularity,
            solidity=0.0 if best is None else best.solidity,
        )

    def _candidate_from_contour(
        self,
        slot: CapSlot,
        contour: np.ndarray,
        crop_x: int,
        crop_y: int,
    ) -> Optional[_Candidate]:
        """Return metrics for a contour only when every hard gate passes."""

        scale = self.config.pixels_per_meter
        area_px = float(cv2.contourArea(contour))
        if area_px <= 0.0:
            return None
        area_m2 = area_px / (scale * scale)
        expected_area = math.pi * slot.expected_radius_m**2
        area_ratio = area_m2 / expected_area
        if not self.config.min_cap_area_ratio <= area_ratio <= self.config.max_cap_area_ratio:
            return None

        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            return None
        circularity = 4.0 * math.pi * area_px / (perimeter * perimeter)
        if circularity < self.config.min_circularity:
            return None

        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        solidity = area_px / hull_area if hull_area > 0.0 else 0.0
        if solidity < self.config.min_solidity:
            return None

        _circle_center, radius_px = cv2.minEnclosingCircle(contour)
        radius_ratio = (radius_px / scale) / slot.expected_radius_m
        if not (
            self.config.min_cap_radius_ratio
            <= radius_ratio
            <= self.config.max_cap_radius_ratio
        ):
            return None
        moments = cv2.moments(contour)
        if abs(moments["m00"]) <= 1.0e-12:
            return None
        centroid_px = np.asarray(
            (
                crop_x + moments["m10"] / moments["m00"],
                crop_y + moments["m01"] / moments["m00"],
            ),
            dtype=np.float64,
        )
        x_min, y_min, _x_max, _y_max = self.layout.workspace_bounds_m
        centroid_m = (
            float(centroid_px[0] / scale + x_min),
            float(centroid_px[1] / scale + y_min),
        )
        center_error = math.hypot(
            centroid_m[0] - slot.x_m,
            centroid_m[1] - slot.y_m,
        )
        if center_error > slot.max_center_error_m:
            return None

        center_score = max(0.0, 1.0 - center_error / slot.max_center_error_m)
        area_score = math.exp(-abs(math.log(max(area_ratio, 1.0e-12))))
        radius_score = math.exp(-abs(math.log(max(radius_ratio, 1.0e-12))))
        circularity_score = np.clip(
            (circularity - self.config.min_circularity)
            / max(1.0e-9, 1.0 - self.config.min_circularity),
            0.0,
            1.0,
        )
        solidity_score = np.clip(
            (solidity - self.config.min_solidity)
            / max(1.0e-9, 1.0 - self.config.min_solidity),
            0.0,
            1.0,
        )
        confidence = float(
            0.30 * center_score
            + 0.25 * area_score
            + 0.15 * radius_score
            + 0.20 * circularity_score
            + 0.10 * solidity_score
        )
        return _Candidate(
            confidence=confidence,
            centroid_m=centroid_m,
            center_error_m=center_error,
            area_m2=area_m2,
            circularity=circularity,
            solidity=solidity,
        )

    def _annotate_slot(
        self,
        image: np.ndarray,
        slot: CapSlot,
        observation: SlotObservation,
    ) -> None:
        """Draw one debug-only slot circle and classification label."""

        colors = {
            SlotOccupancy.PRESENT: (0, 220, 0),
            SlotOccupancy.ABSENT: (255, 180, 0),
            SlotOccupancy.AMBIGUOUS: (0, 0, 255),
        }
        center = self._plane_to_rectified_px(slot.center_m)
        radius = int(round(slot.search_radius_m * self.config.pixels_per_meter))
        color = colors[observation.occupancy]
        cv2.circle(image, center, radius, color, 2)
        label = f"{slot.slot_id}:{observation.occupancy.value[0]} {observation.confidence:.2f}"
        cv2.putText(
            image,
            label,
            (center[0] - radius, max(12, center[1] - radius - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            color,
            1,
            cv2.LINE_AA,
        )


class ObservationStabilizer:
    """Require distinct, geometrically stable frames before motion is allowed."""

    def __init__(
        self,
        layout: FixtureLayout,
        config: Optional[InspectorConfig] = None,
    ) -> None:
        """Create an empty consecutive-frame safety gate."""

        self.layout = layout
        self.config = config or InspectorConfig()
        self._frames: deque[FrameObservation] = deque(
            maxlen=self.config.stability_frames
        )
        self._last_sequence: Optional[int] = None

    def reset(self) -> None:
        """Discard accumulated frames while retaining stale-frame protection."""

        self._frames.clear()

    def add(self, frame: FrameObservation) -> StableObservation:
        """Add one new frame and return a fail-closed stability decision."""

        if self._last_sequence is not None and frame.frame_sequence <= self._last_sequence:
            self._frames.clear()
            return self._unsafe("frame sequence is stale or repeated", frame)
        self._last_sequence = frame.frame_sequence
        if not frame.ready_for_stability:
            self._frames.clear()
            return self._unsafe(frame.reason, frame)

        if self._frames and (
            frame.expected_present_ids != self._frames[-1].expected_present_ids
        ):
            self._frames.clear()
        self._frames.append(frame)
        if len(self._frames) < self.config.stability_frames:
            return self._unsafe(
                f"need {self.config.stability_frames - len(self._frames)} more stable frames",
                frame,
            )

        frames = tuple(self._frames)
        references = np.stack(
            [item.homography.workspace_corners_image_px for item in frames]
        )
        median_reference = np.median(references, axis=0)
        homography_jitter = float(
            np.max(np.linalg.norm(references - median_reference, axis=2))
        )
        if homography_jitter > self.config.max_homography_jitter_px:
            self._frames.clear()
            return self._unsafe(
                f"fixture pose jitter {homography_jitter:.2f}px exceeds limit",
                frame,
            )

        poses = [item.fixture_pose for item in frames]
        fixture_pose = None
        if any(item is not None for item in poses):
            if any(item is None for item in poses):
                self._frames.clear()
                return self._unsafe(
                    "metric camera pose disappeared between stable frames",
                    frame,
                )
            complete_poses = tuple(item for item in poses if item is not None)
            positions = np.asarray(
                [item.position_fixture_m for item in complete_poses],
                dtype=np.float64,
            )
            median_position = np.median(positions, axis=0)
            translation_jitter = float(
                np.max(np.linalg.norm(positions - median_position, axis=1))
            )
            if translation_jitter > self.config.max_camera_translation_jitter_m:
                self._frames.clear()
                return self._unsafe(
                    "camera translation is not stable "
                    f"({translation_jitter * 1000.0:.2f}mm jitter)",
                    frame,
                )
            rotation_jitter = max(
                quaternion_rotation_distance_deg(
                    first.orientation_fixture_from_camera_xyzw,
                    second.orientation_fixture_from_camera_xyzw,
                )
                for index, first in enumerate(complete_poses)
                for second in complete_poses[index:]
            )
            if rotation_jitter > self.config.max_camera_rotation_jitter_deg:
                self._frames.clear()
                return self._unsafe(
                    "camera orientation is not stable "
                    f"({rotation_jitter:.3f}deg jitter)",
                    frame,
                )
            fixture_pose = complete_poses[-1]

        aggregate: list[SlotObservation] = []
        for slot_index, slot in enumerate(self.layout.slots):
            observations = [item.slots[slot_index] for item in frames]
            if any(item.slot_id != slot.slot_id for item in observations):
                self._frames.clear()
                return self._unsafe("cap slot ordering changed between frames", frame)
            occupancies = {item.occupancy for item in observations}
            if len(occupancies) != 1:
                self._frames.clear()
                return self._unsafe(
                    f"slot {slot.slot_id} occupancy changed between frames",
                    frame,
                )
            minimum_confidence = min(item.confidence for item in observations)
            if minimum_confidence < self.config.min_stable_slot_confidence:
                self._frames.clear()
                return self._unsafe(
                    f"slot {slot.slot_id} confidence is below the stable limit",
                    frame,
                )
            occupancy = observations[0].occupancy
            if occupancy is SlotOccupancy.PRESENT:
                centers = np.asarray(
                    [item.centroid_m for item in observations],
                    dtype=np.float64,
                )
                center = np.mean(centers, axis=0)
                center_jitter = float(
                    np.max(np.linalg.norm(centers - center, axis=1))
                )
                if center_jitter > self.config.max_cap_center_jitter_m:
                    self._frames.clear()
                    return self._unsafe(
                        f"slot {slot.slot_id} centroid is not stable",
                        frame,
                    )
                centroid: Optional[Point2] = (float(center[0]), float(center[1]))
                center_error: Optional[float] = math.hypot(
                    centroid[0] - slot.x_m,
                    centroid[1] - slot.y_m,
                )
            else:
                centroid = None
                center_error = None
            aggregate.append(
                SlotObservation(
                    slot_id=slot.slot_id,
                    occupancy=occupancy,
                    confidence=minimum_confidence,
                    centroid_m=centroid,
                    center_error_m=center_error,
                    contour_area_m2=float(
                        np.mean([item.contour_area_m2 for item in observations])
                    ),
                    red_area_m2=float(
                        np.mean([item.red_area_m2 for item in observations])
                    ),
                    circularity=float(
                        np.mean([item.circularity for item in observations])
                    ),
                    solidity=float(np.mean([item.solidity for item in observations])),
                )
            )

        minimum_confidence = min(item.confidence for item in aggregate)
        worst_reprojection = max(
            item.homography.reprojection_rms_px for item in frames
        )
        return StableObservation(
            safe_to_move=True,
            reason="fixture and all seven cap slots are temporally stable",
            frame_sequences=tuple(item.frame_sequence for item in frames),
            expected_present_ids=frame.expected_present_ids,
            slots=tuple(aggregate),
            worst_reprojection_rms_px=worst_reprojection,
            minimum_slot_confidence=minimum_confidence,
            fixture_pose=fixture_pose,
        )

    def _unsafe(
        self,
        reason: str,
        frame: FrameObservation,
    ) -> StableObservation:
        """Build a conservative non-motion result for the current buffer."""

        frames = tuple(self._frames)
        usable = frames if frames else (frame,)
        reprojection = [
            item.homography.reprojection_rms_px
            for item in usable
            if item.homography is not None
        ]
        confidences = [
            slot.confidence
            for item in usable
            for slot in item.slots
        ]
        return StableObservation(
            safe_to_move=False,
            reason=reason or "perception validation failed",
            frame_sequences=tuple(item.frame_sequence for item in frames),
            expected_present_ids=frame.expected_present_ids,
            slots=frame.slots,
            worst_reprojection_rms_px=max(reprojection, default=math.inf),
            minimum_slot_confidence=min(confidences, default=0.0),
            fixture_pose=frame.fixture_pose,
        )
