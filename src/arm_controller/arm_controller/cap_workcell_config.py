"""Configuration and fail-closed commissioning gates for cap handling."""

from __future__ import annotations

from dataclasses import dataclass, fields
import hashlib
import json
import math
from numbers import Real
import os
from pathlib import Path
from typing import Any, Mapping

import yaml

from arm_controller.cap_perception import CapSlot, FixtureLayout, InspectorConfig
from arm_controller.fixture_pose import FixturePoseReference
from arm_controller.joint_contract import (
    GRIPPER_OPEN_DEGREES,
    validate_servo_degrees,
)
from arm_controller.red_cap_visual_servo import (
    FixedJigReference,
    VisualServoConfig,
)


FIXTURE_COORDINATE_CONVENTION = "right_handed_z_up"


@dataclass(frozen=True)
class InspectionSettings:
    """Validated subset required for camera-only fixture inspection."""

    source_path: str
    config_sha256: str
    example_only: bool
    commissioned: bool
    fixture_frame_id: str
    camera_image_topic: str
    camera_info_topic: str
    require_calibrated_camera_info: bool
    localization_mode: str
    camera_pose_reference: FixturePoseReference | None
    camera_pose_reference_verified: bool
    fixed_jig_reference: FixedJigReference | None
    fixed_jig_reference_verified: bool
    visual_servo: VisualServoConfig | None
    visual_servo_verified: bool
    maximum_image_age_s: float
    minimum_stability_duration_s: float
    joint_settle_time_s: float
    joint_motion_threshold_deg: float
    observation_joint_positions_deg: tuple[float, ...] | None
    observation_pose_tolerance_deg: float
    expected_present_ids: tuple[str, ...]
    layout: FixtureLayout
    inspector: InspectorConfig
    raw: Mapping[str, Any]


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{path} must be a mapping")
    return value


def _finite(value: Any, path: str, *, positive: bool = False) -> float:
    # YAML numbers must actually be numeric scalars.  Accepting strings here
    # makes a typo such as "0.25" hash differently while looking valid to an
    # operator, and Python booleans are integers unless rejected explicitly.
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(
            f"{path} must be an unquoted finite YAML number, not a boolean"
        )
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{path} must be a finite number")
    if positive and number <= 0.0:
        raise ValueError(f"{path} must be positive")
    return number


def _environment_number(value: str, path: str) -> float:
    """Parse an environment variable without weakening YAML type checks."""

    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{path} must be a finite number")
    return number


def _boolean(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{path} must be an unquoted YAML boolean")
    return value


def _commissioning_range(
    value: float,
    path: str,
    minimum: float,
    maximum: float,
) -> float:
    if not minimum <= value <= maximum:
        raise ValueError(
            f"{path} must be in the commissioning-safe range "
            f"[{minimum}, {maximum}]"
        )
    return value


def _canonical_hash(document: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        document,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def load_inspection_settings(path: str) -> InspectionSettings:
    """Load the camera-inspection section of a cap-workcell YAML file."""

    source = Path(path)
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    root = _mapping(document, "workcell")
    schema_version = root.get("schema_version")
    if type(schema_version) is not int or schema_version != 1:
        raise ValueError("schema_version must be 1")

    fixture = _mapping(root.get("fixture"), "fixture")
    if fixture.get("coordinate_convention") != FIXTURE_COORDINATE_CONVENTION:
        raise ValueError(
            "fixture.coordinate_convention must be "
            f"{FIXTURE_COORDINATE_CONVENTION!r}"
        )
    bounds_raw = fixture.get("workspace_bounds_m")
    if not isinstance(bounds_raw, (list, tuple)) or len(bounds_raw) != 4:
        raise ValueError("fixture.workspace_bounds_m must contain four values")
    bounds = tuple(
        _finite(value, f"fixture.workspace_bounds_m[{index}]")
        for index, value in enumerate(bounds_raw)
    )

    marker_raw = _mapping(fixture.get("marker_corners_m"), "fixture.marker_corners_m")
    marker_corners = {}
    for raw_id, raw_corners in marker_raw.items():
        if type(raw_id) is not int or raw_id < 0:
            raise ValueError(f"invalid marker ID {raw_id!r}")
        marker_id = raw_id
        if marker_id in marker_corners:
            raise ValueError(f"duplicate marker ID {marker_id}")
        if not isinstance(raw_corners, (list, tuple)) or len(raw_corners) != 4:
            raise ValueError(f"marker {marker_id} must contain four corners")
        corners = []
        for corner_index, point in enumerate(raw_corners):
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError(
                    f"marker {marker_id} corner {corner_index} must be XY"
                )
            corners.append(
                (
                    _finite(point[0], f"marker {marker_id} corner X"),
                    _finite(point[1], f"marker {marker_id} corner Y"),
                )
            )
        marker_corners[marker_id] = tuple(corners)

    cap_section = _mapping(root.get("caps"), "caps")
    raw_slots = cap_section.get("source_slots")
    if not isinstance(raw_slots, list):
        raise ValueError("caps.source_slots must be a list")
    slots = []
    for index, raw_slot in enumerate(raw_slots):
        slot = _mapping(raw_slot, f"caps.source_slots[{index}]")
        slot_id = str(slot.get("id", "")).strip()
        slots.append(
            CapSlot(
                slot_id=slot_id,
                x_m=_finite(slot.get("x_m"), f"{slot_id}.x_m"),
                y_m=_finite(slot.get("y_m"), f"{slot_id}.y_m"),
                search_radius_m=_finite(
                    slot.get("search_radius_m"),
                    f"{slot_id}.search_radius_m",
                    positive=True,
                ),
                expected_radius_m=_finite(
                    slot.get("expected_radius_m"),
                    f"{slot_id}.expected_radius_m",
                    positive=True,
                ),
                max_center_error_m=_finite(
                    slot.get("max_center_error_m"),
                    f"{slot_id}.max_center_error_m",
                    positive=True,
                ),
            )
        )
    layout = FixtureLayout(bounds, tuple(slots), marker_corners)

    perception_raw = _mapping(root.get("perception", {}), "perception")
    inspector_fields = {item.name for item in fields(InspectorConfig)}
    unknown = sorted(set(perception_raw) - inspector_fields)
    if unknown:
        raise ValueError(f"unknown perception setting(s): {', '.join(unknown)}")
    perception_values = dict(perception_raw)
    for name in ("red_low_1", "red_high_1", "red_low_2", "red_high_2"):
        if name in perception_values:
            raw_bound = perception_values[name]
            if not isinstance(raw_bound, (list, tuple)):
                raise ValueError(f"perception.{name} must contain three integers")
            perception_values[name] = tuple(raw_bound)
    inspector = InspectorConfig(**perception_values)
    inspector.validate_layout_raster(layout)

    camera = _mapping(root.get("camera", {}), "camera")
    localization_mode = str(
        camera.get("localization_mode", "aruco")
    ).strip()
    if localization_mode not in {"aruco", "fixed_red_jig", "fixed_cap_row"}:
        raise ValueError(
            "camera.localization_mode must be 'aruco', 'fixed_red_jig', "
            "or 'fixed_cap_row'"
        )
    pose_reference_raw = _mapping(
        camera.get("fixture_pose_reference", {}),
        "camera.fixture_pose_reference",
    )
    position_raw = pose_reference_raw.get("position_fixture_m")
    orientation_raw = pose_reference_raw.get(
        "orientation_fixture_from_camera_xyzw"
    )
    reference_values = []
    for name, raw_values, length in (
        ("position_fixture_m", position_raw, 3),
        (
            "orientation_fixture_from_camera_xyzw",
            orientation_raw,
            4,
        ),
    ):
        if not isinstance(raw_values, list) or len(raw_values) != length:
            raise ValueError(
                f"camera.fixture_pose_reference.{name} must contain "
                f"{length} values"
            )
        reference_values.extend(raw_values)
    if all(value is None for value in reference_values):
        camera_pose_reference = None
    elif any(value is None for value in reference_values):
        raise ValueError("camera fixture pose reference cannot be partially measured")
    else:
        camera_pose_reference = FixturePoseReference(
            position_fixture_m=tuple(
                _finite(
                    value,
                    f"camera.fixture_pose_reference.position_fixture_m[{index}]",
                )
                for index, value in enumerate(position_raw)
            ),
            orientation_fixture_from_camera_xyzw=tuple(
                _finite(
                    value,
                    "camera.fixture_pose_reference."
                    "orientation_fixture_from_camera_xyzw"
                    f"[{index}]",
                )
                for index, value in enumerate(orientation_raw)
            ),
        )
    camera_pose_reference_verified = _boolean(
        pose_reference_raw.get("verified"),
        "camera.fixture_pose_reference.verified",
    )
    fixed_reference_raw = _mapping(
        camera.get("fixed_jig_reference", {}),
        "camera.fixed_jig_reference",
    )
    homography_raw = fixed_reference_raw.get("plane_to_image_homography")
    image_size_raw = fixed_reference_raw.get("image_size_px")
    if not isinstance(homography_raw, list) or len(homography_raw) != 9:
        raise ValueError(
            "camera.fixed_jig_reference.plane_to_image_homography must "
            "contain nine values"
        )
    if all(value is None for value in homography_raw):
        fixed_jig_reference = None
    elif any(value is None for value in homography_raw):
        raise ValueError("fixed-jig homography cannot be partially measured")
    else:
        if not isinstance(image_size_raw, list) or len(image_size_raw) != 2:
            raise ValueError(
                "camera.fixed_jig_reference.image_size_px must contain two integers"
            )
        image_size = tuple(image_size_raw)
        if any(type(value) is not int or value <= 0 for value in image_size):
            raise ValueError(
                "camera.fixed_jig_reference.image_size_px must contain "
                "positive integers"
            )
        fixed_jig_reference = FixedJigReference(
            plane_to_image=tuple(
                _finite(
                    value,
                    "camera.fixed_jig_reference."
                    f"plane_to_image_homography[{index}]",
                )
                for index, value in enumerate(homography_raw)
            ),
            image_size=image_size,
        )
    fixed_jig_reference_verified = _boolean(
        fixed_reference_raw.get("verified"),
        "camera.fixed_jig_reference.verified",
    )
    if fixed_jig_reference_verified and fixed_jig_reference is None:
        raise ValueError("a fixed-jig reference cannot be verified before teaching")
    if localization_mode == "aruco" and not inspector.require_fixture_pose:
        raise ValueError("ArUco localization must require metric fixture pose")
    if localization_mode in {"fixed_red_jig", "fixed_cap_row"} and (
        inspector.require_fixture_pose
    ):
        raise ValueError(
            "fiducial-free localization cannot claim live fixture-pose validation"
        )
    servo_raw = _mapping(
        camera.get("visual_servo", {}),
        "camera.visual_servo",
    )
    target_pixel_raw = servo_raw.get("target_pixel")
    correction_raw = servo_raw.get("correction_matrix_m_per_px")
    if not isinstance(target_pixel_raw, list) or len(target_pixel_raw) != 2:
        raise ValueError("camera.visual_servo.target_pixel must contain UV")
    target_pixel = tuple(
        _finite(value, f"camera.visual_servo.target_pixel[{index}]")
        for index, value in enumerate(target_pixel_raw)
    )
    if not isinstance(correction_raw, list) or len(correction_raw) != 4:
        raise ValueError(
            "camera.visual_servo.correction_matrix_m_per_px must contain "
            "four values"
        )
    if all(value is None for value in correction_raw):
        visual_servo = None
    elif any(value is None for value in correction_raw):
        raise ValueError("visual-servo correction matrix cannot be partial")
    else:
        visual_servo = VisualServoConfig(
            target_pixel=target_pixel,
            correction_matrix_m_per_px=tuple(
                _finite(
                    value,
                    "camera.visual_servo."
                    f"correction_matrix_m_per_px[{index}]",
                )
                for index, value in enumerate(correction_raw)
            ),
            max_step_m=_finite(
                servo_raw.get("max_step_m"),
                "camera.visual_servo.max_step_m",
                positive=True,
            ),
            centered_tolerance_px=_finite(
                servo_raw.get("centered_tolerance_px"),
                "camera.visual_servo.centered_tolerance_px",
                positive=True,
            ),
            acquisition_radius_px=_finite(
                servo_raw.get("acquisition_radius_px"),
                "camera.visual_servo.acquisition_radius_px",
                positive=True,
            ),
            ambiguity_margin_px=_finite(
                servo_raw.get("ambiguity_margin_px"),
                "camera.visual_servo.ambiguity_margin_px",
                positive=True,
            ),
            required_stable_frames=servo_raw.get("required_stable_frames"),
            max_iterations=servo_raw.get("max_iterations"),
        )
    visual_servo_verified = _boolean(
        servo_raw.get("verified"),
        "camera.visual_servo.verified",
    )
    if visual_servo_verified and visual_servo is None:
        raise ValueError("visual-servo calibration cannot be verified before measuring")
    timing = _mapping(root.get("observation", {}), "observation")
    expected_raw = timing.get(
        "expected_present_ids", [slot.slot_id for slot in layout.slots]
    )
    if not isinstance(expected_raw, list):
        raise ValueError("observation.expected_present_ids must be a list")
    expected = tuple(str(item) for item in expected_raw)
    known = {slot.slot_id for slot in layout.slots}
    if len(set(expected)) != len(expected) or not set(expected).issubset(known):
        raise ValueError("expected_present_ids must be unique known cap IDs")

    fixture_frame = str(fixture.get("frame_id", "battery_fixture")).strip()
    image_topic = str(camera.get("image_topic", "/camera/image_raw")).strip()
    info_topic = str(camera.get("camera_info_topic", "/camera/camera_info")).strip()
    if not fixture_frame or not image_topic or not info_topic:
        raise ValueError("fixture frame and camera topics cannot be empty")

    observation_pose_raw = timing.get("joint_positions_deg")
    if observation_pose_raw is None or (
        isinstance(observation_pose_raw, list)
        and observation_pose_raw
        and all(value is None for value in observation_pose_raw)
    ):
        observation_pose = None
    else:
        if not isinstance(observation_pose_raw, list) or len(observation_pose_raw) != 6:
            raise ValueError("observation.joint_positions_deg must contain six values")
        try:
            observation_pose = validate_servo_degrees(
                _finite(
                    value,
                    f"observation.joint_positions_deg[{index}]",
                )
                for index, value in enumerate(observation_pose_raw)
            )
        except ValueError as exc:
            raise ValueError(f"invalid observation pose: {exc}") from exc

    maximum_image_age_s = _commissioning_range(
        _finite(
            timing.get("maximum_image_age_s", 0.25),
            "observation.maximum_image_age_s",
            positive=True,
        ),
        "observation.maximum_image_age_s",
        0.05,
        1.0,
    )
    minimum_stability_duration_s = _commissioning_range(
        _finite(
            timing.get("minimum_stability_duration_s", 1.0),
            "observation.minimum_stability_duration_s",
            positive=True,
        ),
        "observation.minimum_stability_duration_s",
        0.5,
        30.0,
    )
    joint_settle_time_s = _commissioning_range(
        _finite(
            timing.get("joint_settle_time_s", 0.75),
            "observation.joint_settle_time_s",
            positive=True,
        ),
        "observation.joint_settle_time_s",
        0.25,
        30.0,
    )
    joint_motion_threshold_deg = _commissioning_range(
        _finite(
            timing.get("joint_motion_threshold_deg", 0.25),
            "observation.joint_motion_threshold_deg",
            positive=True,
        ),
        "observation.joint_motion_threshold_deg",
        0.01,
        1.0,
    )
    observation_pose_tolerance_deg = _commissioning_range(
        _finite(
            timing.get("pose_tolerance_deg", 0.5),
            "observation.pose_tolerance_deg",
            positive=True,
        ),
        "observation.pose_tolerance_deg",
        0.05,
        2.0,
    )

    return InspectionSettings(
        source_path=str(source),
        config_sha256=_canonical_hash(root),
        example_only=_boolean(root.get("example_only"), "example_only"),
        commissioned=_boolean(root.get("commissioned"), "commissioned"),
        fixture_frame_id=fixture_frame,
        camera_image_topic=image_topic,
        camera_info_topic=info_topic,
        require_calibrated_camera_info=_boolean(
            camera.get("require_calibrated_camera_info", True),
            "camera.require_calibrated_camera_info",
        ),
        localization_mode=localization_mode,
        camera_pose_reference=camera_pose_reference,
        camera_pose_reference_verified=camera_pose_reference_verified,
        fixed_jig_reference=fixed_jig_reference,
        fixed_jig_reference_verified=fixed_jig_reference_verified,
        visual_servo=visual_servo,
        visual_servo_verified=visual_servo_verified,
        maximum_image_age_s=maximum_image_age_s,
        minimum_stability_duration_s=minimum_stability_duration_s,
        joint_settle_time_s=joint_settle_time_s,
        joint_motion_threshold_deg=joint_motion_threshold_deg,
        observation_joint_positions_deg=observation_pose,
        observation_pose_tolerance_deg=observation_pose_tolerance_deg,
        expected_present_ids=expected,
        layout=layout,
        inspector=inspector,
        raw=root,
    )


def load_source_destination_mapping(
    settings: InspectionSettings,
    destination_ids: tuple[str, ...] | list[str],
) -> tuple[tuple[str, str], ...]:
    """Load an explicit, complete source-to-destination bijection.

    List ordering is not a safe motion contract.  The mapping is therefore
    keyed by the seven stable source IDs and must name every destination once.
    """

    geometry = _mapping(settings.raw.get("geometry"), "geometry")
    raw = _mapping(
        geometry.get("source_destination_map"),
        "geometry.source_destination_map",
    )
    source_ids = tuple(slot.slot_id for slot in settings.layout.slots)
    expected_sources = set(source_ids)
    if any(not isinstance(key, str) or not key.strip() for key in raw):
        raise ValueError("source_destination_map keys must be non-empty source IDs")
    if set(raw) != expected_sources:
        missing = sorted(expected_sources - set(raw))
        extra = sorted(set(raw) - expected_sources)
        raise ValueError(
            "source_destination_map must name every source exactly once; "
            f"missing={missing}, extra={extra}"
        )

    known_destinations = tuple(destination_ids)
    if (
        len(known_destinations) != len(set(known_destinations))
        or any(not isinstance(item, str) or not item for item in known_destinations)
    ):
        raise ValueError("destination IDs must be unique non-empty strings")
    mapped = tuple(raw[source_id] for source_id in source_ids)
    if any(not isinstance(item, str) or not item.strip() for item in mapped):
        raise ValueError("source_destination_map values must be destination IDs")
    if len(set(mapped)) != len(mapped) or set(mapped) != set(known_destinations):
        raise ValueError(
            "source_destination_map must use every destination exactly once"
        )
    return tuple(zip(source_ids, mapped))


def hardware_execution_blockers(settings: InspectionSettings) -> tuple[str, ...]:
    """List every known reason the configuration cannot drive hardware."""

    root = settings.raw
    blockers = []
    if settings.example_only:
        blockers.append("configuration is marked example_only")
    if not settings.commissioned:
        blockers.append("configuration is not commissioned")

    execution = root.get("execution")
    execution_permitted = False
    if isinstance(execution, Mapping):
        try:
            execution_permitted = _boolean(
                execution.get("hardware_execution_permitted"),
                "execution.hardware_execution_permitted",
            )
        except ValueError as exc:
            blockers.append(str(exc))
    if not execution_permitted:
        blockers.append("hardware_execution_permitted is false")
    if isinstance(execution, Mapping):
        if execution.get("backend") != "hardware":
            blockers.append("execution backend is not hardware")
        for key in ("require_runtime_manual_arm", "require_confirmation_each_pick"):
            try:
                enabled = _boolean(execution.get(key), f"execution.{key}")
            except ValueError as exc:
                blockers.append(str(exc))
            else:
                if not enabled:
                    blockers.append(f"execution.{key} must remain true")
    if not settings.require_calibrated_camera_info:
        blockers.append("calibrated CameraInfo is not required by configuration")
    if settings.localization_mode == "aruco":
        if not settings.inspector.require_fixture_pose:
            blockers.append("metric camera fixture-pose validation is not required")
        if settings.camera_pose_reference is None:
            blockers.append("camera fixture-pose reference is not measured")
        if not settings.camera_pose_reference_verified:
            blockers.append("camera fixture-pose reference is not verified")
    elif settings.localization_mode == "fixed_red_jig":
        if settings.fixed_jig_reference is None:
            blockers.append("fixed red-jig image reference is not taught")
        if not settings.fixed_jig_reference_verified:
            blockers.append("fixed red-jig image reference is not verified")
    else:
        blockers.append(
            "a one-dimensional cap row cannot validate the battery's full "
            "2D fixture pose"
        )
    if settings.visual_servo is None:
        blockers.append("camera-to-tool visual-servo mapping is not measured")
    if not settings.visual_servo_verified:
        blockers.append("camera-to-tool visual-servo mapping is not verified")
    if settings.observation_joint_positions_deg is None:
        blockers.append("stationary camera observation pose is not measured")

    gripper = root.get("gripper")
    if not isinstance(gripper, Mapping):
        blockers.append("gripper measurements are missing")
    else:
        if gripper.get("physically_measured") is not True:
            blockers.append("gripper endpoint is not physically measured")
        try:
            open_degrees = _finite(gripper.get("open_degrees"), "gripper.open")
            close_degrees = _finite(gripper.get("close_degrees"), "gripper.close")
        except ValueError as exc:
            blockers.append(str(exc))
        else:
            if open_degrees != GRIPPER_OPEN_DEGREES:
                blockers.append("gripper open endpoint is not canonical 0 degrees")
            if close_degrees <= open_degrees:
                blockers.append("gripper close endpoint must be above open")
            configured_endpoint = os.environ.get(
                "RASHKO_GRIPPER_CLOSED_DEGREES", ""
            ).strip()
            try:
                bridge_endpoint = _environment_number(
                    configured_endpoint,
                    "RASHKO_GRIPPER_CLOSED_DEGREES",
                )
            except ValueError:
                blockers.append(
                    "RASHKO_GRIPPER_CLOSED_DEGREES is not calibrated"
                )
            else:
                if not math.isclose(
                    close_degrees, bridge_endpoint, abs_tol=1e-6
                ):
                    blockers.append(
                        "gripper close endpoint differs from the bridge endpoint"
                    )

    required_paths = (
        ("geometry.fixture_in_base.x_m", ("geometry", "fixture_in_base", "x_m")),
        ("geometry.fixture_in_base.y_m", ("geometry", "fixture_in_base", "y_m")),
        ("geometry.fixture_in_base.z_m", ("geometry", "fixture_in_base", "z_m")),
        ("geometry.fixture_in_base.yaw_deg", ("geometry", "fixture_in_base", "yaw_deg")),
        ("geometry.battery.x_min_m", ("geometry", "battery", "x_min_m")),
        ("geometry.battery.x_max_m", ("geometry", "battery", "x_max_m")),
        ("geometry.battery.y_min_m", ("geometry", "battery", "y_min_m")),
        ("geometry.battery.y_max_m", ("geometry", "battery", "y_max_m")),
        ("geometry.safe_transit_z_base_m", ("geometry", "safe_transit_z_base_m")),
        ("geometry.gripper_outer_width_m", ("geometry", "gripper_outer_width_m")),
    )
    for label, keys in required_paths:
        value: Any = root
        for key in keys:
            value = value.get(key) if isinstance(value, Mapping) else None
        try:
            _finite(value, label)
        except ValueError:
            blockers.append(f"{label} is not measured")
    return tuple(dict.fromkeys(blockers))
