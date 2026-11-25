"""Unverified Cartesian safety sketches and conservative geometric gates.

These point/corridor checks are not robot collision checking.  Any future motion
executor must still validate the complete arm, tool, attached cap, and workcell
in a MoveIt PlanningScene before sending a trajectory.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping, Optional, Sequence

from arm_controller.cap_workcell_config import InspectionSettings


FIXTURE_COORDINATE_CONVENTION = "right_handed_z_up"


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class ProtectedCircle:
    region_id: str
    x_m: float
    y_m: float
    radius_m: float
    top_z_fixture_m: float


@dataclass(frozen=True)
class DestinationSlot:
    slot_id: str
    point_base: Point3


@dataclass(frozen=True)
class MotionGeometry:
    fixture_x_base_m: float
    fixture_y_base_m: float
    fixture_z_base_m: float
    fixture_yaw_rad: float
    battery_x_min_m: float
    battery_x_max_m: float
    battery_y_min_m: float
    battery_y_max_m: float
    battery_bottom_z_fixture_m: float
    battery_top_z_fixture_m: float
    safe_transit_z_base_m: float
    grasp_tool_z_base_m: float
    gripper_outer_width_m: float
    cap_radius_m: float
    pose_uncertainty_m: float
    collision_padding_m: float
    minimum_vertical_clearance_m: float
    tool_payload_below_tcp_m: float
    fixture_plane_is_level: bool
    marker_plane_at_cap_top: bool
    protected_regions_measured: bool
    destination_clearance_verified: bool
    protected_regions: tuple[ProtectedCircle, ...]
    destinations: tuple[DestinationSlot, ...]

    @property
    def corridor_radius_m(self) -> float:
        return (
            self.gripper_outer_width_m / 2.0
            + self.pose_uncertainty_m
            + self.collision_padding_m
        )

    @property
    def battery_top_z_base_m(self) -> float:
        return self.fixture_z_base_m + self.battery_top_z_fixture_m

    def fixture_xy_to_base(self, x_m: float, y_m: float) -> tuple[float, float]:
        cosine = math.cos(self.fixture_yaw_rad)
        sine = math.sin(self.fixture_yaw_rad)
        return (
            self.fixture_x_base_m + cosine * x_m - sine * y_m,
            self.fixture_y_base_m + sine * x_m + cosine * y_m,
        )

    def base_xy_to_fixture(self, x_m: float, y_m: float) -> tuple[float, float]:
        delta_x = x_m - self.fixture_x_base_m
        delta_y = y_m - self.fixture_y_base_m
        cosine = math.cos(self.fixture_yaw_rad)
        sine = math.sin(self.fixture_yaw_rad)
        return (
            cosine * delta_x + sine * delta_y,
            -sine * delta_x + cosine * delta_y,
        )


@dataclass(frozen=True)
class PickPreview:
    source_slot_id: str
    destination_slot_id: str
    pregrasp: Point3
    grasp: Point3
    lift: Point3
    place_above: Point3
    place: Point3
    retreat: Point3
    corridor_radius_m: float

    @property
    def staged_path(self) -> tuple[Point3, ...]:
        return (
            self.pregrasp,
            self.grasp,
            self.lift,
            self.place_above,
            self.place,
            self.retreat,
        )


def _map(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{path} must be a mapping")
    return value


def _number(value: Any, path: str, *, positive: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{path} is not a physical number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{path} is not physically measured") from exc
    if not math.isfinite(result):
        raise ValueError(f"{path} must be finite")
    if positive and result <= 0.0:
        raise ValueError(f"{path} must be positive")
    return result


def _attestation(value: Any, path: str) -> bool:
    if value is not True:
        raise ValueError(f"{path} must be physically verified true")
    return True


def require_right_handed_fixture(settings: InspectionSettings) -> None:
    """Reject legacy fixture files whose printed Y axis can mirror base targets."""

    fixture = _map(settings.raw.get("fixture"), "fixture")
    convention = fixture.get("coordinate_convention")
    if convention != FIXTURE_COORDINATE_CONVENTION:
        raise ValueError(
            "fixture.coordinate_convention must be "
            f"{FIXTURE_COORDINATE_CONVENTION!r}; +X is right, +Y is toward the "
            "top of the printed fixture, and +Z is out of the fixture plane"
        )


def load_motion_geometry(settings: InspectionSettings) -> MotionGeometry:
    """Load measurements required for the unverified Cartesian safety sketch."""

    require_right_handed_fixture(settings)
    geometry = _map(settings.raw.get("geometry"), "geometry")
    fixture = _map(geometry.get("fixture_in_base"), "geometry.fixture_in_base")
    battery = _map(geometry.get("battery"), "geometry.battery")

    protected = []
    protected_raw = geometry.get("protected_regions", [])
    if not isinstance(protected_raw, list):
        raise ValueError("geometry.protected_regions must be a list")
    for index, item in enumerate(protected_raw):
        region = _map(item, f"geometry.protected_regions[{index}]")
        region_id = str(region.get("id", "")).strip()
        if not region_id:
            raise ValueError("every protected region needs an ID")
        protected.append(
            ProtectedCircle(
                region_id,
                _number(region.get("x_m"), f"{region_id}.x_m"),
                _number(region.get("y_m"), f"{region_id}.y_m"),
                _number(
                    region.get("radius_m"),
                    f"{region_id}.radius_m",
                    positive=True,
                ),
                _number(
                    region.get("top_z_fixture_m"),
                    f"{region_id}.top_z_fixture_m",
                ),
            )
        )
    if len({item.region_id for item in protected}) != len(protected):
        raise ValueError("protected region IDs must be unique")

    destinations_raw = geometry.get("destination_slots")
    if not isinstance(destinations_raw, list) or len(destinations_raw) != 7:
        raise ValueError("geometry.destination_slots must define exactly seven slots")
    destinations = []
    for index, item in enumerate(destinations_raw):
        slot = _map(item, f"geometry.destination_slots[{index}]")
        slot_id = str(slot.get("id", "")).strip()
        if not slot_id:
            raise ValueError("every destination slot needs an ID")
        destinations.append(
            DestinationSlot(
                slot_id,
                Point3(
                    _number(slot.get("x_base_m"), f"{slot_id}.x_base_m"),
                    _number(slot.get("y_base_m"), f"{slot_id}.y_base_m"),
                    _number(slot.get("z_base_m"), f"{slot_id}.z_base_m"),
                ),
            )
        )
    if len({item.slot_id for item in destinations}) != 7:
        raise ValueError("destination slot IDs must be unique")

    result = MotionGeometry(
        fixture_x_base_m=_number(fixture.get("x_m"), "fixture_in_base.x_m"),
        fixture_y_base_m=_number(fixture.get("y_m"), "fixture_in_base.y_m"),
        fixture_z_base_m=_number(fixture.get("z_m"), "fixture_in_base.z_m"),
        fixture_yaw_rad=math.radians(
            _number(fixture.get("yaw_deg"), "fixture_in_base.yaw_deg")
        ),
        battery_x_min_m=_number(battery.get("x_min_m"), "battery.x_min_m"),
        battery_x_max_m=_number(battery.get("x_max_m"), "battery.x_max_m"),
        battery_y_min_m=_number(battery.get("y_min_m"), "battery.y_min_m"),
        battery_y_max_m=_number(battery.get("y_max_m"), "battery.y_max_m"),
        battery_bottom_z_fixture_m=_number(
            battery.get("bottom_z_fixture_m"), "battery.bottom_z_fixture_m"
        ),
        battery_top_z_fixture_m=_number(
            battery.get("top_z_fixture_m"), "battery.top_z_fixture_m"
        ),
        safe_transit_z_base_m=_number(
            geometry.get("safe_transit_z_base_m"),
            "geometry.safe_transit_z_base_m",
        ),
        grasp_tool_z_base_m=_number(
            geometry.get("grasp_tool_z_base_m"),
            "geometry.grasp_tool_z_base_m",
        ),
        gripper_outer_width_m=_number(
            geometry.get("gripper_outer_width_m"),
            "geometry.gripper_outer_width_m",
            positive=True,
        ),
        cap_radius_m=_number(
            geometry.get("cap_radius_m"),
            "geometry.cap_radius_m",
            positive=True,
        ),
        pose_uncertainty_m=_number(
            geometry.get("pose_uncertainty_m", 0.003),
            "geometry.pose_uncertainty_m",
            positive=True,
        ),
        collision_padding_m=_number(
            geometry.get("collision_padding_m", 0.002),
            "geometry.collision_padding_m",
            positive=True,
        ),
        minimum_vertical_clearance_m=_number(
            geometry.get("minimum_vertical_clearance_m", 0.040),
            "geometry.minimum_vertical_clearance_m",
            positive=True,
        ),
        tool_payload_below_tcp_m=_number(
            geometry.get("tool_payload_below_tcp_m"),
            "geometry.tool_payload_below_tcp_m",
            positive=True,
        ),
        fixture_plane_is_level=_attestation(
            geometry.get("fixture_plane_is_level"),
            "geometry.fixture_plane_is_level",
        ),
        marker_plane_at_cap_top=_attestation(
            geometry.get("marker_plane_at_cap_top"),
            "geometry.marker_plane_at_cap_top",
        ),
        protected_regions_measured=_attestation(
            geometry.get("protected_regions_measured"),
            "geometry.protected_regions_measured",
        ),
        destination_clearance_verified=_attestation(
            geometry.get("destination_clearance_verified"),
            "geometry.destination_clearance_verified",
        ),
        protected_regions=tuple(protected),
        destinations=tuple(destinations),
    )
    _validate_motion_geometry(result, settings)
    return result


def _validate_motion_geometry(
    geometry: MotionGeometry, settings: InspectionSettings
) -> None:
    if geometry.battery_x_max_m <= geometry.battery_x_min_m:
        raise ValueError("battery X bounds are reversed or empty")
    if geometry.battery_y_max_m <= geometry.battery_y_min_m:
        raise ValueError("battery Y bounds are reversed or empty")
    if geometry.battery_top_z_fixture_m <= geometry.battery_bottom_z_fixture_m:
        raise ValueError("battery Z bounds are reversed or empty")
    largest_slot_radius_m = max(
        slot.expected_radius_m for slot in settings.layout.slots
    )
    if geometry.cap_radius_m < largest_slot_radius_m:
        raise ValueError(
            "geometry.cap_radius_m is smaller than a configured cap-slot radius"
        )
    for region in geometry.protected_regions:
        if region.top_z_fixture_m < geometry.battery_bottom_z_fixture_m:
            raise ValueError(
                f"protected region {region.region_id} top is below the battery bottom"
            )
    obstacle_tops_base_m = (geometry.battery_top_z_base_m,) + tuple(
        geometry.fixture_z_base_m + region.top_z_fixture_m
        for region in geometry.protected_regions
    )
    highest_obstacle_top_base_m = max(obstacle_tops_base_m)
    required_transit = (
        highest_obstacle_top_base_m
        + geometry.minimum_vertical_clearance_m
        + geometry.tool_payload_below_tcp_m
    )
    if geometry.safe_transit_z_base_m < required_transit:
        raise ValueError(
            "safe transit Z is below the highest battery/protected-region top "
            "plus vertical clearance and the measured below-TCP tool/payload envelope"
        )
    if not (
        geometry.battery_top_z_base_m
        <= geometry.grasp_tool_z_base_m
        < geometry.safe_transit_z_base_m
    ):
        raise ValueError(
            "grasp tool Z must be at/above the battery top and below transit Z"
        )

    for slot in settings.layout.slots:
        # Prove the entire configured localization-error disk safe, not merely
        # the nominal point. Runtime centroids are checked again before a sketch.
        _validate_source_corridor(
            settings,
            geometry,
            slot.slot_id,
            slot.x_m,
            slot.y_m,
            uncertainty_margin_m=slot.max_center_error_m,
        )

    destination_required_spacing = (
        geometry.corridor_radius_m
        + geometry.cap_radius_m
        + geometry.collision_padding_m
    )
    for index, destination in enumerate(geometry.destinations):
        point = destination.point_base
        if not point.z < geometry.safe_transit_z_base_m:
            raise ValueError(
                f"{destination.slot_id} place Z must be below safe transit Z"
            )
        if (
            geometry.safe_transit_z_base_m - point.z
            < geometry.minimum_vertical_clearance_m
        ):
            raise ValueError(
                f"{destination.slot_id} has insufficient vertical approach "
                "clearance below safe transit Z"
            )
        fixture_x, fixture_y = geometry.base_xy_to_fixture(point.x, point.y)
        inside_expanded_battery = (
            geometry.battery_x_min_m - geometry.corridor_radius_m
            <= fixture_x
            <= geometry.battery_x_max_m + geometry.corridor_radius_m
            and geometry.battery_y_min_m - geometry.corridor_radius_m
            <= fixture_y
            <= geometry.battery_y_max_m + geometry.corridor_radius_m
        )
        if inside_expanded_battery:
            raise ValueError(
                f"{destination.slot_id} vertical placement corridor overlaps "
                "the battery footprint"
            )
        for region in geometry.protected_regions:
            distance = math.hypot(fixture_x - region.x_m, fixture_y - region.y_m)
            required = geometry.corridor_radius_m + region.radius_m
            if distance < required:
                raise ValueError(
                    f"{destination.slot_id} vertical placement corridor "
                    f"intersects protected region {region.region_id}"
                )
        for other in geometry.destinations[index + 1:]:
            distance = math.hypot(
                point.x - other.point_base.x,
                point.y - other.point_base.y,
            )
            if distance < destination_required_spacing:
                raise ValueError(
                    f"destination slots {destination.slot_id} and "
                    f"{other.slot_id} are too close for the gripper corridor"
                )


def _validate_source_corridor(
    settings: InspectionSettings,
    geometry: MotionGeometry,
    source_slot_id: str,
    source_x_m: float,
    source_y_m: float,
    *,
    uncertainty_margin_m: float,
) -> None:
    required_corridor = geometry.corridor_radius_m + uncertainty_margin_m
    edge_clearance = min(
        source_x_m - geometry.battery_x_min_m,
        geometry.battery_x_max_m - source_x_m,
        source_y_m - geometry.battery_y_min_m,
        geometry.battery_y_max_m - source_y_m,
    )
    if edge_clearance < required_corridor:
        raise ValueError(
            f"{source_slot_id} has only {edge_clearance:.4f} m edge clearance; "
            f"needs {required_corridor:.4f} m for its gripper corridor"
        )
    for region in geometry.protected_regions:
        distance = math.hypot(source_x_m - region.x_m, source_y_m - region.y_m)
        required = required_corridor + region.radius_m
        if distance < required:
            raise ValueError(
                f"{source_slot_id} grasp corridor intersects protected "
                f"region {region.region_id}"
            )
    for neighbour in settings.layout.slots:
        if neighbour.slot_id == source_slot_id:
            continue
        distance = math.hypot(
            source_x_m - neighbour.x_m,
            source_y_m - neighbour.y_m,
        )
        required = (
            required_corridor
            + geometry.cap_radius_m
            + geometry.collision_padding_m
        )
        if distance < required:
            raise ValueError(
                f"gripper corridor at {source_slot_id} can strike "
                f"{neighbour.slot_id}; spacing={distance:.4f} m, "
                f"required={required:.4f} m"
            )


def motion_geometry_blockers(settings: InspectionSettings) -> tuple[str, ...]:
    """Return a single actionable geometry blocker without guessing values."""

    try:
        load_motion_geometry(settings)
    except ValueError as exc:
        return (str(exc),)
    return ()


def build_pick_preview(
    settings: InspectionSettings,
    geometry: MotionGeometry,
    source_slot_id: str,
    destination_slot_id: str,
    observed_source_xy_m: Optional[Sequence[float]] = None,
) -> PickPreview:
    """Construct an unverified high/vertical/high Cartesian safety sketch.

    The returned points do not prove IK feasibility or collision freedom for any
    arm link.  A full MoveIt PlanningScene check remains mandatory before motion.
    """

    slots = {slot.slot_id: slot for slot in settings.layout.slots}
    if source_slot_id not in slots:
        raise ValueError(f"unknown source slot {source_slot_id!r}")
    destinations = {slot.slot_id: slot for slot in geometry.destinations}
    if destination_slot_id not in destinations:
        raise ValueError(f"unknown destination slot {destination_slot_id!r}")
    nominal = slots[source_slot_id]
    source_x, source_y = nominal.x_m, nominal.y_m
    if observed_source_xy_m is not None:
        if len(observed_source_xy_m) != 2:
            raise ValueError("observed source coordinate must be XY")
        source_x = _number(observed_source_xy_m[0], "observed source X")
        source_y = _number(observed_source_xy_m[1], "observed source Y")
        deviation = math.hypot(source_x - nominal.x_m, source_y - nominal.y_m)
        if deviation > nominal.max_center_error_m:
            raise ValueError(
                f"observed {source_slot_id} is {deviation:.4f} m from its slot"
            )
    _validate_source_corridor(
        settings,
        geometry,
        source_slot_id,
        source_x,
        source_y,
        uncertainty_margin_m=0.0,
    )
    source_base_x, source_base_y = geometry.fixture_xy_to_base(
        source_x, source_y
    )
    destination = destinations[destination_slot_id].point_base
    safe_z = geometry.safe_transit_z_base_m
    preview = PickPreview(
        source_slot_id=source_slot_id,
        destination_slot_id=destination_slot_id,
        pregrasp=Point3(source_base_x, source_base_y, safe_z),
        grasp=Point3(
            source_base_x, source_base_y, geometry.grasp_tool_z_base_m
        ),
        lift=Point3(source_base_x, source_base_y, safe_z),
        place_above=Point3(destination.x, destination.y, safe_z),
        place=destination,
        retreat=Point3(destination.x, destination.y, safe_z),
        corridor_radius_m=geometry.corridor_radius_m,
    )
    assert_no_low_lateral_motion(preview.staged_path, safe_z)
    return preview


def assert_no_low_lateral_motion(
    points: Sequence[Point3], safe_transit_z_m: float, tolerance_m: float = 1e-6
) -> None:
    """Reject any path segment that moves laterally below the transit plane."""

    if len(points) < 2:
        raise ValueError("a staged path needs at least two points")
    for start, end in zip(points, points[1:]):
        lateral = math.hypot(end.x - start.x, end.y - start.y)
        if lateral > tolerance_m and (
            start.z < safe_transit_z_m - tolerance_m
            or end.z < safe_transit_z_m - tolerance_m
        ):
            raise ValueError(
                "lateral motion is forbidden below the safe transit plane"
            )
