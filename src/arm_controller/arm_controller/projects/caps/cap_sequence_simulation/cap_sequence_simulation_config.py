"""Strict loader for the synthetic, offline-only seven-cap scene."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from numbers import Real
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from arm_controller.projects.caps.workcell.red_cap_visual_servo import VisualServoConfig


CONFIGURATION_KIND = "cap_sequence_simulation"
SCHEMA_VERSION = 1
_GEOMETRY_TOLERANCE_M = 1e-9


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that refuses ambiguous duplicate mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    loader.flatten_mapping(node)
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in result
        except TypeError as exc:
            raise ValueError("YAML mapping keys must be hashable") from exc
        if duplicate:
            raise ValueError(f"duplicate YAML mapping key {key!r}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


@dataclass(frozen=True)
class Point3:
    """A point in the configured MoveIt planning frame, in metres."""

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class BoxVolume:
    """An axis-aligned synthetic collision box."""

    object_id: str
    center: Point3
    size: Point3

    @property
    def x_min(self) -> float:
        return self.center.x - self.size.x / 2.0

    @property
    def x_max(self) -> float:
        return self.center.x + self.size.x / 2.0

    @property
    def y_min(self) -> float:
        return self.center.y - self.size.y / 2.0

    @property
    def y_max(self) -> float:
        return self.center.y + self.size.y / 2.0

    @property
    def bottom_z(self) -> float:
        return self.center.z - self.size.z / 2.0

    @property
    def top_z(self) -> float:
        return self.center.z + self.size.z / 2.0


@dataclass(frozen=True)
class CylinderVolume:
    """An upright synthetic collision cylinder."""

    object_id: str
    center: Point3
    radius_m: float
    height_m: float

    @property
    def bottom_z(self) -> float:
        return self.center.z - self.height_m / 2.0

    @property
    def top_z(self) -> float:
        return self.center.z + self.height_m / 2.0


@dataclass(frozen=True)
class CapGeometry:
    """Shared dimensions of all seven synthetic caps."""

    radius_m: float
    height_m: float


@dataclass(frozen=True)
class CapSceneSlot:
    """Stable source or destination ID and its synthetic cap centre."""

    slot_id: str
    cap_center: Point3


@dataclass(frozen=True)
class VisualServoSimulationSettings:
    """Synthetic eye-in-hand camera and bounded alignment configuration."""

    enabled: bool
    image_size_px: tuple[int, int]
    cap_radius_px: int
    metres_per_pixel_xy: tuple[float, float]
    coarse_offsets_xy_m: tuple[tuple[str, tuple[float, float]], ...]
    controller: VisualServoConfig


@dataclass(frozen=True)
class CapSequenceSimulationConfig:
    """Fully validated configuration for offline seven-cap simulation."""

    source_path: str
    config_sha256: str
    simulation_only: bool
    synthetic_scene: bool
    hardware_execution_permitted: bool
    frame_id: str
    table: BoxVolume
    battery: BoxVolume
    protected_objects: tuple[CylinderVolume, ...]
    tray: BoxVolume
    cap: CapGeometry
    sources: tuple[CapSceneSlot, ...]
    destinations: tuple[CapSceneSlot, ...]
    source_destination_pairs: tuple[tuple[str, str], ...]
    safe_transit_z_m: float
    grasp_tool_z_m: float
    place_tool_z_m: float
    minimum_vertical_clearance_m: float
    tool_payload_below_tcp_m: float
    collision_padding_m: float
    tool_elevation_deg: float
    gripper_open_degrees: float
    gripper_closed_degrees: float
    visual_servo: VisualServoSimulationSettings


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{path} must be a mapping")
    return value


def _keys(value: Mapping[str, Any], path: str, expected: set[str]) -> None:
    actual = set(value)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing or unknown:
        raise ValueError(
            f"{path} fields must match the simulation schema; "
            f"missing={missing}, unknown={unknown}"
        )


def _boolean(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{path} must be an unquoted YAML boolean")
    return value


def _number(value: Any, path: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{path} must be an unquoted finite YAML number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{path} must be finite")
    if positive and number <= 0.0:
        raise ValueError(f"{path} must be positive")
    return number


def _identifier(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{path} must be a non-empty, trimmed string")
    return value


def _vector3(
    value: Any,
    path: str,
    *,
    positive: bool = False,
) -> Point3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{path} must contain exactly three numbers")
    return Point3(
        *(
            _number(item, f"{path}[{index}]", positive=positive)
            for index, item in enumerate(value)
        )
    )


def _vector2(value: Any, path: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"{path} must contain exactly two numbers")
    return tuple(
        _number(item, f"{path}[{index}]")
        for index, item in enumerate(value)
    )


def _box(value: Any, path: str) -> BoxVolume:
    raw = _mapping(value, path)
    _keys(raw, path, {"id", "center_m", "size_m"})
    return BoxVolume(
        object_id=_identifier(raw["id"], f"{path}.id"),
        center=_vector3(raw["center_m"], f"{path}.center_m"),
        size=_vector3(raw["size_m"], f"{path}.size_m", positive=True),
    )


def _cylinder(value: Any, path: str) -> CylinderVolume:
    raw = _mapping(value, path)
    _keys(
        raw,
        path,
        {"id", "center_xy_m", "bottom_z_m", "radius_m", "height_m"},
    )
    center_xy = raw["center_xy_m"]
    if not isinstance(center_xy, (list, tuple)) or len(center_xy) != 2:
        raise ValueError(
            f"{path}.center_xy_m must contain exactly two numbers"
        )
    x = _number(center_xy[0], f"{path}.center_xy_m[0]")
    y = _number(center_xy[1], f"{path}.center_xy_m[1]")
    bottom_z = _number(raw["bottom_z_m"], f"{path}.bottom_z_m")
    height = _number(raw["height_m"], f"{path}.height_m", positive=True)
    return CylinderVolume(
        object_id=_identifier(raw["id"], f"{path}.id"),
        center=Point3(x, y, bottom_z + height / 2.0),
        radius_m=_number(raw["radius_m"], f"{path}.radius_m", positive=True),
        height_m=height,
    )


def _slot(value: Any, path: str, cap_center_z: float) -> CapSceneSlot:
    raw = _mapping(value, path)
    _keys(raw, path, {"id", "x_m", "y_m"})
    return CapSceneSlot(
        slot_id=_identifier(raw["id"], f"{path}.id"),
        cap_center=Point3(
            _number(raw["x_m"], f"{path}.x_m"),
            _number(raw["y_m"], f"{path}.y_m"),
            cap_center_z,
        ),
    )


def _sequence(value: Any, path: str) -> Sequence[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{path} must be a list")
    return value


def _canonical_hash(document: Mapping[str, Any]) -> str:
    encoded = json.dumps(
        document,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _contains_circle(box: BoxVolume, point: Point3, radius: float) -> bool:
    return (
        box.x_min <= point.x - radius
        and point.x + radius <= box.x_max
        and box.y_min <= point.y - radius
        and point.y + radius <= box.y_max
    )


def _contains_box_xy(outer: BoxVolume, inner: BoxVolume) -> bool:
    return (
        outer.x_min <= inner.x_min
        and inner.x_max <= outer.x_max
        and outer.y_min <= inner.y_min
        and inner.y_max <= outer.y_max
    )


def _validate_slot_clearance(
    slots: tuple[CapSceneSlot, ...],
    container: BoxVolume,
    cap: CapGeometry,
    padding: float,
    path: str,
) -> None:
    boundary_radius = cap.radius_m + padding
    for index, slot in enumerate(slots):
        if not _contains_circle(container, slot.cap_center, boundary_radius):
            raise ValueError(
                f"{path} {slot.slot_id} does not fit inside "
                f"{container.object_id} "
                "with collision padding"
            )
        for other in slots[index + 1:]:
            distance = math.hypot(
                slot.cap_center.x - other.cap_center.x,
                slot.cap_center.y - other.cap_center.y,
            )
            if distance < 2.0 * cap.radius_m + padding:
                raise ValueError(
                    f"{path} {slot.slot_id} and {other.slot_id} overlap"
                )


def load_simulation_config(path: str) -> CapSequenceSimulationConfig:
    """Load and validate the separate synthetic scene configuration.

    A commissioned or measured cap-workcell file cannot satisfy this schema.
    The returned object is guaranteed to be explicitly simulation-only and to
    contain no hardware execution permission.
    """

    source = Path(path)
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.load(stream, Loader=_UniqueKeyLoader)
    root = _mapping(document, "simulation configuration")
    _keys(
        root,
        "simulation configuration",
        {
            "schema_version",
            "configuration_kind",
            "simulation_only",
            "synthetic_scene",
            "frame_id",
            "execution",
            "scene",
            "motion",
            "gripper",
            "sources",
            "destinations",
            "source_destination_map",
            "visual_servo",
        },
    )
    if (
        type(root["schema_version"]) is not int
        or root["schema_version"] != SCHEMA_VERSION
    ):
        raise ValueError(f"schema_version must be {SCHEMA_VERSION}")
    if root["configuration_kind"] != CONFIGURATION_KIND:
        raise ValueError(f"configuration_kind must be {CONFIGURATION_KIND!r}")
    if _boolean(root["simulation_only"], "simulation_only") is not True:
        raise ValueError("simulation_only must remain true")
    if _boolean(root["synthetic_scene"], "synthetic_scene") is not True:
        raise ValueError("synthetic_scene must remain true")
    if _identifier(root["frame_id"], "frame_id") != "base_link":
        raise ValueError("frame_id must be 'base_link'")

    execution = _mapping(root["execution"], "execution")
    _keys(execution, "execution", {"backend", "hardware_execution_permitted"})
    if execution["backend"] != "simulation":
        raise ValueError("execution.backend must be 'simulation'")
    hardware_permitted = _boolean(
        execution["hardware_execution_permitted"],
        "execution.hardware_execution_permitted",
    )
    if hardware_permitted is not False:
        raise ValueError(
            "execution.hardware_execution_permitted must remain false"
        )

    scene = _mapping(root["scene"], "scene")
    _keys(
        scene,
        "scene",
        {
            "collision_padding_m",
            "table",
            "battery",
            "protected_objects",
            "tray",
            "cap",
        },
    )
    padding = _number(
        scene["collision_padding_m"],
        "scene.collision_padding_m",
        positive=True,
    )
    table = _box(scene["table"], "scene.table")
    battery = _box(scene["battery"], "scene.battery")
    tray = _box(scene["tray"], "scene.tray")
    cap_raw = _mapping(scene["cap"], "scene.cap")
    _keys(cap_raw, "scene.cap", {"radius_m", "height_m"})
    cap = CapGeometry(
        radius_m=_number(
            cap_raw["radius_m"], "scene.cap.radius_m", positive=True
        ),
        height_m=_number(
            cap_raw["height_m"], "scene.cap.height_m", positive=True
        ),
    )
    protected_raw = _sequence(
        scene["protected_objects"], "scene.protected_objects"
    )
    if not protected_raw:
        raise ValueError("scene.protected_objects must not be empty")
    protected = tuple(
        _cylinder(item, f"scene.protected_objects[{index}]")
        for index, item in enumerate(protected_raw)
    )

    scene_ids = [table.object_id, battery.object_id, tray.object_id]
    scene_ids.extend(item.object_id for item in protected)
    if len(scene_ids) != len(set(scene_ids)):
        raise ValueError("all collision object IDs must be unique")
    if (
        not _contains_box_xy(table, battery)
        or not _contains_box_xy(table, tray)
    ):
        raise ValueError("table must contain the battery and tray footprints")
    if not math.isclose(
        battery.bottom_z, table.top_z, abs_tol=_GEOMETRY_TOLERANCE_M
    ) or not math.isclose(
        tray.bottom_z, table.top_z, abs_tol=_GEOMETRY_TOLERANCE_M
    ):
        raise ValueError("battery and tray must rest on the table top")
    overlap_x = battery.x_min < tray.x_max and tray.x_min < battery.x_max
    overlap_y = battery.y_min < tray.y_max and tray.y_min < battery.y_max
    if overlap_x and overlap_y:
        raise ValueError("battery and tray footprints must not overlap")
    for index, item in enumerate(protected):
        if not math.isclose(
            item.bottom_z, battery.top_z, abs_tol=_GEOMETRY_TOLERANCE_M
        ):
            raise ValueError(
                f"protected object {item.object_id} must start at the "
                "battery top"
            )
        if not _contains_circle(
            battery, item.center, item.radius_m + padding
        ):
            raise ValueError(
                f"protected object {item.object_id} does not fit on the "
                "battery"
            )
        for other in protected[index + 1:]:
            distance = math.hypot(
                item.center.x - other.center.x,
                item.center.y - other.center.y,
            )
            if distance < item.radius_m + other.radius_m + padding:
                raise ValueError(
                    f"protected objects {item.object_id} and "
                    f"{other.object_id} overlap"
                )

    source_center_z = battery.top_z + cap.height_m / 2.0
    destination_center_z = tray.top_z + cap.height_m / 2.0
    source_raw = _sequence(root["sources"], "sources")
    destination_raw = _sequence(root["destinations"], "destinations")
    if len(source_raw) != 7 or len(destination_raw) != 7:
        raise ValueError(
            "sources and destinations must each contain exactly seven slots"
        )
    sources = tuple(
        _slot(item, f"sources[{index}]", source_center_z)
        for index, item in enumerate(source_raw)
    )
    destinations = tuple(
        _slot(item, f"destinations[{index}]", destination_center_z)
        for index, item in enumerate(destination_raw)
    )
    source_ids = tuple(item.slot_id for item in sources)
    destination_ids = tuple(item.slot_id for item in destinations)
    if len(set(source_ids)) != 7:
        raise ValueError("source IDs must be unique")
    if len(set(destination_ids)) != 7:
        raise ValueError("destination IDs must be unique")
    if set(source_ids) & set(destination_ids):
        raise ValueError("source and destination IDs must be disjoint")
    reserved_id_conflicts = sorted(
        (set(source_ids) | set(destination_ids)) & set(scene_ids)
    )
    if reserved_id_conflicts:
        raise ValueError(
            "slot IDs conflict with fixed collision object IDs: "
            f"{reserved_id_conflicts}"
        )
    _validate_slot_clearance(sources, battery, cap, padding, "source")
    _validate_slot_clearance(destinations, tray, cap, padding, "destination")
    for source_slot in sources:
        for obstacle in protected:
            distance = math.hypot(
                source_slot.cap_center.x - obstacle.center.x,
                source_slot.cap_center.y - obstacle.center.y,
            )
            if distance < cap.radius_m + obstacle.radius_m + padding:
                raise ValueError(
                    f"source {source_slot.slot_id} overlaps protected object "
                    f"{obstacle.object_id}"
                )

    mapping = _mapping(
        root["source_destination_map"], "source_destination_map"
    )
    if any(not isinstance(key, str) for key in mapping):
        raise ValueError("source_destination_map keys must be source IDs")
    if set(mapping) != set(source_ids):
        raise ValueError(
            "source_destination_map must name every source exactly once"
        )
    mapped_ids = tuple(mapping[source_id] for source_id in source_ids)
    if any(not isinstance(item, str) for item in mapped_ids):
        raise ValueError(
            "source_destination_map values must be destination IDs"
        )
    if len(set(mapped_ids)) != 7 or set(mapped_ids) != set(destination_ids):
        raise ValueError(
            "source_destination_map must use every destination exactly once"
        )

    visual_raw = _mapping(root["visual_servo"], "visual_servo")
    _keys(
        visual_raw,
        "visual_servo",
        {
            "enabled",
            "image_size_px",
            "target_pixel",
            "cap_radius_px",
            "metres_per_pixel_xy",
            "max_step_m",
            "centered_tolerance_px",
            "acquisition_radius_px",
            "ambiguity_margin_px",
            "required_stable_frames",
            "max_iterations",
            "coarse_offsets_xy_m",
        },
    )
    if _boolean(visual_raw["enabled"], "visual_servo.enabled") is not True:
        raise ValueError("visual_servo.enabled must remain true")
    image_size_raw = visual_raw["image_size_px"]
    if (
        not isinstance(image_size_raw, (list, tuple))
        or len(image_size_raw) != 2
        or any(type(item) is not int or item < 64 for item in image_size_raw)
    ):
        raise ValueError(
            "visual_servo.image_size_px must contain two integers >= 64"
        )
    image_size = (int(image_size_raw[0]), int(image_size_raw[1]))
    target_pixel = _vector2(
        visual_raw["target_pixel"], "visual_servo.target_pixel"
    )
    if not (
        0.0 <= target_pixel[0] < image_size[0]
        and 0.0 <= target_pixel[1] < image_size[1]
    ):
        raise ValueError("visual_servo.target_pixel must be inside the image")
    cap_radius_px = visual_raw["cap_radius_px"]
    if type(cap_radius_px) is not int or not 4 <= cap_radius_px <= 100:
        raise ValueError(
            "visual_servo.cap_radius_px must be an integer in [4, 100]"
        )
    metres_per_pixel = _vector2(
        visual_raw["metres_per_pixel_xy"],
        "visual_servo.metres_per_pixel_xy",
    )
    if any(not 1e-6 <= item <= 0.002 for item in metres_per_pixel):
        raise ValueError(
            "visual_servo metres-per-pixel values must be in [1e-6, 0.002]"
        )
    controller = VisualServoConfig(
        target_pixel=target_pixel,
        correction_matrix_m_per_px=(
            metres_per_pixel[0],
            0.0,
            0.0,
            -metres_per_pixel[1],
        ),
        max_step_m=_number(
            visual_raw["max_step_m"],
            "visual_servo.max_step_m",
            positive=True,
        ),
        centered_tolerance_px=_number(
            visual_raw["centered_tolerance_px"],
            "visual_servo.centered_tolerance_px",
            positive=True,
        ),
        acquisition_radius_px=_number(
            visual_raw["acquisition_radius_px"],
            "visual_servo.acquisition_radius_px",
            positive=True,
        ),
        ambiguity_margin_px=_number(
            visual_raw["ambiguity_margin_px"],
            "visual_servo.ambiguity_margin_px",
            positive=True,
        ),
        required_stable_frames=visual_raw["required_stable_frames"],
        max_iterations=visual_raw["max_iterations"],
    )
    offsets_raw = _mapping(
        visual_raw["coarse_offsets_xy_m"],
        "visual_servo.coarse_offsets_xy_m",
    )
    if set(offsets_raw) != set(source_ids):
        raise ValueError(
            "visual_servo coarse offsets must name every source exactly once"
        )
    offsets = tuple(
        (
            source_id,
            _vector2(
                offsets_raw[source_id],
                f"visual_servo.coarse_offsets_xy_m.{source_id}",
            ),
        )
        for source_id in source_ids
    )
    for source_id, offset in offsets:
        distance = math.hypot(*offset)
        maximum_acquisition_m = controller.acquisition_radius_px * min(
            metres_per_pixel
        )
        if distance >= maximum_acquisition_m:
            raise ValueError(
                f"visual_servo coarse offset for {source_id} leaves the "
                "camera acquisition radius"
            )
    visual_servo = VisualServoSimulationSettings(
        enabled=True,
        image_size_px=image_size,
        cap_radius_px=cap_radius_px,
        metres_per_pixel_xy=metres_per_pixel,
        coarse_offsets_xy_m=offsets,
        controller=controller,
    )

    motion = _mapping(root["motion"], "motion")
    _keys(
        motion,
        "motion",
        {
            "safe_transit_z_m",
            "grasp_tool_z_m",
            "place_tool_z_m",
            "minimum_vertical_clearance_m",
            "tool_payload_below_tcp_m",
            "tool_elevation_deg",
        },
    )
    safe_z = _number(
        motion["safe_transit_z_m"],
        "motion.safe_transit_z_m",
        positive=True,
    )
    grasp_z = _number(
        motion["grasp_tool_z_m"],
        "motion.grasp_tool_z_m",
    )
    place_z = _number(
        motion["place_tool_z_m"],
        "motion.place_tool_z_m",
    )
    clearance = _number(
        motion["minimum_vertical_clearance_m"],
        "motion.minimum_vertical_clearance_m",
        positive=True,
    )
    payload_below_tcp = _number(
        motion["tool_payload_below_tcp_m"],
        "motion.tool_payload_below_tcp_m",
        positive=True,
    )
    if payload_below_tcp < cap.height_m:
        raise ValueError(
            "motion.tool_payload_below_tcp_m must be at least the full "
            "cap height"
        )
    elevation = _number(
        motion["tool_elevation_deg"], "motion.tool_elevation_deg"
    )
    if not math.isclose(elevation, -90.0, abs_tol=1e-9):
        raise ValueError(
            "motion.tool_elevation_deg must be -90 for vertical picks"
        )

    source_cap_top = battery.top_z + cap.height_m
    destination_cap_top = tray.top_z + cap.height_m
    if grasp_z <= source_cap_top:
        raise ValueError("grasp tool Z must be above the source cap top")
    if place_z <= destination_cap_top:
        raise ValueError("place tool Z must be above the destination cap top")
    # The nominal transfer plane must keep the complete carried envelope clear
    # of support surfaces. Standing caps and terminals remain in the MoveIt
    # PlanningScene and are never ignored; OMPL must route around them.
    highest_scene_top = max(
        table.top_z,
        battery.top_z,
        tray.top_z,
        source_cap_top,
        destination_cap_top,
        *(item.top_z for item in protected),
    )
    required_safe_z = highest_scene_top + clearance + payload_below_tcp
    if safe_z + _GEOMETRY_TOLERANCE_M < required_safe_z:
        raise ValueError(
            "safe transit Z must clear every scene obstacle by the vertical "
            "clearance plus below-TCP envelope"
        )
    if safe_z - grasp_z < clearance or safe_z - place_z < clearance:
        raise ValueError(
            "grasp and place Z need minimum vertical approach clearance"
        )

    gripper = _mapping(root["gripper"], "gripper")
    _keys(gripper, "gripper", {"open_degrees", "closed_degrees"})
    open_degrees = _number(gripper["open_degrees"], "gripper.open_degrees")
    closed_degrees = _number(
        gripper["closed_degrees"], "gripper.closed_degrees"
    )
    if not 0.0 <= open_degrees < closed_degrees <= 180.0:
        raise ValueError(
            "simulation gripper endpoints must satisfy "
            "0 <= open < closed <= 180"
        )

    return CapSequenceSimulationConfig(
        source_path=str(source),
        config_sha256=_canonical_hash(root),
        simulation_only=True,
        synthetic_scene=True,
        hardware_execution_permitted=False,
        frame_id="base_link",
        table=table,
        battery=battery,
        protected_objects=protected,
        tray=tray,
        cap=cap,
        sources=sources,
        destinations=destinations,
        source_destination_pairs=tuple(zip(source_ids, mapped_ids)),
        safe_transit_z_m=safe_z,
        grasp_tool_z_m=grasp_z,
        place_tool_z_m=place_z,
        minimum_vertical_clearance_m=clearance,
        tool_payload_below_tcp_m=payload_below_tcp,
        collision_padding_m=padding,
        tool_elevation_deg=elevation,
        gripper_open_degrees=open_degrees,
        gripper_closed_degrees=closed_degrees,
        visual_servo=visual_servo,
    )
