"""Motion-free RViz sketch of the seven staged cap transfers.

This node intentionally has no action client and no arm-command publisher.  It
turns a stable inspection result into conservative high/vertical/high paths so
the measured battery envelope, gripper corridor, and all seven transfers can be
reviewed before any independently verified planner or executor exists.
"""

from __future__ import annotations

import json
import math
import os
from typing import Any, Mapping

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from arm_controller.projects.caps.workcell.cap_motion_safety import (
    MotionGeometry,
    Point3,
    build_pick_preview,
    load_motion_geometry,
)
from arm_controller.projects.caps.workcell.cap_workcell_config import (
    load_inspection_settings,
    load_source_destination_mapping,
)


class CapPickPreviewNode(Node):
    """Publish an unverified Cartesian safety sketch without moving anything."""

    def __init__(self) -> None:
        super().__init__("cap_pick_preview")
        default_config = os.environ.get(
            "RASHKO_CAP_WORKCELL_CONFIG",
            "/workspace/config/cap_workcell.example.yaml",
        )
        config_file = str(
            self.declare_parameter("config_file", default_config).value
        ).strip()
        if not config_file:
            raise ValueError("config_file cannot be empty")
        self.settings = load_inspection_settings(config_file)
        self.geometry: MotionGeometry | None = None
        self.geometry_blocker = ""
        self.source_destination_pairs: tuple[tuple[str, str], ...] = ()
        self.active_valid_until_ros_ns: int | None = None
        try:
            self.geometry = load_motion_geometry(self.settings)
            self.source_destination_pairs = load_source_destination_mapping(
                self.settings,
                [item.slot_id for item in self.geometry.destinations],
            )
        except ValueError as exc:
            self.geometry_blocker = str(exc)

        self.marker_pub = self.create_publisher(
            MarkerArray, "/cap_workcell/pick_preview_markers", 10
        )
        self.status_pub = self.create_publisher(
            String, "/cap_workcell/pick_preview", 10
        )
        self.create_subscription(
            String, "/cap_workcell/status", self._status_callback, 10
        )
        self.create_timer(0.05, self._lease_watchdog)
        self.static_broadcaster = None
        if self.geometry is not None:
            self._publish_fixture_transform(self.geometry)
        if self.geometry is None or not self.source_destination_pairs:
            self.get_logger().warning(
                "Cartesian safety sketch locked until configuration is complete: "
                + self.geometry_blocker
            )
        self.get_logger().warning(
            "Cartesian safety sketch is UNVERIFIED VISUALIZATION ONLY; "
            "it has no motion interface and performs no collision planning. "
            "Any executor requires a full MoveIt PlanningScene check"
        )

    def _publish_fixture_transform(self, geometry: MotionGeometry) -> None:
        if self.settings.fixture_frame_id == "base_link":
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = self.settings.fixture_frame_id
        transform.transform.translation.x = geometry.fixture_x_base_m
        transform.transform.translation.y = geometry.fixture_y_base_m
        transform.transform.translation.z = geometry.fixture_z_base_m
        transform.transform.rotation.z = math.sin(geometry.fixture_yaw_rad / 2.0)
        transform.transform.rotation.w = math.cos(geometry.fixture_yaw_rad / 2.0)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(transform)

    def _status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, Mapping):
                raise ValueError("inspection status must be a JSON object")
        except (json.JSONDecodeError, ValueError) as exc:
            self._publish_locked(f"invalid inspection status: {exc}")
            return

        if payload.get("config_sha256") != self.settings.config_sha256:
            self._publish_locked("inspection and preview configuration hashes differ")
            return
        if self.geometry is None:
            self._publish_locked(self.geometry_blocker)
            return
        if not self.source_destination_pairs:
            self._publish_locked(self.geometry_blocker)
            return
        if payload.get("vision_stable") is not True:
            self._publish_locked(str(payload.get("reason", "vision not stable")))
            return

        try:
            valid_until_ns = payload.get("valid_until_ros_ns")
            if type(valid_until_ns) is not int:
                raise ValueError("inspection status has no integer validity lease")
            if self.get_clock().now().nanoseconds >= valid_until_ns:
                raise ValueError("inspection status lease has expired")
            observed = self._observed_centres(payload)
            present_raw = payload.get("observed_present_ids")
            if not isinstance(present_raw, list) or any(
                not isinstance(item, str) for item in present_raw
            ):
                raise ValueError("observed_present_ids is invalid")
            present_ids = set(present_raw)
            known_ids = {slot.slot_id for slot in self.settings.layout.slots}
            if len(present_ids) != len(present_raw) or not present_ids <= known_ids:
                raise ValueError("observed cap IDs are duplicated or unknown")
            previews = []
            for source_id, destination_id in self.source_destination_pairs:
                if source_id not in present_ids:
                    continue
                if source_id not in observed:
                    raise ValueError(
                        f"stable result has no centroid for {source_id}"
                    )
                previews.append(
                    build_pick_preview(
                        self.settings,
                        self.geometry,
                        source_id,
                        destination_id,
                        observed[source_id],
                    )
                )
            if self.get_clock().now().nanoseconds >= valid_until_ns:
                raise ValueError(
                    "inspection status lease expired while building the sketch"
                )
        except (TypeError, ValueError) as exc:
            self._publish_locked(f"preview rejected: {exc}")
            return

        if previews:
            self.active_valid_until_ros_ns = valid_until_ns
            reason = "unverified Cartesian safety sketch available"
        else:
            self.active_valid_until_ros_ns = None
            reason = "stable inspection reports no caps remaining"
        self._publish_preview(
            tuple(previews),
            reason,
            valid_until_ros_ns=valid_until_ns,
        )

    @staticmethod
    def _observed_centres(payload: Mapping[str, Any]) -> dict[str, tuple[float, float]]:
        raw = payload.get("slot_observations")
        if not isinstance(raw, list):
            raise ValueError("slot_observations is missing")
        result: dict[str, tuple[float, float]] = {}
        for item in raw:
            if not isinstance(item, Mapping) or item.get("occupancy") != "present":
                continue
            slot_id = str(item.get("id", "")).strip()
            centroid = item.get("centroid_m")
            if (
                not slot_id
                or not isinstance(centroid, list)
                or len(centroid) != 2
            ):
                raise ValueError("present slot has an invalid ID or centroid")
            x_value = float(centroid[0])
            y_value = float(centroid[1])
            if not math.isfinite(x_value) or not math.isfinite(y_value):
                raise ValueError(f"{slot_id} centroid is non-finite")
            if slot_id in result:
                raise ValueError(f"duplicate slot observation {slot_id}")
            result[slot_id] = (x_value, y_value)
        return result

    def _publish_locked(self, reason: str) -> None:
        self.active_valid_until_ros_ns = None
        self._publish_preview((), reason)

    def _lease_watchdog(self) -> None:
        lease_ns = self.active_valid_until_ros_ns
        if lease_ns is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns >= lease_ns:
            self.active_valid_until_ros_ns = None
            self._publish_preview(
                (),
                "inspection status lease expired; Cartesian sketch cleared",
                valid_until_ros_ns=now_ns,
            )

    def _publish_preview(
        self,
        previews,
        reason: str,
        *,
        valid_until_ros_ns: int | None = None,
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if valid_until_ros_ns is None:
            valid_until_ros_ns = now_ns
        if previews and now_ns >= valid_until_ros_ns:
            previews = ()
            self.active_valid_until_ros_ns = None
            reason = "inspection status lease expired; Cartesian sketch cleared"
        self.marker_pub.publish(self._make_markers(previews))
        payload = {
            "preview_valid": bool(previews),
            "motion_capable": False,
            "collision_checked": False,
            "kinematics_checked": False,
            "requires_moveit_planning_scene": True,
            "verification_level": "unverified_cartesian_safety_sketch",
            "reason": reason,
            "config_sha256": self.settings.config_sha256,
            "status_published_ros_ns": now_ns,
            "valid_until_ros_ns": int(valid_until_ros_ns),
            "paths": [
                {
                    "source": preview.source_slot_id,
                    "destination": preview.destination_slot_id,
                    "corridor_radius_m": round(preview.corridor_radius_m, 6),
                    "points_base_m": [
                        [round(point.x, 6), round(point.y, 6), round(point.z, 6)]
                        for point in preview.staged_path
                    ],
                }
                for preview in previews
            ],
        }
        output = String()
        output.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(output)

    def _make_markers(self, previews) -> MarkerArray:
        result = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        result.markers.append(delete)
        if self.geometry is None:
            return result

        geometry = self.geometry
        stamp = self.get_clock().now().to_msg()
        battery = Marker()
        battery.header.frame_id = "base_link"
        battery.header.stamp = stamp
        battery.ns = "battery_unverified_envelope"
        battery.id = 0
        battery.type = Marker.CUBE
        battery.action = Marker.ADD
        centre_x_fixture = (geometry.battery_x_min_m + geometry.battery_x_max_m) / 2.0
        centre_y_fixture = (geometry.battery_y_min_m + geometry.battery_y_max_m) / 2.0
        centre_x, centre_y = geometry.fixture_xy_to_base(
            centre_x_fixture, centre_y_fixture
        )
        battery.pose.position.x = centre_x
        battery.pose.position.y = centre_y
        battery.pose.position.z = geometry.fixture_z_base_m + (
            geometry.battery_bottom_z_fixture_m
            + geometry.battery_top_z_fixture_m
        ) / 2.0
        battery.pose.orientation.z = math.sin(geometry.fixture_yaw_rad / 2.0)
        battery.pose.orientation.w = math.cos(geometry.fixture_yaw_rad / 2.0)
        battery.scale.x = geometry.battery_x_max_m - geometry.battery_x_min_m
        battery.scale.y = geometry.battery_y_max_m - geometry.battery_y_min_m
        battery.scale.z = (
            geometry.battery_top_z_fixture_m
            - geometry.battery_bottom_z_fixture_m
        )
        battery.color.r = 0.75
        battery.color.g = 0.15
        battery.color.b = 0.1
        battery.color.a = 0.35
        result.markers.append(battery)

        clearance = Marker()
        clearance.header = battery.header
        clearance.ns = "safe_transit_plane"
        clearance.id = 1
        clearance.type = Marker.LINE_STRIP
        clearance.action = Marker.ADD
        clearance.pose.orientation.w = 1.0
        clearance.scale.x = 0.003
        clearance.color.r = 1.0
        clearance.color.g = 0.75
        clearance.color.a = 1.0
        for x_fixture, y_fixture in (
            (geometry.battery_x_min_m, geometry.battery_y_min_m),
            (geometry.battery_x_max_m, geometry.battery_y_min_m),
            (geometry.battery_x_max_m, geometry.battery_y_max_m),
            (geometry.battery_x_min_m, geometry.battery_y_max_m),
            (geometry.battery_x_min_m, geometry.battery_y_min_m),
        ):
            x_base, y_base = geometry.fixture_xy_to_base(x_fixture, y_fixture)
            clearance.points.append(
                Point(x=x_base, y=y_base, z=geometry.safe_transit_z_base_m)
            )
        result.markers.append(clearance)

        next_id = 10
        for region in geometry.protected_regions:
            marker = Marker()
            marker.header = battery.header
            marker.ns = "protected_regions"
            marker.id = next_id
            next_id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            x_base, y_base = geometry.fixture_xy_to_base(region.x_m, region.y_m)
            marker.pose.position.x = x_base
            marker.pose.position.y = y_base
            bottom_z = geometry.fixture_z_base_m + (
                geometry.battery_bottom_z_fixture_m
            )
            top_z = geometry.fixture_z_base_m + region.top_z_fixture_m
            marker.pose.position.z = (bottom_z + top_z) / 2.0
            marker.pose.orientation.w = 1.0
            forbidden_radius = region.radius_m + geometry.corridor_radius_m
            marker.scale.x = 2.0 * forbidden_radius
            marker.scale.y = 2.0 * forbidden_radius
            marker.scale.z = top_z - bottom_z
            marker.color.r = 1.0
            marker.color.a = 0.55
            result.markers.append(marker)

        for preview in previews:
            marker = Marker()
            marker.header = battery.header
            marker.ns = "staged_pick_paths"
            marker.id = next_id
            next_id += 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(0.002, preview.corridor_radius_m * 0.12)
            marker.color.g = 1.0
            marker.color.b = 0.25
            marker.color.a = 1.0
            marker.points = [self._point_message(point) for point in preview.staged_path]
            result.markers.append(marker)
        return result

    @staticmethod
    def _point_message(point: Point3) -> Point:
        return Point(x=point.x, y=point.y, z=point.z)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CapPickPreviewNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
