import copy
import dataclasses
import unittest
from pathlib import Path

from arm_controller.projects.caps.workcell.cap_motion_safety import (
    Point3,
    assert_no_low_lateral_motion,
    build_pick_preview,
    load_motion_geometry,
    motion_geometry_blockers,
)
from arm_controller.projects.caps.workcell.cap_workcell_config import load_inspection_settings


_CONFIG_PATH = Path(__file__).parents[3] / "config" / "cap_workcell.example.yaml"


def _measured_settings():
    settings = load_inspection_settings(str(_CONFIG_PATH))
    raw = copy.deepcopy(dict(settings.raw))
    raw["geometry"] = {
        "fixture_plane_is_level": True,
        "marker_plane_at_cap_top": True,
        "protected_regions_measured": True,
        "destination_clearance_verified": True,
        "fixture_in_base": {"x_m": 0.05, "y_m": -0.08, "z_m": 0.10, "yaw_deg": 5.0},
        "battery": {
            "x_min_m": -0.010,
            "x_max_m": 0.215,
            "y_min_m": 0.000,
            "y_max_m": 0.090,
            "bottom_z_fixture_m": -0.080,
            "top_z_fixture_m": 0.0,
        },
        "safe_transit_z_base_m": 0.19,
        "grasp_tool_z_base_m": 0.105,
        "gripper_outer_width_m": 0.020,
        "cap_radius_m": 0.005,
        "pose_uncertainty_m": 0.003,
        "collision_padding_m": 0.002,
        "minimum_vertical_clearance_m": 0.040,
        "tool_payload_below_tcp_m": 0.010,
        "protected_regions": [
            {
                "id": "positive_terminal",
                "x_m": 0.205,
                "y_m": 0.075,
                "radius_m": 0.005,
                "top_z_fixture_m": 0.030,
            }
        ],
        "destination_slots": [
            {
                "id": f"place_{index}",
                "x_base_m": 0.04 + index * 0.030,
                "y_base_m": 0.16,
                "z_base_m": 0.11,
            }
            for index in range(7)
        ],
    }
    return dataclasses.replace(settings, raw=raw)


class CapMotionSafetyTests(unittest.TestCase):
    def test_example_has_no_plausible_motion_defaults(self):
        settings = load_inspection_settings(str(_CONFIG_PATH))
        blockers = motion_geometry_blockers(settings)
        self.assertTrue(blockers)
        self.assertIn("measured", blockers[0])

    def test_builds_vertical_pick_and_high_lateral_transit(self):
        settings = _measured_settings()
        geometry = load_motion_geometry(settings)
        preview = build_pick_preview(settings, geometry, "cap_2", "place_2")

        self.assertEqual(preview.pregrasp.x, preview.grasp.x)
        self.assertEqual(preview.grasp.x, preview.lift.x)
        self.assertEqual(preview.pregrasp.y, preview.grasp.y)
        self.assertEqual(preview.lift.z, geometry.safe_transit_z_base_m)
        self.assertEqual(preview.place_above.z, geometry.safe_transit_z_base_m)
        assert_no_low_lateral_motion(
            preview.staged_path, geometry.safe_transit_z_base_m
        )

    def test_rejects_lateral_motion_below_transit_plane(self):
        with self.assertRaisesRegex(ValueError, "lateral motion"):
            assert_no_low_lateral_motion(
                (Point3(0.1, 0.1, 0.12), Point3(0.2, 0.1, 0.12)),
                0.17,
            )

    def test_rejects_gripper_corridor_that_can_hit_battery_edge(self):
        settings = _measured_settings()
        raw = copy.deepcopy(dict(settings.raw))
        raw["geometry"]["gripper_outer_width_m"] = 0.080
        unsafe = dataclasses.replace(settings, raw=raw)
        with self.assertRaisesRegex(ValueError, "edge clearance"):
            load_motion_geometry(unsafe)

    def test_rejects_large_visual_deviation_from_known_slot(self):
        settings = _measured_settings()
        geometry = load_motion_geometry(settings)
        with self.assertRaisesRegex(ValueError, "from its slot"):
            build_pick_preview(
                settings,
                geometry,
                "cap_0",
                "place_0",
                observed_source_xy_m=(0.060, 0.055),
            )

    def test_localization_tolerance_is_included_in_edge_clearance(self):
        settings = _measured_settings()
        raw = copy.deepcopy(dict(settings.raw))
        raw["geometry"]["battery"]["x_min_m"] = -0.007
        settings = dataclasses.replace(settings, raw=raw)
        slots = list(settings.layout.slots)
        slots[0] = dataclasses.replace(slots[0], max_center_error_m=0.0075)
        layout = dataclasses.replace(settings.layout, slots=tuple(slots))
        unsafe = dataclasses.replace(settings, layout=layout)
        with self.assertRaisesRegex(ValueError, "edge clearance"):
            load_motion_geometry(unsafe)

    def test_rejects_destination_overlapping_battery(self):
        settings = _measured_settings()
        raw = copy.deepcopy(dict(settings.raw))
        source = settings.layout.slots[2]
        x_base, y_base = load_motion_geometry(settings).fixture_xy_to_base(
            source.x_m, source.y_m
        )
        raw["geometry"]["destination_slots"][0].update(
            {"x_base_m": x_base, "y_base_m": y_base}
        )
        unsafe = dataclasses.replace(settings, raw=raw)
        with self.assertRaisesRegex(ValueError, "battery footprint"):
            load_motion_geometry(unsafe)

    def test_requires_physical_geometry_attestations(self):
        settings = _measured_settings()
        raw = copy.deepcopy(dict(settings.raw))
        raw["geometry"]["marker_plane_at_cap_top"] = False
        unsafe = dataclasses.replace(settings, raw=raw)
        with self.assertRaisesRegex(ValueError, "physically verified"):
            load_motion_geometry(unsafe)

    def test_requires_explicit_right_handed_fixture_convention(self):
        settings = _measured_settings()
        raw = copy.deepcopy(dict(settings.raw))
        raw["fixture"]["coordinate_convention"] = "legacy_page_y_down"
        unsafe = dataclasses.replace(settings, raw=raw)
        with self.assertRaisesRegex(ValueError, "right_handed_z_up"):
            load_motion_geometry(unsafe)

    def test_protected_region_top_controls_safe_transit_height(self):
        settings = _measured_settings()
        raw = copy.deepcopy(dict(settings.raw))
        raw["geometry"]["protected_regions"][0]["top_z_fixture_m"] = 0.040
        unsafe = dataclasses.replace(settings, raw=raw)
        with self.assertRaisesRegex(ValueError, "protected-region top"):
            load_motion_geometry(unsafe)

    def test_destination_needs_vertical_approach_clearance(self):
        settings = _measured_settings()
        raw = copy.deepcopy(dict(settings.raw))
        raw["geometry"]["destination_slots"][0]["z_base_m"] = 0.160
        unsafe = dataclasses.replace(settings, raw=raw)
        with self.assertRaisesRegex(ValueError, "vertical approach clearance"):
            load_motion_geometry(unsafe)

    def test_cap_radius_cannot_understate_detected_slot_radius(self):
        settings = _measured_settings()
        raw = copy.deepcopy(dict(settings.raw))
        raw["geometry"]["cap_radius_m"] = 0.004
        unsafe = dataclasses.replace(settings, raw=raw)
        with self.assertRaisesRegex(ValueError, "smaller than a configured"):
            load_motion_geometry(unsafe)

    def test_protected_region_top_cannot_be_below_battery_bottom(self):
        settings = _measured_settings()
        raw = copy.deepcopy(dict(settings.raw))
        raw["geometry"]["protected_regions"][0]["top_z_fixture_m"] = -0.090
        unsafe = dataclasses.replace(settings, raw=raw)
        with self.assertRaisesRegex(ValueError, "below the battery bottom"):
            load_motion_geometry(unsafe)


if __name__ == "__main__":
    unittest.main()
