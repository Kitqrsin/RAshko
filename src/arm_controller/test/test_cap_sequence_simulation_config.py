"""Pure tests for the isolated synthetic cap-sequence scene schema."""

import copy
import math
import tempfile
import unittest
from pathlib import Path

import yaml

from arm_controller.projects.caps.cap_sequence_simulation.cap_sequence_simulation_config import (
    CapSequenceSimulationConfig,
    load_simulation_config,
)


_CONFIG_PATH = (
    Path(__file__).parents[3] / "config" / "cap_sequence.simulation.yaml"
)
_WORKCELL_PATH = (
    Path(__file__).parents[3] / "config" / "cap_workcell.example.yaml"
)


class CapSequenceSimulationConfigTests(unittest.TestCase):
    def setUp(self):
        self.raw = yaml.safe_load(_CONFIG_PATH.read_text(encoding="utf-8"))

    def _load(self, raw):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.yaml"
            path.write_text(
                yaml.safe_dump(raw, sort_keys=False), encoding="utf-8"
            )
            return load_simulation_config(str(path))

    def _load_text(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.yaml"
            path.write_text(text, encoding="utf-8")
            return load_simulation_config(str(path))

    def test_checked_in_scene_loads_as_typed_simulation_only_config(self):
        config = load_simulation_config(str(_CONFIG_PATH))

        self.assertIsInstance(config, CapSequenceSimulationConfig)
        self.assertIs(config.simulation_only, True)
        self.assertIs(config.synthetic_scene, True)
        self.assertIs(config.hardware_execution_permitted, False)
        self.assertEqual(config.frame_id, "base_link")
        self.assertEqual(len(config.protected_objects), 7)
        self.assertGreater(config.table.size.x, 0.0)
        self.assertGreater(config.battery.size.z, 0.0)
        self.assertGreater(config.tray.size.y, 0.0)
        self.assertGreater(config.cap.radius_m, 0.0)
        self.assertGreater(config.cap.height_m, 0.0)
        self.assertTrue(config.visual_servo.enabled)
        self.assertEqual(len(config.visual_servo.coarse_offsets_xy_m), 7)
        self.assertLessEqual(config.visual_servo.controller.max_step_m, 0.003)
        self.assertEqual(len(config.config_sha256), 64)

    def test_checked_in_scene_matches_known_physical_measurements(self):
        config = load_simulation_config(str(_CONFIG_PATH))

        self.assertTrue(math.isclose(config.battery.size.y, 0.205))
        self.assertTrue(math.isclose(config.battery.size.z, 0.140))
        self.assertTrue(math.isclose(config.cap.radius_m * 2.0, 0.0083))
        self.assertTrue(math.isclose(config.cap.height_m, 0.020))
        self.assertTrue(math.isclose(config.battery.bottom_z, -0.015935))
        self.assertTrue(math.isclose(config.battery.top_z, 0.124065))
        self.assertTrue(
            math.isclose(
                config.battery.x_max - (-0.043047),
                -0.120,
                abs_tol=1e-9,
            )
        )
        neighbor_distances = [
            math.hypot(
                second.cap_center.x - first.cap_center.x,
                second.cap_center.y - first.cap_center.y,
            )
            for first, second in zip(config.sources, config.sources[1:])
        ]
        self.assertTrue(
            all(
                math.isclose(distance, 0.030)
                for distance in neighbor_distances
            )
        )

    def test_visual_servo_must_cover_all_sources_and_remain_bounded(self):
        raw = copy.deepcopy(self.raw)
        del raw["visual_servo"]["coarse_offsets_xy_m"]["cap_6"]
        with self.assertRaisesRegex(ValueError, "every source"):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        raw["visual_servo"]["max_step_m"] = 0.02
        with self.assertRaisesRegex(ValueError, "max step"):
            self._load(raw)

    def test_checked_in_scene_has_exact_complete_bijection(self):
        config = load_simulation_config(str(_CONFIG_PATH))

        source_ids = tuple(slot.slot_id for slot in config.sources)
        destination_ids = tuple(slot.slot_id for slot in config.destinations)
        self.assertEqual(len(source_ids), 7)
        self.assertEqual(len(set(source_ids)), 7)
        self.assertEqual(len(destination_ids), 7)
        self.assertEqual(len(set(destination_ids)), 7)
        self.assertEqual(
            config.source_destination_pairs,
            tuple((f"cap_{index}", f"place_{index}") for index in range(7)),
        )

    def test_measured_workcell_schema_cannot_be_loaded(self):
        with self.assertRaisesRegex(ValueError, "simulation schema"):
            load_simulation_config(str(_WORKCELL_PATH))

    def test_requires_exact_simulation_and_hardware_flags(self):
        mutations = (
            ("simulation_only", False, "simulation_only must remain true"),
            ("synthetic_scene", False, "synthetic_scene must remain true"),
        )
        for field, value, message in mutations:
            with self.subTest(field=field):
                raw = copy.deepcopy(self.raw)
                raw[field] = value
                with self.assertRaisesRegex(ValueError, message):
                    self._load(raw)

        raw = copy.deepcopy(self.raw)
        raw["execution"]["hardware_execution_permitted"] = True
        with self.assertRaisesRegex(ValueError, "must remain false"):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        raw["execution"]["backend"] = "hardware"
        with self.assertRaisesRegex(ValueError, "must be 'simulation'"):
            self._load(raw)

    def test_requires_exactly_seven_unique_source_and_destination_ids(self):
        raw = copy.deepcopy(self.raw)
        raw["sources"].pop()
        with self.assertRaisesRegex(ValueError, "exactly seven"):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        raw["destinations"][6]["id"] = "place_5"
        with self.assertRaisesRegex(
            ValueError, "destination IDs must be unique"
        ):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        raw["sources"][6]["id"] = "cap_5"
        with self.assertRaisesRegex(ValueError, "source IDs must be unique"):
            self._load(raw)

    def test_mapping_must_be_complete_and_bijective(self):
        raw = copy.deepcopy(self.raw)
        del raw["source_destination_map"]["cap_6"]
        with self.assertRaisesRegex(ValueError, "every source exactly once"):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        raw["source_destination_map"]["cap_6"] = "place_5"
        with self.assertRaisesRegex(
            ValueError, "every destination exactly once"
        ):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        raw["source_destination_map"]["cap_6"] = "unknown_place"
        with self.assertRaisesRegex(
            ValueError, "every destination exactly once"
        ):
            self._load(raw)

    def test_rejects_nonfinite_quoted_boolean_and_nonpositive_geometry(self):
        mutations = (
            (("scene", "table", "size_m", 0), 0.0, "positive"),
            (("scene", "battery", "size_m", 1), -0.1, "positive"),
            (("scene", "tray", "size_m", 2), math.inf, "finite"),
            (("scene", "cap", "radius_m"), "0.005", "unquoted finite"),
            (("scene", "cap", "height_m"), True, "unquoted finite"),
            (
                ("scene", "protected_objects", 0, "height_m"),
                0.0,
                "positive",
            ),
        )
        for keys, value, message in mutations:
            with self.subTest(keys=keys):
                raw = copy.deepcopy(self.raw)
                target = raw
                for key in keys[:-1]:
                    target = target[key]
                target[keys[-1]] = value
                with self.assertRaisesRegex(ValueError, message):
                    self._load(raw)

    def test_table_support_and_protected_object_geometry_is_consistent(self):
        raw = copy.deepcopy(self.raw)
        raw["scene"]["battery"]["center_m"][2] += 0.001
        with self.assertRaisesRegex(ValueError, "rest on the table"):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        raw["scene"]["protected_objects"][0]["bottom_z_m"] += 0.001
        with self.assertRaisesRegex(ValueError, "start at the battery top"):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        raw["scene"]["protected_objects"][0]["center_xy_m"][0] = 1.0
        with self.assertRaisesRegex(ValueError, "does not fit on the battery"):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        first = raw["scene"]["protected_objects"][0]
        raw["scene"]["protected_objects"][1]["center_xy_m"] = first[
            "center_xy_m"
        ]
        with self.assertRaisesRegex(
            ValueError, "protected objects .* overlap"
        ):
            self._load(raw)

    def test_sources_and_destinations_must_fit_without_overlap(self):
        raw = copy.deepcopy(self.raw)
        raw["sources"][0]["x_m"] = raw["sources"][1]["x_m"]
        raw["sources"][0]["y_m"] = raw["sources"][1]["y_m"]
        with self.assertRaisesRegex(ValueError, "source .* overlap"):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        raw["destinations"][0]["x_m"] = 1.0
        with self.assertRaisesRegex(ValueError, "does not fit inside"):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        terminal = raw["scene"]["protected_objects"][0]
        raw["sources"][0]["x_m"] = terminal["center_xy_m"][0]
        raw["sources"][0]["y_m"] = terminal["center_xy_m"][1]
        with self.assertRaisesRegex(ValueError, "overlaps protected object"):
            self._load(raw)

    def test_grasp_and_place_tool_origins_must_be_above_cap_tops(self):
        raw = copy.deepcopy(self.raw)
        battery = raw["scene"]["battery"]
        battery_top = (
            battery["center_m"][2] + battery["size_m"][2] / 2.0
        )
        raw["motion"]["grasp_tool_z_m"] = (
            battery_top + raw["scene"]["cap"]["height_m"]
        )
        with self.assertRaisesRegex(ValueError, "source cap top"):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        tray = raw["scene"]["tray"]
        tray_top = tray["center_m"][2] + tray["size_m"][2] / 2.0
        raw["motion"]["place_tool_z_m"] = (
            tray_top + raw["scene"]["cap"]["height_m"]
        )
        with self.assertRaisesRegex(ValueError, "destination cap top"):
            self._load(raw)

    def test_safe_transit_clears_scene_and_below_tcp_envelope(self):
        raw = copy.deepcopy(self.raw)
        raw["motion"]["tool_payload_below_tcp_m"] = 0.063
        with self.assertRaisesRegex(ValueError, "safe transit Z"):
            self._load(raw)

        raw = copy.deepcopy(self.raw)
        raw["scene"]["protected_objects"][0]["height_m"] = 0.100
        with self.assertRaisesRegex(ValueError, "safe transit Z"):
            self._load(raw)

    def test_below_tcp_envelope_covers_the_full_attached_cap(self):
        raw = copy.deepcopy(self.raw)
        raw["motion"]["tool_payload_below_tcp_m"] = 0.019
        with self.assertRaisesRegex(ValueError, "full cap height"):
            self._load(raw)

    def test_unknown_fields_fail_closed(self):
        raw = copy.deepcopy(self.raw)
        raw["commissioned"] = True
        with self.assertRaisesRegex(ValueError, "unknown=.*commissioned"):
            self._load(raw)

    def test_duplicate_yaml_mapping_keys_fail_closed(self):
        text = _CONFIG_PATH.read_text(encoding="utf-8")
        text = text.replace(
            "  hardware_execution_permitted: false",
            "  hardware_execution_permitted: false\n"
            "  hardware_execution_permitted: false",
        )
        with self.assertRaisesRegex(ValueError, "duplicate YAML mapping key"):
            self._load_text(text)

    def test_slot_ids_cannot_shadow_fixed_collision_objects(self):
        raw = copy.deepcopy(self.raw)
        raw["sources"][0]["id"] = raw["scene"]["table"]["id"]
        with self.assertRaisesRegex(ValueError, "fixed collision object IDs"):
            self._load(raw)


if __name__ == "__main__":
    unittest.main()
