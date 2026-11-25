import copy
import dataclasses
import tempfile
import unittest
from pathlib import Path

import yaml

from arm_controller.cap_workcell_config import (
    hardware_execution_blockers,
    load_inspection_settings,
    load_source_destination_mapping,
)


_CONFIG_PATH = (
    Path(__file__).parents[3] / "config" / "cap_workcell.example.yaml"
)


class CapWorkcellConfigTests(unittest.TestCase):
    def test_example_loads_for_camera_only_inspection(self):
        settings = load_inspection_settings(str(_CONFIG_PATH))
        self.assertEqual(len(settings.layout.slots), 7)
        self.assertEqual(settings.fixture_frame_id, "battery_fixture")
        self.assertEqual(len(settings.config_sha256), 64)
        self.assertIsNone(settings.observation_joint_positions_deg)
        self.assertIsNone(settings.camera_pose_reference)
        self.assertFalse(settings.camera_pose_reference_verified)
        self.assertEqual(settings.localization_mode, "fixed_cap_row")
        self.assertIsNone(settings.fixed_jig_reference)
        self.assertFalse(settings.fixed_jig_reference_verified)
        self.assertIsNone(settings.visual_servo)
        self.assertFalse(settings.visual_servo_verified)
        self.assertFalse(settings.inspector.require_fixture_pose)
        self.assertFalse(settings.require_calibrated_camera_info)

    def test_example_is_hard_locked_against_hardware(self):
        settings = load_inspection_settings(str(_CONFIG_PATH))
        blockers = hardware_execution_blockers(settings)
        self.assertIn("configuration is marked example_only", blockers)
        self.assertIn("configuration is not commissioned", blockers)
        self.assertIn("hardware_execution_permitted is false", blockers)
        self.assertIn(
            "a one-dimensional cap row cannot validate the battery's full "
            "2D fixture pose",
            blockers,
        )
        self.assertIn(
            "camera-to-tool visual-servo mapping is not measured", blockers
        )
        self.assertIn(
            "calibrated CameraInfo is not required by configuration", blockers
        )

    def test_fixed_jig_reference_is_strict_and_mode_consistent(self):
        document = yaml.safe_load(_CONFIG_PATH.read_text(encoding="utf-8"))
        document["camera"]["localization_mode"] = "fixed_red_jig"
        reference = document["camera"]["fixed_jig_reference"]
        reference["plane_to_image_homography"] = [
            1000.0,
            0.0,
            120.0,
            0.0,
            -1000.0,
            400.0,
            0.0,
            0.0,
            1.0,
        ]
        reference["verified"] = True
        visual = document["camera"]["visual_servo"]
        visual["correction_matrix_m_per_px"] = [
            0.0002,
            0.0,
            0.0,
            -0.0002,
        ]
        visual["verified"] = True
        document["example_only"] = False
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "fixed.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            settings = load_inspection_settings(str(path))
        self.assertIsNotNone(settings.fixed_jig_reference)
        self.assertTrue(settings.fixed_jig_reference_verified)
        self.assertIsNotNone(settings.visual_servo)
        self.assertTrue(settings.visual_servo_verified)

        document["perception"]["require_fixture_pose"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "cannot claim live"):
                load_inspection_settings(str(path))

    def test_rejects_unknown_perception_key(self):
        text = _CONFIG_PATH.read_text(encoding="utf-8")
        text = text.replace(
            "  stability_frames: 8", "  stability_frames: 8\n  typo_limit: 1"
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text(text, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unknown perception"):
                load_inspection_settings(str(path))

    def test_rejects_quoted_boolean(self):
        document = yaml.safe_load(_CONFIG_PATH.read_text(encoding="utf-8"))
        document["commissioned"] = "false"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unquoted YAML boolean"):
                load_inspection_settings(str(path))

    def test_rejects_boolean_as_measurement(self):
        document = yaml.safe_load(_CONFIG_PATH.read_text(encoding="utf-8"))
        document["fixture"]["workspace_bounds_m"][0] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "not a boolean"):
                load_inspection_settings(str(path))

    def test_rejects_quoted_yaml_number_and_non_integer_count(self):
        for value in ("4000.0", False):
            with self.subTest(value=value):
                document = yaml.safe_load(
                    _CONFIG_PATH.read_text(encoding="utf-8")
                )
                document["perception"]["pixels_per_meter"] = value
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory) / "bad.yaml"
                    path.write_text(yaml.safe_dump(document), encoding="utf-8")
                    with self.assertRaises(ValueError):
                        load_inspection_settings(str(path))

        document = yaml.safe_load(_CONFIG_PATH.read_text(encoding="utf-8"))
        document["perception"]["stability_frames"] = 8.0
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "must be an integer"):
                load_inspection_settings(str(path))

    def test_rejects_perception_threshold_that_weakens_fail_closed_gate(self):
        document = yaml.safe_load(_CONFIG_PATH.read_text(encoding="utf-8"))
        document["perception"]["min_valid_roi_fraction"] = 0.5
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "min_valid_roi_fraction"):
                load_inspection_settings(str(path))

    def test_rejects_unsafe_observation_timing_and_handedness(self):
        for section, key, value, reason in (
            ("observation", "maximum_image_age_s", 10.0, "commissioning-safe"),
            (
                "observation",
                "minimum_stability_duration_s",
                0.1,
                "commissioning-safe",
            ),
            ("observation", "pose_tolerance_deg", 15.0, "commissioning-safe"),
            (
                "fixture",
                "coordinate_convention",
                "page_y_down",
                "coordinate_convention",
            ),
        ):
            with self.subTest(key=key):
                document = yaml.safe_load(
                    _CONFIG_PATH.read_text(encoding="utf-8")
                )
                document[section][key] = value
                with tempfile.TemporaryDirectory() as directory:
                    path = Path(directory) / "bad.yaml"
                    path.write_text(yaml.safe_dump(document), encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, reason):
                        load_inspection_settings(str(path))

    def test_camera_pose_reference_is_strict_and_complete(self):
        document = yaml.safe_load(_CONFIG_PATH.read_text(encoding="utf-8"))
        reference = document["camera"]["fixture_pose_reference"]
        reference["position_fixture_m"] = [0.12, 0.08, 0.30]
        reference["orientation_fixture_from_camera_xyzw"] = [
            1.0,
            0.0,
            0.0,
            0.0,
        ]
        reference["verified"] = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pose.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            settings = load_inspection_settings(str(path))
        self.assertIsNotNone(settings.camera_pose_reference)
        self.assertTrue(settings.camera_pose_reference_verified)

        reference["position_fixture_m"][1] = None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "partial.yaml"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "partially measured"):
                load_inspection_settings(str(path))

    def test_source_destination_mapping_is_explicit_complete_bijection(self):
        settings = load_inspection_settings(str(_CONFIG_PATH))
        raw = copy.deepcopy(dict(settings.raw))
        raw["geometry"]["source_destination_map"] = {
            f"cap_{index}": f"place_{6 - index}" for index in range(7)
        }
        mapped = dataclasses.replace(settings, raw=raw)
        pairs = load_source_destination_mapping(
            mapped, [f"place_{index}" for index in range(7)]
        )
        self.assertEqual(pairs[0], ("cap_0", "place_6"))
        self.assertEqual(pairs[-1], ("cap_6", "place_0"))

        raw["geometry"]["source_destination_map"]["cap_6"] = "place_1"
        with self.assertRaisesRegex(
            ValueError, "every destination exactly once"
        ):
            load_source_destination_mapping(
                dataclasses.replace(settings, raw=raw),
                [f"place_{index}" for index in range(7)],
            )


if __name__ == "__main__":
    unittest.main()
