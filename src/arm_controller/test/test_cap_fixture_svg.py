import copy
import dataclasses
import unittest
from pathlib import Path

import cv2

from arm_controller.projects.caps.workcell.cap_fixture_svg import generate_fixture_svg
from arm_controller.projects.caps.workcell.cap_workcell_config import load_inspection_settings


_CONFIG_PATH = Path(__file__).parents[3] / "config" / "cap_workcell.example.yaml"


@unittest.skipUnless(hasattr(cv2, "aruco"), "OpenCV build has no ArUco module")
class CapFixtureSvgTests(unittest.TestCase):
    def test_generates_exact_size_four_marker_svg(self):
        settings = load_inspection_settings(str(_CONFIG_PATH))
        svg = generate_fixture_svg(settings)
        self.assertIn('width="264.000mm"', svg)
        self.assertEqual(svg.count("<image "), 4)
        for marker_id in (10, 11, 12, 13):
            self.assertIn(f"ArUco {marker_id}", svg)
        self.assertIn("print at 100%", svg)
        self.assertIn('data-fixture-convention="right_handed_z_up"', svg)
        self.assertIn("+Y page up", svg)
        # IDs 10/11 remain at the physical page top although fixture Y grows up.
        self.assertEqual(svg.count(' y="12.000" width="25.000"'), 2)
        self.assertEqual(svg.count(' y="147.000" width="25.000"'), 2)

    def test_rejects_legacy_page_y_down_marker_order(self):
        settings = load_inspection_settings(str(_CONFIG_PATH))
        raw = copy.deepcopy(dict(settings.raw))
        raw["fixture"]["marker_corners_m"][10] = [
            [0.000, 0.000],
            [0.025, 0.000],
            [0.025, 0.025],
            [0.000, 0.025],
        ]
        legacy = dataclasses.replace(settings, raw=raw)
        marker_corners = dict(settings.layout.marker_corners_m)
        marker_corners[10] = tuple(
            tuple(point) for point in raw["fixture"]["marker_corners_m"][10]
        )
        legacy = dataclasses.replace(
            legacy,
            layout=dataclasses.replace(settings.layout, marker_corners_m=marker_corners),
        )
        with self.assertRaisesRegex(ValueError, "TL, TR, BR, BL"):
            generate_fixture_svg(legacy)


if __name__ == "__main__":
    unittest.main()
