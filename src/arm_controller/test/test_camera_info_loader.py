import tempfile
import unittest
from pathlib import Path

from arm_controller.camera_info_loader import (
    load_camera_info_yaml,
    require_matching_resolution,
)


_VALID_YAML = """\
image_width: 1280
image_height: 720
camera_name: gripper_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [800.0, 0.0, 640.0, 0.0, 801.0, 360.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1, -0.2, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [800.0, 0.0, 640.0, 0.0, 0.0, 801.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]
"""


class CameraInfoLoaderTests(unittest.TestCase):
    def _load(self, contents):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.yaml"
            path.write_text(contents, encoding="utf-8")
            return load_camera_info_yaml(str(path))

    def test_loads_standard_ros_calibration(self):
        calibration = self._load(_VALID_YAML)
        self.assertEqual(calibration["width"], 1280)
        self.assertEqual(calibration["height"], 720)
        self.assertEqual(calibration["distortion_model"], "plumb_bob")
        self.assertEqual(len(calibration["k"]), 9)
        self.assertEqual(len(calibration["p"]), 12)

    def test_rejects_zero_focal_length(self):
        with self.assertRaisesRegex(ValueError, "focal lengths"):
            self._load(_VALID_YAML.replace("800.0, 0.0, 640.0", "0.0, 0.0, 640.0", 1))

    def test_requires_exact_runtime_resolution(self):
        calibration = self._load(_VALID_YAML)
        require_matching_resolution(calibration, 1280, 720)
        with self.assertRaisesRegex(ValueError, "runtime image"):
            require_matching_resolution(calibration, 640, 480)


if __name__ == "__main__":
    unittest.main()
