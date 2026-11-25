"""Safety tests for exporting the official camera calibration archive."""

import io
import tarfile
import tempfile
import unittest
from pathlib import Path

from arm_controller.projects.camera_calibration.camera_calibration_export import export_calibration_archive


_VALID_YAML = b"""\
image_width: 640
image_height: 480
camera_name: rashko_gripper_camera
camera_matrix: {rows: 3, cols: 3, data: [500, 0, 320, 0, 501, 240, 0, 0, 1]}
distortion_model: plumb_bob
distortion_coefficients: {rows: 1, cols: 5, data: [0.1, -0.1, 0, 0, 0]}
rectification_matrix: {rows: 3, cols: 3, data: [1, 0, 0, 0, 1, 0, 0, 0, 1]}
projection_matrix: {rows: 3, cols: 4, data: [500, 0, 320, 0, 0, 501, 240, 0, 0, 0, 1, 0]}
"""


class CameraCalibrationExportTests(unittest.TestCase):
    """Reject missing, malformed, and accidental replacement exports."""

    @staticmethod
    def _archive(path: Path, name: str, contents: bytes) -> None:
        with tarfile.open(path, "w:gz") as bundle:
            member = tarfile.TarInfo(name)
            member.size = len(contents)
            bundle.addfile(member, io.BytesIO(contents))

    def test_exports_only_valid_standard_yaml(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / "calibration.tar.gz"
            output = root / "camera.yaml"
            self._archive(archive, "ost.yaml", _VALID_YAML)

            result = export_calibration_archive(str(archive), str(output))

            self.assertEqual(result["width"], 640)
            self.assertEqual(output.read_bytes(), _VALID_YAML)

    def test_refuses_overwrite_and_missing_or_invalid_yaml(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            archive = root / "calibration.tar.gz"
            output = root / "camera.yaml"
            output.write_text("known calibration", encoding="utf-8")
            self._archive(archive, "ost.yaml", _VALID_YAML)
            with self.assertRaisesRegex(ValueError, "already exists"):
                export_calibration_archive(str(archive), str(output))
            self.assertEqual(
                output.read_text(encoding="utf-8"), "known calibration"
            )

            output.unlink()
            self._archive(archive, "wrong.yaml", _VALID_YAML)
            with self.assertRaisesRegex(ValueError, "no ost.yaml"):
                export_calibration_archive(str(archive), str(output))

            self._archive(archive, "ost.yaml", b"not: a camera calibration\n")
            with self.assertRaises(ValueError):
                export_calibration_archive(str(archive), str(output))
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
