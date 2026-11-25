"""Synthetic metric tests for planar fixture camera-pose validation."""

import math
import unittest

import cv2
import numpy as np

from arm_controller.projects.caps.workcell.fixture_pose import (
    FixturePoseReference,
    compare_fixture_pose_to_reference,
    estimate_fixture_pose,
    quaternion_rotation_distance_deg,
    rotation_matrix_to_quaternion_xyzw,
)


class FixturePoseTests(unittest.TestCase):
    """Prove calibrated PnP recovers and gates a camera-above-plane pose."""

    def setUp(self):
        self.camera_matrix = np.asarray(
            ((720.0, 0.0, 320.0), (0.0, 715.0, 240.0), (0.0, 0.0, 1.0)),
            dtype=np.float64,
        )
        self.plane = np.asarray(
            (
                (0.000, 0.000),
                (0.025, 0.000),
                (0.215, 0.000),
                (0.240, 0.000),
                (0.240, 0.160),
                (0.215, 0.160),
                (0.025, 0.160),
                (0.000, 0.160),
            ),
            dtype=np.float64,
        )
        self.camera_position = np.asarray((0.118, 0.079, 0.330))
        # Camera X follows fixture X, camera Y is inverted, and camera Z looks
        # down toward the plane. This is a proper 180-degree X rotation.
        self.rotation_fixture_to_camera = np.diag((1.0, -1.0, -1.0))
        self.translation = (
            -self.rotation_fixture_to_camera @ self.camera_position
        )
        object_points = np.column_stack((self.plane, np.zeros(len(self.plane))))
        rotation_vector, _ = cv2.Rodrigues(self.rotation_fixture_to_camera)
        projected, _ = cv2.projectPoints(
            object_points,
            rotation_vector,
            self.translation,
            self.camera_matrix,
            np.zeros(5),
        )
        self.image = projected.reshape(-1, 2)

    def test_recovers_metric_camera_position_and_orientation(self):
        estimate = estimate_fixture_pose(
            self.plane,
            self.image,
            self.camera_matrix,
            max_reprojection_rms_px=0.1,
        )

        self.assertTrue(
            np.allclose(
                estimate.position_fixture_m,
                self.camera_position,
                atol=1e-6,
            )
        )
        expected_quaternion = rotation_matrix_to_quaternion_xyzw(
            self.rotation_fixture_to_camera.T
        )
        self.assertLess(
            quaternion_rotation_distance_deg(
                estimate.orientation_fixture_from_camera_xyzw,
                expected_quaternion,
            ),
            1e-4,
        )
        self.assertLess(estimate.optical_axis_tilt_deg, 1e-5)
        self.assertLess(estimate.reprojection_rms_px, 1e-5)

    def test_reference_rejects_millimetric_or_angular_displacement(self):
        estimate = estimate_fixture_pose(
            self.plane,
            self.image,
            self.camera_matrix,
            max_reprojection_rms_px=0.1,
        )
        reference = FixturePoseReference(
            position_fixture_m=tuple(self.camera_position),
            orientation_fixture_from_camera_xyzw=(1.0, 0.0, 0.0, 0.0),
        )
        accepted = compare_fixture_pose_to_reference(
            estimate,
            reference,
            max_translation_error_m=0.001,
            max_rotation_error_deg=0.1,
        )
        self.assertLess(accepted.reference_translation_error_m, 1e-6)
        self.assertLess(accepted.reference_rotation_error_deg, 1e-4)

        shifted = FixturePoseReference(
            position_fixture_m=(
                self.camera_position[0] + 0.004,
                self.camera_position[1],
                self.camera_position[2],
            ),
            orientation_fixture_from_camera_xyzw=(1.0, 0.0, 0.0, 0.0),
        )
        with self.assertRaisesRegex(ValueError, "translation differs"):
            compare_fixture_pose_to_reference(
                estimate,
                shifted,
                max_translation_error_m=0.003,
                max_rotation_error_deg=0.1,
            )

        angle = math.radians(2.0) / 2.0
        rotated = FixturePoseReference(
            position_fixture_m=tuple(self.camera_position),
            orientation_fixture_from_camera_xyzw=(
                math.cos(angle),
                0.0,
                0.0,
                math.sin(angle),
            ),
        )
        with self.assertRaisesRegex(ValueError, "rotation differs"):
            compare_fixture_pose_to_reference(
                estimate,
                rotated,
                max_translation_error_m=0.001,
                max_rotation_error_deg=1.0,
            )

    def test_rejects_non_unit_or_below_plane_reference(self):
        with self.assertRaisesRegex(ValueError, "unit length"):
            FixturePoseReference((0.0, 0.0, 0.2), (0.0, 0.0, 0.0, 2.0))
        with self.assertRaisesRegex(ValueError, "above"):
            FixturePoseReference((0.0, 0.0, -0.2), (0.0, 0.0, 0.0, 1.0))


if __name__ == "__main__":
    unittest.main()
