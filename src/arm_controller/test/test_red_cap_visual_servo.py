"""Tests for fixed-jig red detection and bounded eye-in-hand alignment."""

import unittest

import cv2
import numpy as np

from arm_controller.projects.caps.workcell.red_cap_visual_servo import (
    RedBlobDetectorConfig,
    VisualServoConfig,
    VisualServoController,
    VisualServoError,
    derive_fixed_cap_row_candidate,
    derive_fixed_jig_reference,
    detect_red_blobs,
    require_vertical_motion,
    select_alignment_blob,
)


class RedCapVisualServoTests(unittest.TestCase):
    """Require unique red geometry, bounded XY steps, and vertical descent."""

    def setUp(self):
        self.detector = RedBlobDetectorConfig(
            min_area_px2=100.0,
            max_area_px2=2000.0,
        )

    @staticmethod
    def _image(points):
        image = np.full((480, 640, 3), 45, dtype=np.uint8)
        for point in points:
            cv2.circle(image, point, 14, (0, 0, 255), -1, cv2.LINE_AA)
        return image

    def test_detects_only_circular_solid_red_components(self):
        image = self._image(((100, 100), (300, 220)))
        cv2.rectangle(image, (450, 80), (520, 86), (0, 0, 255), -1)

        blobs = detect_red_blobs(image, self.detector)

        self.assertEqual(len(blobs), 2)
        self.assertTrue(all(item.circularity >= 0.55 for item in blobs))

    def test_default_detector_accepts_current_close_camera_cap_scale(self):
        image = np.full((480, 640, 3), 45, dtype=np.uint8)
        cv2.circle(image, (320, 240), 44, (0, 0, 255), -1, cv2.LINE_AA)

        blobs = detect_red_blobs(image)

        self.assertEqual(len(blobs), 1)
        self.assertGreater(blobs[0].area_px2, 5000.0)

    def test_selects_one_target_and_rejects_neighbor_ambiguity(self):
        blobs = detect_red_blobs(
            self._image(((325, 240), (500, 240))), self.detector
        )
        chosen = select_alignment_blob(
            blobs,
            (320.0, 240.0),
            acquisition_radius_px=80.0,
            ambiguity_margin_px=25.0,
        )
        self.assertLess(abs(chosen.center_px[0] - 325.0), 1.0)

        ambiguous = detect_red_blobs(
            self._image(((300, 240), (340, 240))), self.detector
        )
        with self.assertRaisesRegex(VisualServoError, "ambiguous"):
            select_alignment_blob(
                ambiguous,
                (320.0, 240.0),
                acquisition_radius_px=80.0,
                ambiguity_margin_px=25.0,
            )

    def test_visual_servo_bounds_steps_and_requires_stable_centering(self):
        config = VisualServoConfig(
            target_pixel=(320.0, 240.0),
            correction_matrix_m_per_px=(0.0002, 0.0, 0.0, -0.0002),
            max_step_m=0.003,
            centered_tolerance_px=2.0,
            required_stable_frames=3,
            max_iterations=8,
        )
        controller = VisualServoController(config)
        far_blob = detect_red_blobs(
            self._image(((400, 200),)), self.detector
        )[0]
        command = controller.observe(far_blob)
        self.assertLessEqual(np.linalg.norm(command.correction_xy_m), 0.003001)
        self.assertEqual(command.correction_xy_m[0] > 0.0, True)
        self.assertEqual(command.correction_xy_m[1] > 0.0, True)
        self.assertFalse(command.alignment_verified)

        centered_blob = detect_red_blobs(
            self._image(((320, 240),)), self.detector
        )[0]
        self.assertFalse(controller.observe(centered_blob).alignment_verified)
        self.assertFalse(controller.observe(centered_blob).alignment_verified)
        self.assertTrue(controller.observe(centered_blob).alignment_verified)

    def test_derives_unique_seven_cap_fixed_jig_homography(self):
        slots = {
            "cap_0": (0.010, 0.020),
            "cap_1": (0.050, 0.020),
            "cap_2": (0.095, 0.025),
            "cap_3": (0.140, 0.035),
            "cap_4": (0.025, 0.085),
            "cap_5": (0.090, 0.095),
            "cap_6": (0.155, 0.110),
        }
        plane = np.asarray([slots[key] for key in sorted(slots)], dtype=np.float32)
        transform = np.asarray(
            ((2200.0, 80.0, 90.0), (-50.0, 2100.0, 70.0), (0.7, 0.4, 1.0)),
            dtype=np.float64,
        )
        pixels = cv2.perspectiveTransform(
            plane.reshape(-1, 1, 2), transform
        ).reshape(-1, 2)
        image = self._image(
            tuple((int(round(x)), int(round(y))) for x, y in pixels)
        )
        blobs = tuple(reversed(detect_red_blobs(image, self.detector)))

        candidate = derive_fixed_jig_reference(
            slots,
            blobs,
            (640, 480),
            max_reprojection_rms_px=2.0,
            minimum_second_best_gap_px=0.25,
        )

        self.assertLess(candidate.reprojection_rms_px, 2.0)
        self.assertGreater(
            candidate.second_best_rms_px - candidate.reprojection_rms_px,
            0.25,
        )
        self.assertEqual(
            tuple(item[0] for item in candidate.slot_pixels), tuple(sorted(slots))
        )

    def test_low_motion_must_be_vertical(self):
        require_vertical_motion((0.1, 0.2, 0.05), (0.1, 0.2, 0.01))
        with self.assertRaisesRegex(VisualServoError, "lateral"):
            require_vertical_motion(
                (0.1, 0.2, 0.05),
                (0.102, 0.2, 0.01),
            )

    def test_orders_collinear_cap_row_without_claiming_plane_pose(self):
        slots = {
            f"cap_{index}": (0.0125 + index * 0.030, 0.050)
            for index in range(7)
        }
        image = self._image(
            tuple((110 + index * 65, 242 + (index % 2)) for index in range(7))
        )
        candidate = derive_fixed_cap_row_candidate(
            slots,
            tuple(reversed(detect_red_blobs(image, self.detector))),
            (640, 480),
        )
        self.assertEqual(
            tuple(item[0] for item in candidate.slot_pixels),
            tuple(f"cap_{index}" for index in range(7)),
        )
        self.assertLess(candidate.row_straightness_rms_px, 2.0)
        self.assertGreater(candidate.minimum_neighbor_spacing_px, 60.0)

    def test_keyed_jig_axis_convention_removes_mirror_ambiguity(self):
        slots = {
            "cap_0": (0.040, 0.105),
            "cap_1": (0.085, 0.105),
            "cap_2": (0.130, 0.105),
            "cap_3": (0.175, 0.105),
            "cap_4": (0.060, 0.050),
            "cap_5": (0.115, 0.050),
            "cap_6": (0.170, 0.050),
        }
        transform = np.asarray(
            (
                (1800.0, 100.0, 100.0),
                (-50.0, -1700.0, 400.0),
                (0.2, 0.1, 1.0),
            )
        )
        image = np.full((480, 640, 3), 40, dtype=np.uint8)
        for point in slots.values():
            pixel = cv2.perspectiveTransform(
                np.asarray(point, dtype=np.float64).reshape(1, 1, 2),
                transform,
            ).reshape(2)
            cv2.circle(
                image,
                tuple(np.rint(pixel).astype(int)),
                10,
                (0, 0, 255),
                -1,
            )
        candidate = derive_fixed_jig_reference(
            slots,
            detect_red_blobs(image),
            (640, 480),
            require_fixture_x_right_y_up=True,
        )
        self.assertLess(candidate.reprojection_rms_px, 0.5)
        self.assertGreater(candidate.second_best_rms_px, 10.0)

    def test_all_seven_synthetic_coarse_offsets_converge(self):
        sources = (
            (0.105, 0.020),
            (0.140, 0.020),
            (0.175, 0.020),
            (0.210, 0.020),
            (0.1225, 0.055),
            (0.1575, 0.055),
            (0.1925, 0.055),
        )
        offsets = (
            (0.0040, -0.0020),
            (-0.0030, 0.0025),
            (0.0020, 0.0035),
            (-0.0040, -0.0020),
            (0.0030, -0.0030),
            (-0.0025, 0.0030),
            (0.0035, 0.0020),
        )
        config = VisualServoConfig(
            target_pixel=(320.0, 240.0),
            correction_matrix_m_per_px=(0.0002, 0.0, 0.0, -0.0002),
            max_step_m=0.003,
            centered_tolerance_px=2.0,
            acquisition_radius_px=80.0,
            ambiguity_margin_px=25.0,
            required_stable_frames=3,
            max_iterations=10,
        )
        for source, offset in zip(sources, offsets):
            with self.subTest(source=source):
                tool = np.asarray(source) + np.asarray(offset)
                controller = VisualServoController(config)
                verified = False
                for _ in range(config.max_iterations):
                    pixel = (
                        int(round(320.0 + (source[0] - tool[0]) / 0.0002)),
                        int(round(240.0 - (source[1] - tool[1]) / 0.0002)),
                    )
                    blob = select_alignment_blob(
                        detect_red_blobs(self._image((pixel,)), self.detector),
                        config.target_pixel,
                        acquisition_radius_px=config.acquisition_radius_px,
                        ambiguity_margin_px=config.ambiguity_margin_px,
                    )
                    command = controller.observe(blob)
                    tool += np.asarray(command.correction_xy_m)
                    if command.alignment_verified:
                        verified = True
                        break
                self.assertTrue(verified)
                self.assertLess(np.linalg.norm(tool - source), 0.0005)


if __name__ == "__main__":
    unittest.main()
