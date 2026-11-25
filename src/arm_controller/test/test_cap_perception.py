"""Focused synthetic tests for fail-closed seven-cap perception."""

import dataclasses
import math
import unittest

import cv2
import numpy as np

from arm_controller.cap_perception import (
    CapPerception,
    CapSlot,
    FixtureLayout,
    InspectorConfig,
    ObservationStabilizer,
    PerceptionError,
    SlotOccupancy,
    estimate_planar_homography,
    image_content_digest,
    marker_correspondences_from_detections,
)


class CapPerceptionTests(unittest.TestCase):
    """Exercise geometry, classification, identity, and temporal safety gates."""

    def setUp(self):
        self.bounds = (0.0, 0.0, 0.200, 0.120)
        self.slots = tuple(
            CapSlot(name, x, y, 0.012, 0.004, 0.006)
            for name, x, y in (
                ("cap_0", 0.030, 0.035),
                ("cap_1", 0.075, 0.035),
                ("cap_2", 0.120, 0.035),
                ("cap_3", 0.165, 0.035),
                ("cap_4", 0.052, 0.083),
                ("cap_5", 0.100, 0.083),
                ("cap_6", 0.148, 0.083),
            )
        )
        marker_size = 0.012
        self.marker_layout = {
            10: ((0.0, 0.0), (marker_size, 0.0), (marker_size, marker_size), (0.0, marker_size)),
            11: ((0.188, 0.0), (0.200, 0.0), (0.200, 0.012), (0.188, 0.012)),
            12: ((0.188, 0.108), (0.200, 0.108), (0.200, 0.120), (0.188, 0.120)),
            13: ((0.0, 0.108), (0.012, 0.108), (0.012, 0.120), (0.0, 0.120)),
        }
        self.layout = FixtureLayout(self.bounds, self.slots, self.marker_layout)
        self.config = InspectorConfig(stability_frames=3)
        self.inspector = CapPerception(self.layout, self.config)
        self.plane_corners = np.asarray(
            ((0.0, 0.0), (0.200, 0.0), (0.200, 0.120), (0.0, 0.120)),
            dtype=np.float64,
        )
        self.image_corners = np.asarray(
            ((105.0, 82.0), (696.0, 68.0), (724.0, 501.0), (88.0, 514.0)),
            dtype=np.float64,
        )
        self.image_size = (820, 600)

    def _source_image(self, present_ids=None, malformed_id=None):
        if present_ids is None:
            present_ids = {slot.slot_id for slot in self.slots}
        scale = self.config.pixels_per_meter
        width = int(math.ceil((self.bounds[2] - self.bounds[0]) * scale)) + 1
        height = int(math.ceil((self.bounds[3] - self.bounds[1]) * scale)) + 1
        rectified = np.full((height, width, 3), 75, dtype=np.uint8)
        for slot in self.slots:
            if slot.slot_id not in present_ids:
                continue
            center = (int(round(slot.x_m * scale)), int(round(slot.y_m * scale)))
            radius = int(round(slot.expected_radius_m * scale))
            if slot.slot_id == malformed_id:
                cv2.rectangle(
                    rectified,
                    (center[0] - radius * 2, center[1] - radius),
                    (center[0] + radius * 2, center[1] + radius),
                    (0, 0, 255),
                    -1,
                )
            else:
                cv2.circle(rectified, center, radius, (0, 0, 255), -1, cv2.LINE_AA)

        plane_to_image = cv2.getPerspectiveTransform(
            self.plane_corners.astype(np.float32),
            self.image_corners.astype(np.float32),
        )
        rectified_px_to_plane = np.asarray(
            ((1.0 / scale, 0.0, 0.0), (0.0, 1.0 / scale, 0.0), (0.0, 0.0, 1.0)),
            dtype=np.float64,
        )
        rectified_to_image = plane_to_image @ rectified_px_to_plane
        return cv2.warpPerspective(
            rectified,
            rectified_to_image,
            self.image_size,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(30, 30, 30),
        )

    def _inspect(self, sequence, present_ids=None, malformed_id=None, image_corners=None):
        if present_ids is None:
            present_ids = {slot.slot_id for slot in self.slots}
        return self.inspector.inspect_bgr(
            self._source_image(present_ids, malformed_id),
            frame_sequence=sequence,
            expected_present_ids=present_ids,
            image_points_px=self.image_corners if image_corners is None else image_corners,
            plane_points_m=self.plane_corners,
            include_debug_images=True,
        )

    def _aruco_source_image(self):
        image = np.full((600, 820, 3), 240, dtype=np.uint8)
        scale = 3000.0
        offset_x = 100
        offset_y = 100
        dictionary = cv2.aruco.getPredefinedDictionary(
            self.config.aruco_dictionary_id
        )
        for marker_id, corners in self.marker_layout.items():
            if hasattr(cv2.aruco, "generateImageMarker"):
                marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 36)
            else:  # OpenCV 4.6 compatibility
                marker = cv2.aruco.drawMarker(dictionary, marker_id, 36)
            marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
            x = int(round(offset_x + corners[0][0] * scale))
            y = int(round(offset_y + corners[0][1] * scale))
            image[y:y + 36, x:x + 36] = marker_bgr
        for slot in self.slots:
            center = (
                int(round(offset_x + slot.x_m * scale)),
                int(round(offset_y + slot.y_m * scale)),
            )
            cv2.circle(
                image,
                center,
                int(round(slot.expected_radius_m * scale)),
                (0, 0, 255),
                -1,
                cv2.LINE_AA,
            )
        return image

    def test_detects_all_seven_caps_with_stable_slot_ids(self):
        observation = self._inspect(1)

        self.assertTrue(observation.fixture_valid, observation.reason)
        self.assertTrue(observation.classification_valid, observation.reason)
        self.assertTrue(observation.expectation_matches, observation.reason)
        self.assertEqual(
            [item.slot_id for item in observation.slots],
            [slot.slot_id for slot in self.slots],
        )
        self.assertTrue(
            all(item.occupancy is SlotOccupancy.PRESENT for item in observation.slots)
        )
        self.assertTrue(
            all(
                item.confidence >= self.config.min_present_confidence
                for item in observation.slots
            )
        )
        self.assertIsNotNone(observation.rectified_bgr)
        self.assertIsNotNone(observation.annotated_rectified_bgr)
        self.assertLess(observation.homography.reprojection_rms_px, 0.01)

    def test_image_digest_detects_exact_repeated_decoded_frame(self):
        first = self._source_image()
        repeated = first.copy()
        changed = first.copy()
        changed[0, 0, 0] ^= 1

        self.assertEqual(image_content_digest(first), image_content_digest(repeated))
        self.assertNotEqual(image_content_digest(first), image_content_digest(changed))

    def test_invalid_undistortion_pixels_cannot_prove_a_slot_absent(self):
        image = self._source_image()
        valid_mask = np.full(image.shape[:2], 255, dtype=np.uint8)
        plane_to_image = cv2.getPerspectiveTransform(
            self.plane_corners.astype(np.float32),
            self.image_corners.astype(np.float32),
        )
        projected = cv2.perspectiveTransform(
            np.asarray([[self.slots[0].center_m]], dtype=np.float32),
            plane_to_image,
        )[0, 0]
        cv2.circle(
            valid_mask,
            (int(round(projected[0])), int(round(projected[1]))),
            50,
            0,
            -1,
        )

        observation = self.inspector.inspect_bgr(
            image,
            frame_sequence=2,
            expected_present_ids={slot.slot_id for slot in self.slots},
            image_points_px=self.image_corners,
            plane_points_m=self.plane_corners,
            source_valid_mask=valid_mask,
        )

        self.assertFalse(observation.fixture_valid)
        self.assertIn("not completely visible", observation.reason)

    def test_inspector_config_rejects_weak_or_mistyped_safety_values(self):
        for values in (
            {"stability_frames": 3.0},
            {"stability_frames": True},
            {"min_valid_roi_fraction": False},
            {"min_valid_roi_fraction": 0.50},
            {"max_reprojection_error_px": 12.0},
        ):
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    InspectorConfig(**values)

    def test_rectified_raster_must_resolve_caps_and_keep_rois_separate(self):
        low_resolution = InspectorConfig(
            pixels_per_meter=900.0,
            morphology_radius_m=0.002,
            stability_frames=3,
        )
        with self.assertRaisesRegex(ValueError, "fewer than four pixels"):
            CapPerception(self.layout, low_resolution)

        crowded = list(self.slots)
        crowded[1] = dataclasses.replace(crowded[1], x_m=0.0544)
        crowded_layout = FixtureLayout(
            self.bounds,
            tuple(crowded),
            self.marker_layout,
        )
        with self.assertRaisesRegex(ValueError, "not separated by two pixels"):
            CapPerception(crowded_layout, self.config)

    @unittest.skipUnless(hasattr(cv2, "aruco"), "OpenCV build has no ArUco module")
    def test_detects_aruco_fixture_and_recomputes_frame_homography(self):
        all_ids = {slot.slot_id for slot in self.slots}
        observation = self.inspector.inspect_bgr(
            self._aruco_source_image(),
            frame_sequence=2,
            expected_present_ids=all_ids,
        )

        self.assertTrue(observation.ready_for_stability, observation.reason)
        self.assertEqual(observation.homography.visible_marker_ids, (10, 11, 12, 13))
        self.assertLess(
            observation.homography.reprojection_rms_px,
            self.config.max_reprojection_rms_px,
        )

    def test_removed_cap_is_confidently_absent_when_expected(self):
        remaining = {slot.slot_id for slot in self.slots} - {"cap_3"}
        observation = self._inspect(2, remaining)
        by_id = {item.slot_id: item for item in observation.slots}

        self.assertTrue(observation.ready_for_stability, observation.reason)
        self.assertIs(by_id["cap_3"].occupancy, SlotOccupancy.ABSENT)
        self.assertGreaterEqual(by_id["cap_3"].confidence, self.config.min_absent_confidence)

    def test_occupancy_mismatch_fails_closed(self):
        image = self._source_image({slot.slot_id for slot in self.slots})
        expected = {slot.slot_id for slot in self.slots} - {"cap_3"}
        observation = self.inspector.inspect_bgr(
            image,
            frame_sequence=3,
            expected_present_ids=expected,
            image_points_px=self.image_corners,
            plane_points_m=self.plane_corners,
        )

        self.assertTrue(observation.fixture_valid)
        self.assertTrue(observation.classification_valid)
        self.assertFalse(observation.expectation_matches)
        self.assertFalse(observation.ready_for_stability)
        self.assertIn("extra=['cap_3']", observation.reason)

    def test_non_cap_red_shape_is_ambiguous_and_resets_stability(self):
        stabilizer = ObservationStabilizer(self.layout, self.config)
        first = stabilizer.add(self._inspect(10))
        malformed = self._inspect(11, malformed_id="cap_2")
        stopped = stabilizer.add(malformed)

        self.assertFalse(first.safe_to_move)
        self.assertFalse(malformed.classification_valid)
        self.assertIs(malformed.slots[2].occupancy, SlotOccupancy.AMBIGUOUS)
        self.assertFalse(stopped.safe_to_move)
        self.assertEqual(stopped.frame_sequences, ())

    def test_requires_distinct_stable_frames_before_motion(self):
        stabilizer = ObservationStabilizer(self.layout, self.config)
        self.assertFalse(stabilizer.add(self._inspect(20)).safe_to_move)
        self.assertFalse(stabilizer.add(self._inspect(21)).safe_to_move)
        stable = stabilizer.add(self._inspect(22))

        self.assertTrue(stable.safe_to_move, stable.reason)
        self.assertEqual(stable.frame_sequences, (20, 21, 22))
        replay = stabilizer.add(self._inspect(22))
        self.assertFalse(replay.safe_to_move)
        self.assertIn("stale", replay.reason)

    def test_homography_coverage_and_full_visibility_are_fail_closed(self):
        clustered_plane = np.asarray(
            ((0.010, 0.010), (0.030, 0.010), (0.030, 0.025), (0.010, 0.025)),
            dtype=np.float64,
        )
        clustered_image = np.asarray(
            ((100.0, 100.0), (180.0, 100.0), (180.0, 160.0), (100.0, 160.0)),
            dtype=np.float64,
        )
        with self.assertRaisesRegex(PerceptionError, "too"):
            estimate_planar_homography(
                clustered_plane,
                clustered_image,
                self.image_size,
                self.bounds,
                self.config,
            )

        outside = self.image_corners.copy()
        outside[0] = (-5.0, 82.0)
        failed = self._inspect(30, image_corners=outside)
        self.assertFalse(failed.fixture_valid)
        self.assertTrue(all(item.occupancy is SlotOccupancy.AMBIGUOUS for item in failed.slots))

    def test_marker_matching_uses_known_ids_and_rejects_duplicates(self):
        detected_ids = np.asarray(((99,), (13,), (10,)), dtype=np.int32)
        detected_corners = (
            np.zeros((1, 4, 2), dtype=np.float32),
            np.asarray(((((90, 450), (120, 450), (120, 490), (90, 490))),), dtype=np.float32),
            np.asarray(((((100, 80), (130, 80), (130, 120), (100, 120))),), dtype=np.float32),
        )
        image, plane, visible = marker_correspondences_from_detections(
            detected_corners,
            detected_ids,
            self.marker_layout,
        )
        self.assertEqual(visible, (10, 13))
        self.assertEqual(image.shape, (8, 2))
        self.assertEqual(plane.shape, (8, 2))

        with self.assertRaisesRegex(PerceptionError, "duplicate"):
            marker_correspondences_from_detections(
                (detected_corners[1], detected_corners[1]),
                (13, 13),
                self.marker_layout,
            )

    def test_unstable_fixture_pose_cannot_authorize_motion(self):
        strict = dataclasses.replace(self.config, max_homography_jitter_px=0.5)
        inspector = CapPerception(self.layout, strict)
        stabilizer = ObservationStabilizer(self.layout, strict)
        image = self._source_image()
        all_ids = {slot.slot_id for slot in self.slots}
        for sequence, shift in ((40, 0.0), (41, 0.3), (42, 1.2)):
            corners = self.image_corners + np.asarray((shift, 0.0))
            frame = inspector.inspect_bgr(
                image,
                frame_sequence=sequence,
                expected_present_ids=all_ids,
                image_points_px=corners,
                plane_points_m=self.plane_corners,
            )
            result = stabilizer.add(frame)
        self.assertFalse(result.safe_to_move)
        self.assertIn("jitter", result.reason)


if __name__ == "__main__":
    unittest.main()
