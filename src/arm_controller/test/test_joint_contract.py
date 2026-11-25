import math
import unittest

from arm_controller.joint_contract import (
    GRIPPER_OPEN_DEGREES,
    HOME_DEGREES,
    merge_joint_radians,
    radians_to_servo_degrees,
    servo_degrees_to_radians,
    validate_servo_degrees,
)


class JointContractTests(unittest.TestCase):
    def test_calibrated_home_matches_the_deployed_controller(self):
        self.assertEqual(HOME_DEGREES, (90.0, 88.0, 90.0, 96.0, 98.0, 0.0))
        self.assertEqual(GRIPPER_OPEN_DEGREES, 0.0)

    def test_home_maps_to_zero_radians(self):
        self.assertEqual(servo_degrees_to_radians(HOME_DEGREES), (0.0,) * 6)

    def test_inverted_urdf_axes_round_trip(self):
        degrees = (120.0, 70.0, 110.0, 60.0, 130.0, 45.0)
        radians = servo_degrees_to_radians(degrees)
        self.assertGreater(radians[0], 0.0)
        self.assertGreater(radians[1], 0.0)
        self.assertLess(radians[2], 0.0)
        self.assertGreater(radians[3], 0.0)
        recovered = merge_joint_radians(
            ("base", "shoulder", "elbow", "wrist", "wrist_rotate", "gripper_base"),
            radians,
        )
        for expected, actual in zip(degrees, recovered):
            self.assertTrue(math.isclose(expected, actual, abs_tol=1e-9))

    def test_partial_trajectory_preserves_unmentioned_joints(self):
        current = (100.0, 90.0, 80.0, 70.0, 60.0, 50.0)
        merged = merge_joint_radians(("base", "elbow"), (0.0, 0.0), current)
        self.assertEqual(merged, (90.0, 90.0, 90.0, 70.0, 60.0, 50.0))

    def test_base_extended_range_and_other_joint_limits(self):
        self.assertEqual(validate_servo_degrees((245, 180, 180, 180, 180, 180))[0], 245.0)
        with self.assertRaises(ValueError):
            validate_servo_degrees((246, 90, 90, 90, 90, 90))
        with self.assertRaises(ValueError):
            validate_servo_degrees((90, 181, 90, 90, 90, 90))

    def test_tiny_limit_roundoff_is_snapped_but_real_violations_are_rejected(self):
        normalized = validate_servo_degrees(
            (90, 88, 90, -0.0000005, 180.0000005, 0)
        )
        self.assertEqual(normalized[3], 0.0)
        self.assertEqual(normalized[4], 180.0)

        with self.assertRaises(ValueError):
            validate_servo_degrees((90, 88, 90, -0.000002, 98, 0))
        with self.assertRaises(ValueError):
            validate_servo_degrees((90, 88, 90, 96, 180.000002, 0))

    def test_radian_conversion_snaps_an_exact_limit_with_roundoff(self):
        radians_just_past_wrist_zero = math.radians(HOME_DEGREES[3] + 0.0000005)
        self.assertEqual(
            radians_to_servo_degrees(3, radians_just_past_wrist_zero),
            0.0,
        )


if __name__ == "__main__":
    unittest.main()
