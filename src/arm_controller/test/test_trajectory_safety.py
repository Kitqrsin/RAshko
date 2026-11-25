import math
import unittest

from arm_controller.joint_contract import HOME_DEGREES, JOINT_NAMES
from arm_controller.trajectory_safety import (
    TrajectoryValidationError,
    estimate_motion_timeout_seconds,
    max_target_error_degrees,
    validate_hardware_status_payload,
    validate_trajectory_rows,
)


class TrajectorySafetyTests(unittest.TestCase):
    def test_accepts_five_joint_moveit_trajectory_and_preserves_gripper(self):
        rows = validate_trajectory_rows(
            JOINT_NAMES[:5],
            [[0.0] * 5, [math.radians(5.0)] + [0.0] * 4],
            [0.0, 1.0],
            [*HOME_DEGREES[:5], 23.0],
        )

        self.assertEqual(rows[0][5], 23.0)
        self.assertEqual(rows[1][5], 23.0)
        self.assertAlmostEqual(rows[1][0], HOME_DEGREES[0] + 5.0)

    def test_accepts_all_six_canonical_joints(self):
        rows = validate_trajectory_rows(
            JOINT_NAMES,
            [[0.0] * 6],
            [0.0],
            HOME_DEGREES,
        )
        self.assertEqual(rows, (HOME_DEGREES,))

    def test_accepts_gripper_only_and_preserves_arm(self):
        rows = validate_trajectory_rows(
            (JOINT_NAMES[5],),
            [[math.radians(15.0)]],
            [1.0],
            HOME_DEGREES,
        )
        self.assertEqual(rows[0][:5], HOME_DEGREES[:5])
        self.assertAlmostEqual(rows[0][5], 15.0)

    def test_rejects_arbitrary_partial_reordered_and_unknown_joint_lists(self):
        invalid_names = [
            ("base",),
            tuple(reversed(JOINT_NAMES[:5])),
            (*JOINT_NAMES[:4], "not_a_joint"),
        ]
        for names in invalid_names:
            with self.subTest(names=names):
                with self.assertRaises(TrajectoryValidationError):
                    validate_trajectory_rows(
                        names,
                        [[0.0] * len(names)],
                        [0.0],
                        HOME_DEGREES,
                    )

    def test_rejects_empty_nonfinite_and_nonmonotonic_trajectories(self):
        with self.assertRaises(TrajectoryValidationError):
            validate_trajectory_rows(JOINT_NAMES[:5], [], [], HOME_DEGREES)
        with self.assertRaises(TrajectoryValidationError):
            validate_trajectory_rows(
                JOINT_NAMES[:5], [[float("nan")] * 5], [0.0], HOME_DEGREES
            )
        with self.assertRaises(TrajectoryValidationError):
            validate_trajectory_rows(
                JOINT_NAMES[:5], [[0.0] * 5, [0.0] * 5], [1.0, 1.0], HOME_DEGREES
            )

    def test_rejects_joint_limit_violations(self):
        with self.assertRaises(TrajectoryValidationError):
            validate_trajectory_rows(
                JOINT_NAMES,
                [[math.radians(500.0)] + [0.0] * 5],
                [0.0],
                HOME_DEGREES,
            )

    def test_target_error_checks_only_controlled_joints(self):
        current = list(HOME_DEGREES)
        target = list(HOME_DEGREES)
        target[0] += 1.5
        target[5] += 90.0

        self.assertEqual(
            max_target_error_degrees(current, target, JOINT_NAMES[:5]), 1.5
        )

    def test_timeout_scales_with_commanded_delta_and_speed(self):
        target = list(HOME_DEGREES)
        target[0] += 90.0
        timeout = estimate_motion_timeout_seconds(
            HOME_DEGREES,
            target,
            JOINT_NAMES[:5],
            speed_percent=10.0,
            maximum_speed_deg_per_second=180.0,
        )
        self.assertAlmostEqual(timeout, 8.5)

    def test_rejects_excessive_horizon(self):
        with self.assertRaisesRegex(TrajectoryValidationError, "horizon"):
            validate_trajectory_rows(
                JOINT_NAMES[:5],
                [[0.0] * 5],
                [121.0],
                HOME_DEGREES,
            )

    def test_validates_monotonic_hardware_ack_status(self):
        status = {
            "host": "192.0.2.1",
            "port": 5005,
            "sequence": 10,
            "code": "OK",
            "state": ["HARDWARE_READY"],
            "angles_deg": list(HOME_DEGREES),
        }
        sequence, code, states, angles = validate_hardware_status_payload(status)
        self.assertEqual((sequence, code), (10, "OK"))
        self.assertEqual(states, ("HARDWARE_READY",))
        self.assertEqual(angles, HOME_DEGREES)
        with self.assertRaisesRegex(TrajectoryValidationError, "not newer"):
            validate_hardware_status_payload(status, previous_sequence=10)

    def test_rejects_hardware_status_outside_canonical_joint_ranges(self):
        status = {
            "host": "192.0.2.1",
            "port": 5005,
            "sequence": 11,
            "code": "OK",
            "state": ["HARDWARE_READY"],
            "angles_deg": [246.0, *HOME_DEGREES[1:]],
        }
        with self.assertRaisesRegex(TrajectoryValidationError, "joint contract"):
            validate_hardware_status_payload(status)


if __name__ == "__main__":
    unittest.main()
