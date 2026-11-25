import math
import unittest

from arm_controller.tool_pose import tool_direction, tool_quaternion


class ToolPoseTests(unittest.TestCase):
    def assertDirectionAlmostEqual(self, actual, expected):
        for actual_value, expected_value in zip(actual, expected):
            self.assertAlmostEqual(actual_value, expected_value, places=7)

    def test_horizontal_tool_points_at_target_azimuth(self):
        quaternion = tool_quaternion(math.pi / 3.0, 0.0)
        self.assertDirectionAlmostEqual(
            tool_direction(quaternion),
            (0.5, math.sqrt(3.0) / 2.0, 0.0),
        )

    def test_up_and_down_match_physical_gripper_axis(self):
        up = tool_direction(tool_quaternion(0.0, math.pi / 2.0))
        down = tool_direction(tool_quaternion(0.0, -math.pi / 2.0))
        self.assertDirectionAlmostEqual(up, (0.0, 0.0, 1.0))
        self.assertDirectionAlmostEqual(down, (0.0, 0.0, -1.0))

    def test_roll_does_not_reverse_tool_direction(self):
        expected = tool_direction(tool_quaternion(-0.7, -0.4, 0.0))
        actual = tool_direction(tool_quaternion(-0.7, -0.4, 1.2))
        self.assertDirectionAlmostEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
