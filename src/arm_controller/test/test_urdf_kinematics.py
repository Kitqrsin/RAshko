import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from arm_controller.joint_contract import (
    HOME_DEGREES,
    JOINT_NAMES,
    SERVO_MAX_DEGREES,
    SERVO_MIN_DEGREES,
    servo_degrees_to_radians,
)
from arm_controller.urdf_kinematics import UrdfKinematics, compact_tool_chain


class UrdfKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        project_root = Path(__file__).resolve().parents[3]
        cls.description = (project_root / "robot_urdf" / "robot_fixed.urdf").read_text(
            encoding="utf-8"
        )
        cls.model = UrdfKinematics(cls.description)
        cls.home = servo_degrees_to_radians(HOME_DEGREES)[:5]

    def test_urdf_limits_match_the_canonical_logical_servo_ranges(self):
        minimum_radians = servo_degrees_to_radians(SERVO_MIN_DEGREES)
        maximum_radians = servo_degrees_to_radians(SERVO_MAX_DEGREES)
        root = ET.fromstring(self.description)

        for index, name in enumerate(JOINT_NAMES):
            joint = root.find(f"./joint[@name='{name}']")
            self.assertIsNotNone(joint, name)
            limit = joint.find("limit")
            self.assertIsNotNone(limit, name)
            expected_lower, expected_upper = sorted(
                (minimum_radians[index], maximum_radians[index])
            )
            self.assertTrue(
                math.isclose(float(limit.attrib["lower"]), expected_lower, abs_tol=1e-12),
                name,
            )
            self.assertTrue(
                math.isclose(float(limit.attrib["upper"]), expected_upper, abs_tol=1e-12),
                name,
            )

    def test_forward_kinematics_matches_robot_state_publisher_home_pose(self):
        result = self.model.forward(self.home)
        np.testing.assert_allclose(
            result.position,
            (0.00168781, -0.02615382, 0.37687685),
            atol=1e-7,
        )
        np.testing.assert_allclose(
            result.tool_direction,
            (-0.02050078, -0.05274810, 0.99839739),
            atol=1e-7,
        )

    def test_gui_chain_keeps_exact_geometry_without_the_cad_model(self):
        chain = compact_tool_chain(self.description)
        root = ET.fromstring(chain)
        self.assertEqual(len(root.findall('joint')), 6)
        self.assertFalse(root.findall('.//mesh'))
        self.assertFalse(root.findall('.//visual'))
        self.assertFalse(root.findall('.//collision'))
        gui_model = UrdfKinematics(chain)
        angles = servo_degrees_to_radians((105, 80, 110, 70, 100, 0))[:5]
        pose = gui_model.forward(angles)
        reference = self.model.forward(angles)
        self.assertEqual(len(pose.joint_positions), 5)
        np.testing.assert_array_equal(pose.joint_positions, reference.joint_positions)
        np.testing.assert_array_equal(pose.position, reference.position)
        np.testing.assert_array_equal(pose.tool_direction, reference.tool_direction)

    def test_downward_pose_is_not_solved_as_upward(self):
        solution = self.model.solve(
            (-0.0636, -0.0660, 0.0090),
            -math.pi / 2.0,
            self.home,
            wrist_roll=0.0,
        )
        self.assertIsNotNone(solution)
        result = self.model.forward(solution.radians)
        self.assertLess(solution.position_error, 0.005)
        self.assertLess(solution.direction_error, math.radians(2.0))
        self.assertLess(result.tool_direction[2], -0.99)

    def test_horizontal_pose_uses_exact_urdf_geometry(self):
        solution = self.model.solve(
            (0.18, 0.10, 0.18),
            0.0,
            self.home,
            wrist_roll=0.0,
        )
        self.assertIsNotNone(solution)
        result = self.model.forward(solution.radians)
        self.assertLess(solution.position_error, 0.005)
        self.assertLess(abs(result.tool_direction[2]), 0.01)

    def test_position_only_mode_leaves_tool_elevation_free(self):
        target_radians = servo_degrees_to_radians(
            (120.0, 55.0, 135.0, 75.0, 98.0, HOME_DEGREES[5])
        )[:5]
        target = self.model.forward(target_radians).position
        solution = self.model.solve(
            target,
            None,
            self.home,
            wrist_roll=0.0,
        )
        self.assertIsNotNone(solution)
        self.assertLess(solution.position_error, 0.005)
        self.assertEqual(solution.direction_error, 0.0)


if __name__ == "__main__":
    unittest.main()
