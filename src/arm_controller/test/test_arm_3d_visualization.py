import math
import unittest
from types import SimpleNamespace

import numpy as np

from arm_controller.arm_3d_visualization import Arm3DVisualization
from arm_controller.ik_solver import ArmIKSolver
from arm_controller.joint_contract import HOME_DEGREES


class _QueuedCanvas:
    def __init__(self):
        self.callbacks = []
        self.deleted = []
        self.lowered = []

    def after(self, delay_ms, callback):
        self.callbacks.append((delay_ms, callback))
        return f"draw-{len(self.callbacks)}"

    def delete(self, tag):
        self.deleted.append(tag)

    def tag_lower(self, tag, below):
        self.lowered.append((tag, below))


class _RetainedCanvas:
    def __init__(self):
        self.created = []
        self.updated = []

    def _create(self, kind, _coordinates, _options):
        item_id = len(self.created) + 1
        self.created.append((item_id, kind))
        return item_id

    def create_line(self, *coordinates, **options):
        return self._create('line', coordinates, options)

    def create_oval(self, *coordinates, **options):
        return self._create('oval', coordinates, options)

    def create_text(self, *coordinates, **options):
        return self._create('text', coordinates, options)

    def coords(self, item_id, *coordinates):
        self.updated.append((item_id, coordinates))


class Arm3DVisualizationTests(unittest.TestCase):
    def setUp(self):
        self.solver = ArmIKSolver()

    def make_visualization(self, angles):
        # calculate_arm_positions does not require Tk widgets.
        visualization = Arm3DVisualization.__new__(Arm3DVisualization)
        visualization.ik_solver = self.solver
        visualization.joint_angles = list(angles)
        visualization.L1 = self.solver.L1
        visualization.L2 = self.solver.L2
        visualization.L3 = self.solver.L3
        visualization.L4 = self.solver.L4
        return visualization

    def make_renderable_visualization(self, angles=HOME_DEGREES):
        visualization = self.make_visualization(angles)
        visualization.canvas = _RetainedCanvas()
        visualization.view_azimuth = 45
        visualization.view_elevation = 25
        visualization.zoom = 550
        visualization.center_offset_x = 250
        visualization.center_offset_y = 360
        visualization._arm_item_ids = []
        visualization._target_item_ids = []
        visualization.target_x = 0.1
        visualization.target_y = 0.0
        visualization.target_z = 0.2
        return visualization

    def test_canvas_always_draws_the_original_five_points(self):
        visualization = self.make_visualization(HOME_DEGREES)

        positions = visualization.calculate_arm_positions()

        self.assertEqual(len(positions), 5)

    def test_canvas_gripper_matches_geometric_forward_kinematics(self):
        angles = (120.0, 72.0, 118.0, 83.0, 98.0, 0.0)
        visualization = self.make_visualization(angles)

        positions = visualization.calculate_arm_positions()
        expected = self.solver.forward_kinematics(*angles[:5])

        np.testing.assert_allclose(positions[-1], expected[:3], atol=1e-12)

    def test_fixed_tf_frames_cannot_become_canvas_joints(self):
        visualization = self.make_visualization(HOME_DEGREES)
        # This stale attribute models the previous regression.  The restored
        # geometric canvas must ignore it rather than drawing seven TF origins.
        visualization._urdf_link_positions = [(float(i), 0.0, 0.0) for i in range(7)]

        self.assertEqual(len(visualization.calculate_arm_positions()), 5)

    def test_point_solver_lands_on_a_reachable_geometric_target(self):
        target_pose = (132.0, 68.0, 124.0, 78.0, 98.0)
        target = self.solver.forward_kinematics(*target_pose)[:3]
        distant_seed = {
            "base": 5.0,
            "shoulder": 175.0,
            "elbow": 5.0,
            "forearm": 175.0,
            "wrist": 98.0,
        }

        solution = self.solver.solve_ik_to_point(
            *target,
            seed_state=distant_seed,
        )

        self.assertIsNotNone(solution)
        achieved = self.solver.forward_kinematics(
            solution["base"],
            solution["shoulder"],
            solution["elbow"],
            solution["forearm"],
            solution["wrist"],
        )[:3]
        error = math.dist(target, achieved)
        self.assertLess(error, 0.001)

    def test_folded_back_pose_uses_the_second_base_branch(self):
        # This target is behind the base even though the base servo itself is
        # at logical 90 degrees. Treating radius as always positive used to
        # demand an impossible 270-degree base command.
        target_pose = (90.0, 120.0, 120.0, 120.0, 98.0)
        target = self.solver.forward_kinematics(*target_pose)[:3]

        solution = self.solver.solve_ik_to_point(*target)

        self.assertIsNotNone(solution)
        achieved = self.solver.forward_kinematics(
            solution["base"],
            solution["shoulder"],
            solution["elbow"],
            solution["forearm"],
            solution["wrist"],
        )[:3]
        self.assertLess(math.dist(target, achieved), 0.001)

    def test_unreachable_target_is_not_replaced_with_a_nearby_point(self):
        self.assertIsNone(self.solver.solve_ik_to_point(0.60, 0.0, 0.30))

    def test_cartesian_axis_steps_follow_the_requested_direction(self):
        names = ('base', 'shoulder', 'elbow', 'forearm', 'wrist')
        initial = (90.0, 45.0, 120.0, 80.0, 98.0)
        origin = self.solver.forward_kinematics(*initial)[:3]
        for axis in range(3):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    target = list(origin)
                    target[axis] += sign * 0.005
                    solution = self.solver.solve_ik_to_point(
                        *target, seed_state=dict(zip(names, initial))
                    )
                    self.assertIsNotNone(solution)
                    achieved = self.solver.forward_kinematics(
                        *(solution[name] for name in names)
                    )[:3]
                    self.assertLess(math.dist(target, achieved), 0.001)
                    self.assertGreater(sign * (achieved[axis] - origin[axis]), 0)

    def test_mouse_drag_projection_is_not_mirrored_in_any_standard_view(self):
        for azimuth, elevation in ((0, 0), (90, 0), (0, 90), (45, 25)):
            with self.subTest(view=(azimuth, elevation)):
                viz = self.make_renderable_visualization()
                viz.view_azimuth, viz.view_elevation = azimuth, elevation
                viz.target_x, viz.target_y, viz.target_z = 0.10, 0.03, 0.16
                viz.dragging = True
                viz.auto_follow = False
                viz.drag_start_x = viz.drag_start_y = 100
                viz.request_draw = lambda *args: None
                before = viz.project_3d_to_2d(viz.target_x, viz.target_y, viz.target_z)
                viz.on_mouse_drag(SimpleNamespace(x=108, y=94))
                after = viz.project_3d_to_2d(viz.target_x, viz.target_y, viz.target_z)
                self.assertAlmostEqual(after[0] - before[0], 8.0)
                self.assertAlmostEqual(after[1] - before[1], -6.0)

    def test_identical_joint_feedback_does_not_queue_a_redraw(self):
        visualization = self.make_visualization(HOME_DEGREES)
        visualization.canvas = _QueuedCanvas()
        visualization._draw_after_id = None
        visualization._draw_dirty = 0

        changed = visualization.set_joint_angles(HOME_DEGREES)

        self.assertFalse(changed)
        self.assertEqual(visualization.canvas.callbacks, [])

    def test_joint_update_burst_is_coalesced_to_the_latest_frame(self):
        visualization = self.make_visualization(HOME_DEGREES)
        visualization.canvas = _QueuedCanvas()
        visualization._draw_after_id = None
        visualization._draw_dirty = 0
        rendered = []
        visualization.draw_arm = lambda: rendered.append(
            ('arm', tuple(visualization.joint_angles))
        )
        visualization.draw_gripper_marker = lambda: rendered.append(
            ('gripper', tuple(visualization.joint_angles))
        )

        first = (100.0, 80.0, 90.0, 96.0, 98.0, 0.0)
        final = (110.0, 75.0, 105.0, 85.0, 98.0, 0.0)
        visualization.set_joint_angles(first)
        visualization.set_joint_angles(final)

        self.assertEqual(len(visualization.canvas.callbacks), 1)
        delay_ms, callback = visualization.canvas.callbacks[0]
        self.assertEqual(delay_ms, visualization.RENDER_INTERVAL_MS)
        callback()
        self.assertEqual(
            rendered,
            [('arm', final), ('gripper', final)],
        )
        self.assertEqual(visualization.canvas.deleted, [])

    def test_target_update_redraws_only_the_target_layer(self):
        visualization = self.make_visualization(HOME_DEGREES)
        visualization.canvas = _QueuedCanvas()
        visualization._draw_after_id = None
        visualization._draw_dirty = 0
        visualization.target_x = 0.0
        visualization.target_y = 0.0
        visualization.target_z = 0.2
        visualization.update_position_label = lambda: None
        rendered = []
        visualization.draw_target = lambda: rendered.append('target')
        visualization.draw_arm = lambda: self.fail('arm layer was redrawn')
        visualization.draw_scene = lambda: self.fail('full scene was redrawn')

        visualization.set_target(0.1, -0.1, 0.25)
        _delay_ms, callback = visualization.canvas.callbacks[0]
        callback()

        self.assertEqual(rendered, ['target'])
        self.assertEqual(visualization.canvas.deleted, [])

    def test_arm_layer_reuses_nine_canvas_items_between_frames(self):
        visualization = self.make_renderable_visualization()

        visualization.draw_arm()
        visualization.joint_angles = [100.0, 80.0, 95.0, 90.0, 98.0, 0.0]
        visualization.draw_arm()

        self.assertEqual(len(visualization.canvas.created), 9)
        self.assertEqual(len(visualization.canvas.updated), 9)

    def test_target_layer_reuses_six_canvas_items_between_frames(self):
        visualization = self.make_renderable_visualization()

        visualization.draw_target()
        visualization.target_x = 0.12
        visualization.draw_target()

        self.assertEqual(len(visualization.canvas.created), 6)
        self.assertEqual(len(visualization.canvas.updated), 6)

    def test_joint_and_target_updates_share_one_partial_frame(self):
        visualization = self.make_visualization(HOME_DEGREES)
        visualization.canvas = _QueuedCanvas()
        visualization._draw_after_id = None
        visualization._draw_dirty = 0
        visualization.target_x = 0.0
        visualization.target_y = 0.0
        visualization.target_z = 0.2
        visualization.update_position_label = lambda: None
        rendered = []
        visualization.draw_arm = lambda: rendered.append('arm')
        visualization.draw_gripper_marker = lambda: rendered.append('gripper')
        visualization.draw_target = lambda: rendered.append('target')

        visualization.set_joint_angles((100, 80, 90, 96, 98, 0))
        visualization.set_target(0.1, 0.0, 0.2)

        self.assertEqual(len(visualization.canvas.callbacks), 1)
        _delay_ms, callback = visualization.canvas.callbacks[0]
        callback()
        self.assertEqual(rendered, ['arm', 'gripper', 'target'])

    def test_full_view_update_supersedes_a_pending_partial_frame(self):
        visualization = self.make_visualization(HOME_DEGREES)
        visualization.canvas = _QueuedCanvas()
        visualization._draw_after_id = None
        visualization._draw_dirty = 0
        visualization.view_azimuth = 45
        visualization.view_elevation = 25
        rendered = []
        visualization.draw_scene = lambda: rendered.append('full')

        visualization.set_joint_angles((100, 80, 90, 96, 98, 0))
        visualization.set_view(90, 0)

        self.assertEqual(len(visualization.canvas.callbacks), 1)
        _delay_ms, callback = visualization.canvas.callbacks[0]
        callback()
        self.assertEqual(rendered, ['full'])


if __name__ == "__main__":
    unittest.main()
