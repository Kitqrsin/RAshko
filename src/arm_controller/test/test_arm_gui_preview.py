import queue
import threading
import unittest
from types import SimpleNamespace

from arm_controller.arm_gui import ArmControlGUI


class _RecordingVisualization:
    def __init__(self):
        self.calls = []
        self.targets = []

    def set_joint_angles(self, angles):
        self.calls.append(tuple(angles))

    def set_target(self, x, y, z):
        self.targets.append((x, y, z))


class _RecordingVariable:
    def __init__(self):
        self.values = []

    def set(self, value):
        self.values.append(value)

    def get(self):
        return self.values[-1] if self.values else False


class _RecordingWidget:
    def __init__(self):
        self.text = None
        self.value = None

    def config(self, **kwargs):
        self.text = kwargs.get('text', self.text)

    def delete(self, _start, _end):
        self.value = ''

    def insert(self, _index, value):
        self.value = value


class ArmGuiPreviewTests(unittest.TestCase):
    def test_gui_only_mode_previews_desired_joint_angles(self):
        visualization = _RecordingVisualization()
        gui = SimpleNamespace(
            viz_3d=visualization,
            _feedback_initialized=False,
            _pending_initial_feedback=None,
            target_positions=[90.0, 80.0, 70.0, 60.0, 98.0, 0.0],
        )

        rendered = ArmControlGUI._preview_targets_without_feedback(gui)

        self.assertTrue(rendered)
        self.assertEqual(
            visualization.calls,
            [(90.0, 80.0, 70.0, 60.0, 98.0, 0.0)],
        )

    def test_live_feedback_remains_the_canvas_authority(self):
        visualization = _RecordingVisualization()
        gui = SimpleNamespace(
            viz_3d=visualization,
            _feedback_initialized=True,
            _pending_initial_feedback=None,
            target_positions=[90.0, 80.0, 70.0, 60.0, 98.0, 0.0],
        )

        rendered = ArmControlGUI._preview_targets_without_feedback(gui)

        self.assertFalse(rendered)
        self.assertEqual(visualization.calls, [])

    def test_pending_ros_feedback_already_blocks_command_preview(self):
        visualization = _RecordingVisualization()
        gui = SimpleNamespace(
            viz_3d=visualization,
            _feedback_initialized=False,
            _pending_initial_feedback=(90.0, 88.0, 90.0, 96.0, 98.0, 0.0),
            target_positions=[100.0, 80.0, 70.0, 60.0, 98.0, 0.0],
        )

        rendered = ArmControlGUI._preview_targets_without_feedback(gui)

        self.assertFalse(rendered)
        self.assertEqual(visualization.calls, [])

    def test_no_feedback_does_not_publish_a_fake_actual_gripper_marker(self):
        gui = SimpleNamespace(
            _feedback_initialized=False,
            _pending_initial_feedback=None,
        )

        published = ArmControlGUI.publish_gripper_marker(gui)

        self.assertFalse(published)

    def test_programmatic_joint_batch_previews_only_once(self):
        preview_calls = []
        gui = SimpleNamespace(
            target_positions=[90.0, 88.0, 90.0, 96.0, 98.0, 0.0],
            joint_values=[_RecordingVariable() for _ in range(6)],
            joint_labels=[_RecordingWidget() for _ in range(6)],
            _preview_targets_without_feedback=lambda: preview_calls.append(True),
        )
        target = [105.2, 80.4, 110.1, 70.3, 100.0, 0.0]

        changed = ArmControlGUI._set_joint_controls(gui, target)

        self.assertTrue(changed)
        self.assertEqual(gui.target_positions, target)
        self.assertEqual(preview_calls, [True])
        self.assertEqual(
            [variable.values for variable in gui.joint_values],
            [[value] for value in target[:5]] + [[]],
        )

    def test_drag_batch_updates_commands_without_repainting_joint_widgets(self):
        preview_calls = []
        gui = SimpleNamespace(
            target_positions=[90.0, 88.0, 90.0, 96.0, 98.0, 0.0],
            joint_values=[_RecordingVariable() for _ in range(6)],
            joint_labels=[_RecordingWidget() for _ in range(6)],
            _preview_targets_without_feedback=lambda: preview_calls.append(True),
        )
        target = [105.2, 80.4, 110.1, 70.3, 100.0, 0.0]

        changed = ArmControlGUI._set_joint_controls(
            gui,
            target,
            update_widgets=False,
        )

        self.assertTrue(changed)
        self.assertEqual(gui.target_positions, target)
        self.assertEqual(preview_calls, [True])
        self.assertEqual([variable.values for variable in gui.joint_values], [[]] * 6)

    def test_programmatic_cartesian_batch_updates_target_once(self):
        visualization = _RecordingVisualization()
        gui = SimpleNamespace(
            cartesian_values={
                axis: _RecordingVariable() for axis in ('X', 'Y', 'Z')
            },
            cartesian_entries={
                axis: _RecordingWidget() for axis in ('X', 'Y', 'Z')
            },
            cartesian_labels={
                axis: _RecordingWidget() for axis in ('X', 'Y', 'Z')
            },
            viz_3d=visualization,
        )

        ArmControlGUI._set_cartesian_controls(gui, 128.4, -127.8, 200.9)

        self.assertEqual(len(visualization.targets), 1)
        for actual, expected in zip(
            visualization.targets[0],
            (0.1284, -0.1278, 0.2009),
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(gui.cartesian_entries['X'].value, '128')
        self.assertEqual(gui.cartesian_entries['Y'].value, '-127')
        self.assertEqual(gui.cartesian_entries['Z'].value, '200')

    def test_camera_repaint_is_throttled_during_target_drag(self):
        gui = SimpleNamespace(
            _is_target_drag_active=lambda: True,
            _camera_motion_mode_active=lambda: False,
        )

        self.assertTrue(ArmControlGUI._prioritize_cartesian_animation(gui))

    def test_camera_repaint_stays_live_for_visual_servo(self):
        gui = SimpleNamespace(
            _is_target_drag_active=lambda: True,
            _camera_motion_mode_active=lambda: True,
        )

        self.assertFalse(ArmControlGUI._prioritize_cartesian_animation(gui))

    def test_camera_repaint_is_not_throttled_with_auto_follow_idle(self):
        gui = SimpleNamespace(
            viz_3d=SimpleNamespace(dragging=False),
            auto_follow_active=SimpleNamespace(get=lambda: True),
            _is_target_drag_active=lambda: False,
            _camera_motion_mode_active=lambda: False,
        )

        self.assertFalse(ArmControlGUI._prioritize_cartesian_animation(gui))

    def test_busy_drag_worker_keeps_only_the_latest_target(self):
        gui = SimpleNamespace(
            _drag_ik_sequence=0,
            _drag_ik_generation=3,
            _drag_ik_latest_request=None,
            _drag_ik_running=True,
            _schedule_drag_ik_poll=lambda: None,
        )

        ArmControlGUI._queue_drag_ik(gui, 0.10, 0.0, 0.20, False)
        ArmControlGUI._queue_drag_ik(gui, 0.14, -0.02, 0.23, True)

        self.assertEqual(gui._drag_ik_sequence, 2)
        self.assertEqual(
            gui._drag_ik_latest_request['target'],
            (0.14, -0.02, 0.23),
        )
        self.assertTrue(gui._drag_ik_latest_request['update_widgets'])

    def test_superseded_ik_result_is_not_applied(self):
        old_request = {
            'sequence': 1,
            'generation': 3,
            'target': (0.10, 0.0, 0.20),
            'update_widgets': False,
        }
        latest_request = {
            'sequence': 2,
            'generation': 3,
            'target': (0.14, -0.02, 0.23),
            'update_widgets': True,
        }
        results = queue.SimpleQueue()
        results.put((old_request, {'base': 90.0}, None))
        applied = []
        started = []
        gui = SimpleNamespace(
            _drag_ik_poll_after_id='poll-1',
            _drag_ik_result_queue=results,
            _drag_ik_running=True,
            _drag_ik_latest_request=latest_request,
            _drag_ik_generation=3,
            auto_follow_active=SimpleNamespace(get=lambda: True),
            _gui_shutdown=threading.Event(),
            _apply_3d_target_solution=lambda *args, **kwargs: applied.append(args),
            _start_drag_ik_request=lambda request: started.append(request),
            _schedule_drag_ik_poll=lambda: None,
        )

        ArmControlGUI._poll_drag_ik(gui)

        self.assertEqual(applied, [])
        self.assertEqual(started, [latest_request])


if __name__ == "__main__":
    unittest.main()
