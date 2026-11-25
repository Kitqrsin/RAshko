"""GUI command handover regressions; no ROS node or hardware is started."""

import queue
import threading
import unittest
from types import MethodType, SimpleNamespace

from arm_controller.arm_gui import ArmControlGUI
from arm_controller.joint_contract import HOME_DEGREES


class Variable:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class Scheduler:
    def __init__(self):
        self.callbacks = {}
        self.cancelled = set()

    def after(self, _delay, callback):
        token = len(self.callbacks)
        self.callbacks[token] = callback
        return token

    def after_cancel(self, token):
        self.cancelled.add(token)

    def run(self, token):
        # Deliberately permit cancelled callbacks to model an already-dequeued
        # callback. Its captured generation must still prevent stale motion.
        self.callbacks[token]()


class Entry:
    def __init__(self, text):
        self.text = text

    def get(self):
        return self.text

    def delete(self, *_args):
        self.text = ''

    def insert(self, _index, text):
        self.text = text


def make_gui(realtime=False, auto_follow=False, feedback=False):
    widget = SimpleNamespace(config=lambda **kwargs: None)
    gui = SimpleNamespace(
        root=Scheduler(),
        target_positions=[110.0, 75.0, 110.0, 82.0, 98.0, 0.0],
        current_positions=list(HOME_DEGREES),
        joint_values=[Variable(value) for value in HOME_DEGREES],
        joint_labels=[widget] * 6,
        info_label=widget,
        start_stop_btn=widget,
        realtime_mode=Variable(realtime),
        auto_follow_active=Variable(auto_follow),
        _feedback_initialized=feedback,
        _pending_initial_feedback=None,
        _gui_shutdown=threading.Event(),
        _drag_ik_generation=0,
        _drag_ik_sequence=0,
        _drag_ik_running=False,
        _drag_ik_latest_request=None,
        _drag_ik_result_queue=queue.SimpleQueue(),
        _drag_ik_poll_after_id=None,
        _update_after_id=None,
        _cartesian_update_after_id=None,
        _interpolation_after_id=None,
        update_pending=False,
        cartesian_update_pending=False,
        update_interval=0.1,
        commands=[],
        drawings=[],
        cartesian_calls=[],
        cartesian_entries={axis: Entry(text) for axis, text in
                           (('X', '100'), ('Y', '0'), ('Z', '200'))},
        _cartesian_entry_text={'X': '100', 'Y': '0', 'Z': '200'},
        cartesian_values={axis: Variable(value) for axis, value in
                          (('X', 100), ('Y', 0), ('Z', 200))},
        cartesian_labels={axis: widget for axis in ('X', 'Y', 'Z')},
        get_logger=lambda: SimpleNamespace(info=lambda *args: None),
    )
    gui._rendered_joint_positions = list(gui.target_positions)
    gui.viz_3d = SimpleNamespace(
        set_auto_follow=lambda enabled: None,
        set_view=lambda *args: None,
        set_joint_angles=lambda angles: gui.drawings.append(tuple(angles)),
        set_target=lambda *args: None,
    )
    gui.send_command = lambda **kwargs: gui.commands.append(tuple(gui.target_positions))
    gui.apply_cartesian_silent = lambda: gui.cartesian_calls.append(True)
    gui.publish_target_marker = lambda *args: None
    for name in (
        '_cancel_pending_motion', '_take_manual_control', 'send_manual_command',
        '_set_joint_controls', '_preview_targets_without_feedback',
        'toggle_auto_follow', 'apply_preset', 'on_slider_change',
        'schedule_update', 'do_update', 'schedule_cartesian_update',
        'do_cartesian_update', 'interpolate_to_target', '_poll_drag_ik',
        '_apply_3d_target_solution',
        '_on_realtime_toggle', 'on_cartesian_slider_change', 'on_cartesian_entry_change',
    ):
        setattr(gui, name, MethodType(getattr(ArmControlGUI, name), gui))
    return gui


class MotionOwnershipTests(unittest.TestCase):
    def test_home_invalidates_worker_and_sends_once_in_realtime(self):
        gui = make_gui(realtime=True, auto_follow=True)
        request = dict(sequence=1, generation=0, target=(0.1, 0.0, 0.2),
                       update_widgets=True)
        gui._drag_ik_latest_request = request
        gui._drag_ik_running = True
        gui._drag_ik_result_queue.put((request, {'base': 130.0}, None))

        gui.apply_preset(HOME_DEGREES)
        gui._poll_drag_ik()
        gui.root.run(gui._update_after_id)

        self.assertFalse(gui.auto_follow_active.get())
        self.assertEqual(gui.target_positions, list(HOME_DEGREES))
        self.assertEqual(gui.commands, [HOME_DEGREES])
        self.assertIsNone(gui._drag_ik_latest_request)

    def test_non_realtime_home_only_stages_even_with_pending_joint_send(self):
        gui = make_gui()
        gui.schedule_update()
        stale_send = gui._update_after_id

        gui.apply_preset(HOME_DEGREES)
        gui.root.run(stale_send)

        self.assertEqual(gui.commands, [])
        self.assertEqual(gui.target_positions, list(HOME_DEGREES))
        gui.send_manual_command()
        self.assertEqual(gui.commands, [HOME_DEGREES])

    def test_manual_joint_edit_cancels_interpolated_cartesian_move(self):
        gui = make_gui()
        gui.interpolate_to_target([125.0, 60.0, 100.0, 80.0, 98.0, 0.0])
        stale_step = gui._interpolation_after_id
        gui.commands.clear()

        gui._take_manual_control()
        gui.on_slider_change(0, 95.0)
        manual_pose = tuple(gui.target_positions)
        gui.root.run(stale_step)
        gui.send_manual_command()

        self.assertEqual(gui.commands, [manual_pose])
        self.assertEqual(gui.target_positions[0], 95.0)

    def test_home_cancels_pending_cartesian_slider_solve(self):
        gui = make_gui(realtime=True)
        gui.schedule_cartesian_update()
        stale_solve = gui._cartesian_update_after_id

        gui.apply_preset(HOME_DEGREES)
        current_send = gui._update_after_id
        gui.root.run(stale_solve)
        gui.root.run(current_send)

        self.assertEqual(gui.cartesian_calls, [])
        self.assertEqual(gui.commands, [HOME_DEGREES])

    def test_second_interpolation_replaces_first_without_interleaving(self):
        gui = make_gui()
        gui.interpolate_to_target([130.0, 60.0, 100.0, 80.0, 98.0, 0.0], steps=1)
        first_step = gui._interpolation_after_id
        gui.interpolate_to_target(HOME_DEGREES, steps=1)
        second_step = gui._interpolation_after_id
        gui.commands.clear()

        gui.root.run(first_step)
        gui.root.run(second_step)

        self.assertEqual(gui.commands, [HOME_DEGREES])

    def test_stop_cancels_deferred_cartesian_solve(self):
        gui = make_gui(auto_follow=True)
        gui.schedule_cartesian_update()
        stale_solve = gui._cartesian_update_after_id
        gui.toggle_auto_follow()
        gui.root.run(stale_solve)

        self.assertFalse(gui.auto_follow_active.get())
        self.assertEqual(gui.cartesian_calls, [])
        self.assertEqual(gui.commands, [])

    def test_programmatic_camera_slider_callback_does_not_take_ownership(self):
        gui = make_gui(auto_follow=True)
        gui.on_slider_change(0, 93.0)
        self.assertTrue(gui.auto_follow_active.get())
        self.assertEqual(gui._drag_ik_generation, 0)

    def test_live_canvas_is_never_painted_with_desired_ik_or_preset_pose(self):
        gui = make_gui(auto_follow=True, feedback=True)
        solution = dict(zip(
            ('base', 'shoulder', 'elbow', 'forearm', 'wrist', 'gripper'),
            (125.0, 60.0, 100.0, 80.0, 98.0, 0.0),
        ))
        gui._apply_3d_target_solution(0.1, 0.0, 0.2, solution)
        gui.apply_preset(HOME_DEGREES)
        self.assertEqual(gui.drawings, [])

    def test_live_canvas_is_not_painted_with_interpolation_targets(self):
        gui = make_gui(feedback=True)
        gui.interpolate_to_target(HOME_DEGREES, steps=1)
        gui.root.run(gui._interpolation_after_id)
        self.assertEqual(gui.drawings, [])

    def test_realtime_disable_revokes_joint_and_cartesian_callbacks(self):
        for schedule, token in (('schedule_update', '_update_after_id'),
                                ('schedule_cartesian_update', '_cartesian_update_after_id')):
            with self.subTest(schedule=schedule):
                gui = make_gui(realtime=True)
                getattr(gui, schedule)()
                stale_callback = getattr(gui, token)
                gui.realtime_mode.set(False)
                gui._on_realtime_toggle()
                gui.root.run(stale_callback)
                self.assertEqual(gui.commands, [])
                self.assertEqual(gui.cartesian_calls, [])

    def test_delayed_handlers_recheck_realtime_even_without_toggle_callback(self):
        gui = make_gui(realtime=False)
        gui.do_update()
        gui.do_cartesian_update()
        self.assertEqual(gui.commands, [])
        self.assertEqual(gui.cartesian_calls, [])

    def test_xyz_edit_with_realtime_off_invalidates_drag_result(self):
        for entry_edit in (False, True):
            with self.subTest(entry_edit=entry_edit):
                gui = make_gui(auto_follow=True)
                request = dict(sequence=1, generation=0, target=(0.1, 0.0, 0.2),
                               update_widgets=True)
                gui._drag_ik_latest_request = request
                gui._drag_ik_running = True
                gui._drag_ik_result_queue.put((request, {'base': 130.0}, None))
                if entry_edit:
                    gui.cartesian_entries['X'].text = '120'
                    gui.on_cartesian_entry_change('X')
                else:
                    gui.on_cartesian_slider_change('X', 120)
                gui._poll_drag_ik()
                self.assertEqual(gui.commands, [])
                self.assertEqual(gui.cartesian_calls, [])
                self.assertEqual(gui.cartesian_entries['X'].get(), '120')

    def test_focusout_after_manual_takeover_does_not_schedule_cartesian_work(self):
        for text in ('100', '120'):
            with self.subTest(entry_text=text):
                gui = make_gui(realtime=True)
                gui.cartesian_entries['X'].text = text
                gui._take_manual_control()
                gui.on_cartesian_entry_change('X')
                self.assertFalse(gui.cartesian_update_pending)
                self.assertEqual(gui.cartesian_entries['X'].get(), '100')

    def test_continuous_xyz_edits_keep_the_original_realtime_deadline(self):
        gui = make_gui(realtime=True)
        gui.on_cartesian_slider_change('X', 110)
        deadline = gui._cartesian_update_after_id
        gui.on_cartesian_slider_change('X', 120)
        self.assertEqual(gui._cartesian_update_after_id, deadline)
        gui.root.run(deadline)
        self.assertEqual(gui.cartesian_calls, [True])
        self.assertEqual(gui.cartesian_entries['X'].get(), '120')

    def test_repeated_manual_keys_do_not_starve_joint_realtime_updates(self):
        gui = make_gui(realtime=True)
        gui._take_manual_control(SimpleNamespace())
        gui.on_slider_change(0, 94)
        deadline = gui._update_after_id
        gui._take_manual_control(SimpleNamespace())
        gui.on_slider_change(0, 95)
        self.assertEqual(gui._update_after_id, deadline)
        gui.root.run(deadline)
        self.assertEqual(gui.commands[0][0], 95)


if __name__ == '__main__':
    unittest.main()
