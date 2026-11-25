"""Opt-in real Tk event tests, isolated from every hardware command topic.

Run with RASHKO_TEST_TK=1, ROS_DOMAIN_ID=177, ROS_LOCALHOST_ONLY=1 and DISPLAY.
Command publishers are recording fakes; no simulation/UDP backend is launched.
"""

import math
import os
import threading
import time
import tkinter as tk
import unittest

import rclpy

from arm_controller.arm_gui import ArmControlGUI
from arm_controller.joint_contract import HOME_DEGREES


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@unittest.skipUnless(os.environ.get('RASHKO_TEST_TK') == '1', 'opt-in Tk integration')
class TkControlTests(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '177')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        rclpy.init(args=[
            '--ros-args', '-p', 'camera_enabled:=false',
            '-r', '/joint_commands:=/gui_regression/joint_commands',
            '-r', '/servo_speed:=/gui_regression/servo_speed',
            '-r', '/emergency_stop:=/gui_regression/emergency_stop',
        ])
        self.root = tk.Tk()
        self.gui = ArmControlGUI(self.root)
        self.gui.joint_pub = RecordingPublisher()
        self.gui.speed_pub = RecordingPublisher()
        self.gui.estop_pub = RecordingPublisher()
        self.pump(0.15)

    def tearDown(self):
        try:
            self.gui.on_closing()
        finally:
            rclpy.shutdown()

    def pump(self, seconds, until=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.root.update()
            if until is not None and until():
                return
            time.sleep(0.005)
        if until is not None:
            self.fail('Tk operation did not settle before deadline')

    def button(self, text):
        def descendants(widget):
            for child in widget.winfo_children():
                yield child
                yield from descendants(child)
        return next(widget for widget in descendants(self.root)
                    if isinstance(widget, tk.Button) and widget.cget('text') == text)

    def test_home_button_wins_over_a_running_drag_worker(self):
        gui = self.gui
        entered, release = threading.Event(), threading.Event()
        solve = gui._solve_ik_to_point_with_seed

        def delayed(*args):
            entered.set()
            release.wait(timeout=3)
            return solve(*args)

        gui._solve_ik_to_point_with_seed = delayed
        gui.realtime_mode.set(True)
        gui.toggle_auto_follow()
        gui.on_3d_target_move(0.10, 0.0, 0.20)
        try:
            self.pump(2, until=entered.is_set)
            self.button('Home').invoke()
        finally:
            release.set()
        self.pump(3, until=lambda: not gui._drag_ik_running and bool(gui.joint_pub.messages))
        self.pump(0.2)
        self.assertFalse(gui.auto_follow_active.get())
        self.assertEqual(gui.target_positions, list(HOME_DEGREES))
        self.assertEqual([list(message.data) for message in gui.joint_pub.messages],
                         [list(HOME_DEGREES)])

    def test_native_slider_input_takes_control_but_programmatic_set_does_not(self):
        gui = self.gui
        gui.toggle_auto_follow()
        gui.joint_sliders[0].set(94)
        self.pump(0.1)
        self.assertTrue(gui.auto_follow_active.get())
        slider = gui.joint_sliders[0]
        x, y = slider.coords(94)
        slider.event_generate('<ButtonPress-1>', x=x, y=y)
        slider.event_generate('<B1-Motion>', x=x + 5, y=y)
        slider.event_generate('<ButtonRelease-1>', x=x + 5, y=y)
        self.pump(0.1)
        self.assertFalse(gui.auto_follow_active.get())
        self.assertEqual(gui.joint_pub.messages, [])  # Real-time is OFF.

    def test_native_mouse_drag_endpoint_follows_both_screen_directions(self):
        gui = self.gui
        gui.apply_preset((90.0, 45.0, 120.0, 80.0, 98.0, 0.0))
        position = gui.forward_kinematics(*gui.target_positions[:5])[:3]
        gui._set_cartesian_controls(*(value * 1000 for value in position))
        gui.toggle_auto_follow()
        self.pump(0.1)
        viz = gui.viz_3d
        for delta in (4, -4):
            with self.subTest(screen_delta=delta):
                screen = viz.project_3d_to_2d(viz.target_x, viz.target_y, viz.target_z)
                x, y = (round(value) for value in screen)
                gui.joint_pub.messages.clear()
                viz.canvas.event_generate('<ButtonPress-1>', x=x, y=y)
                viz.canvas.event_generate('<B1-Motion>', x=x + delta, y=y)
                viz.canvas.event_generate('<ButtonRelease-1>', x=x + delta, y=y)
                self.pump(3, until=lambda: bool(gui.joint_pub.messages) and not gui._drag_ik_running)
                target = (viz.target_x, viz.target_y, viz.target_z)
                achieved = gui.forward_kinematics(*gui.joint_pub.messages[-1].data[:5])[:3]
                error = math.dist(target, achieved)
                self.assertLess(error, 0.001)
                after = viz.project_3d_to_2d(*target)
                self.assertAlmostEqual(after[0] - screen[0], delta)
                self.assertAlmostEqual(after[1] - screen[1], 0)
                print(f'Tk drag {delta:+d}px endpoint error: {error * 1000:.4f} mm')

    def test_entry_focusout_cannot_restart_ik_after_home_or_joint_takeover(self):
        gui = self.gui
        gui.realtime_mode.set(True)
        solves = []
        gui.apply_cartesian_silent = lambda: solves.append(True)
        for control in ('Home', 'joint'):
            for dirty in (False, True):
                with self.subTest(control=control, dirty=dirty):
                    gui._take_manual_control()
                    gui.joint_pub.messages.clear()
                    entry = gui.cartesian_entries['X']
                    entry.focus_force()
                    self.pump(0.05)
                    if dirty:
                        entry.delete(0, tk.END)
                        entry.insert(0, '120')
                    if control == 'Home':
                        button = self.button('Home')
                        button.invoke()
                        button.focus_set()
                    else:
                        slider = gui.joint_sliders[0]
                        x, y = slider.coords(slider.get())
                        slider.event_generate('<ButtonPress-1>', x=x, y=y)
                        slider.event_generate('<ButtonRelease-1>', x=x, y=y)
                        slider.focus_set()
                    # Force delivery after takeover as well as the native
                    # focus transition, covering both possible event orders.
                    entry.event_generate('<FocusOut>')
                    self.pump(0.2)
                    self.assertFalse(gui.cartesian_update_pending)
                    self.assertEqual(solves, [])
                    if control == 'Home':
                        self.assertEqual(gui.target_positions, list(HOME_DEGREES))

    def test_realtime_checkbox_cancels_both_queued_update_paths(self):
        gui = self.gui
        def descendants(widget):
            for child in widget.winfo_children():
                yield child
                yield from descendants(child)
        checkbox = next(widget for widget in descendants(self.root)
                        if isinstance(widget, tk.Checkbutton)
                        and 'Real-time Mode' in widget.cget('text'))
        solves = []
        gui.apply_cartesian_silent = lambda: solves.append(True)
        for schedule in (gui.schedule_update, gui.schedule_cartesian_update):
            gui.realtime_mode.set(True)
            schedule()
            checkbox.invoke()
            self.pump(0.2)
            self.assertFalse(gui.realtime_mode.get())
            self.assertEqual(solves, [])
            self.assertEqual(gui.joint_pub.messages, [])


if __name__ == '__main__':
    unittest.main()
