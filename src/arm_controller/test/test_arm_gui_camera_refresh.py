import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from arm_controller.arm_gui import ArmControlGUI


class _RecordingLabel:
    def __init__(self):
        self.calls = []

    def configure(self, **kwargs):
        self.calls.append(kwargs)

    config = configure


class _Toggle:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value


class _RecordingRoot:
    def __init__(self):
        self.after_calls = []

    def after(self, delay_ms, callback):
        self.after_calls.append((delay_ms, callback))


class ArmGuiCameraRefreshTests(unittest.TestCase):
    def make_gui(self):
        return SimpleNamespace(
            _camera_lock=threading.Lock(),
            latest_camera_frame=None,
            _camera_frame_count=0,
            _camera_frame_at=0.0,
            CAMERA_STALE_S=ArmControlGUI.CAMERA_STALE_S,
            _displayed_camera_frame_count=-1,
        )

    def test_each_source_frame_is_taken_only_once(self):
        gui = self.make_gui()
        first = np.full((2, 3, 3), 7, dtype=np.uint8)

        ArmControlGUI._store_latest_camera_frame(gui, first)
        rendered, has_frame = ArmControlGUI._take_new_camera_frame(gui)
        duplicate, still_has_frame = ArmControlGUI._take_new_camera_frame(gui)

        self.assertTrue(has_frame)
        self.assertTrue(still_has_frame)
        np.testing.assert_array_equal(rendered, first)
        self.assertIsNone(duplicate)

    def test_new_frame_replaces_an_unrendered_older_frame(self):
        gui = self.make_gui()
        older = np.zeros((2, 2, 3), dtype=np.uint8)
        latest = np.full((2, 2, 3), 255, dtype=np.uint8)

        ArmControlGUI._store_latest_camera_frame(gui, older)
        ArmControlGUI._store_latest_camera_frame(gui, latest)
        rendered, has_frame = ArmControlGUI._take_new_camera_frame(gui)

        self.assertTrue(has_frame)
        self.assertEqual(gui._camera_frame_count, 2)
        np.testing.assert_array_equal(rendered, latest)

    def test_stale_camera_frame_is_never_returned(self):
        gui = self.make_gui()
        ArmControlGUI._store_latest_camera_frame(gui, np.zeros((2, 2, 3)))
        gui._camera_frame_at -= ArmControlGUI.CAMERA_STALE_S + 1
        self.assertEqual(ArmControlGUI._take_new_camera_frame(gui), (None, False))

    def test_pi_status_distinguishes_live_ack_from_a_stale_estimate(self):
        label = _RecordingLabel()
        gui = SimpleNamespace(
            _pi_status=None, _link_status_text=None, link_status_label=label,
            LINK_STALE_S=ArmControlGUI.LINK_STALE_S,
        )
        ArmControlGUI._pi_status_callback(
            gui, SimpleNamespace(data='{"code":"OK","state":["OUTPUTS_ENABLED"]}'),
        )
        ArmControlGUI._update_link_status(gui)
        self.assertIn("MG996R position estimated", label.calls[-1]["text"])
        updated, code, states = gui._pi_status
        gui._pi_status = (updated - gui.LINK_STALE_S - 1, code, states)
        ArmControlGUI._update_link_status(gui)
        self.assertIn("ACK stale", label.calls[-1]["text"])

    @patch("arm_controller.arm_gui.ImageTk.PhotoImage", return_value=object())
    def test_identical_placeholder_is_rendered_once(self, photo_image):
        label = _RecordingLabel()
        gui = SimpleNamespace(
            camera_label=label,
            camera_photo=None,
            _camera_placeholder_text=None,
        )

        first = ArmControlGUI._show_camera_placeholder(gui, "Camera disabled")
        duplicate = ArmControlGUI._show_camera_placeholder(gui, "Camera disabled")

        self.assertTrue(first)
        self.assertFalse(duplicate)
        self.assertEqual(photo_image.call_count, 1)
        self.assertEqual(len(label.calls), 1)

    @patch("arm_controller.arm_gui.PIL_AVAILABLE", True)
    @patch("arm_controller.arm_gui.ImageTk.PhotoImage")
    def test_cartesian_preview_reuses_the_image_at_throttled_rate(self, photo_image):
        camera_label = _RecordingLabel()
        status_label = _RecordingLabel()
        root = _RecordingRoot()
        frame = np.full((2, 3, 3), 42, dtype=np.uint8)
        photo_image.return_value.width.return_value = 3
        photo_image.return_value.height.return_value = 2
        gui = SimpleNamespace(
            camera_label=camera_label,
            camera_photo=None,
            camera_enabled=_Toggle(True),
            camera_status_label=status_label,
            camera_fps_label=None,
            object_detector=SimpleNamespace(process_frame=lambda value: value),
            follow_enabled=_Toggle(False),
            handshake_enabled=_Toggle(False),
            _handshake_in_progress=False,
            grab_enabled=_Toggle(False),
            _grab_in_progress=False,
            detect_enabled=_Toggle(False),
            _ros2_camera_active=True,
            _direct_cam_backend=None,
            _camera_placeholder_text=None,
            _cam_fps_count=0,
            _cam_fps_time=time.time(),
            CAMERA_DRAG_REFRESH_MS=ArmControlGUI.CAMERA_DRAG_REFRESH_MS,
            CAMERA_REFRESH_MS=ArmControlGUI.CAMERA_REFRESH_MS,
            root=root,
            _prioritize_cartesian_animation=lambda: True,
            _take_new_camera_frame=lambda: (frame, True),
            _refresh_camera_frame=lambda: None,
            get_logger=lambda: SimpleNamespace(error=lambda _message: None),
        )
        gui._camera_status = None
        gui._set_camera_status = ArmControlGUI._set_camera_status.__get__(gui)

        ArmControlGUI._refresh_camera_frame(gui)
        ArmControlGUI._refresh_camera_frame(gui)

        self.assertEqual(photo_image.call_count, 1)
        photo_image.return_value.paste.assert_called_once()
        self.assertEqual(sum("image" in call for call in camera_label.calls), 1)
        self.assertLessEqual(root.after_calls[0][0], ArmControlGUI.CAMERA_DRAG_REFRESH_MS)
        self.assertEqual(len(status_label.calls), 1)
        self.assertIn("Drag (10 FPS cap)", status_label.calls[-1]["text"])


if __name__ == "__main__":
    unittest.main()
