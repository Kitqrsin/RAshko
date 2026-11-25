import json
import threading
import time
import unittest
import urllib.error
import urllib.request

from rashko_communicator.camera_stream import (
    CameraStreamConfig,
    CameraStreamService,
    SyntheticFrameSource,
)


class CameraStreamConfigTests(unittest.TestCase):
    def test_resource_limits_are_enforced(self):
        for field, value in (
            ("width", 4000),
            ("height", 50),
            ("fps", 100),
            ("jpeg_quality", 1),
        ):
            values = {field: value}
            with self.subTest(field=field):
                with self.assertRaises(ValueError):
                    CameraStreamConfig(**values)


class SyntheticFrameSourceTests(unittest.TestCase):
    def test_source_produces_complete_jpeg_and_stops(self):
        source = SyntheticFrameSource(CameraStreamConfig(fps=30))
        source.start()
        try:
            item = source.wait_for_frame(-1, timeout=1.0)
            self.assertIsNotNone(item)
            _sequence, frame = item
            self.assertTrue(frame.startswith(b"\xff\xd8"))
            self.assertTrue(frame.endswith(b"\xff\xd9"))
            self.assertIn(b"JFIF", frame)
            self.assertGreater(len(frame), 200)
        finally:
            source.stop()
        self.assertFalse(source.running)


class CameraHTTPTests(unittest.TestCase):
    def setUp(self):
        self.config = CameraStreamConfig(
            host="127.0.0.1",
            port=0,
            width=320,
            height=240,
            fps=30,
            jpeg_quality=50,
        )
        self.source = SyntheticFrameSource(self.config)
        self.service = CameraStreamService(self.config, self.source)
        self.thread = threading.Thread(
            target=self.service.serve_forever,
            name="test-camera-http",
        )
        self.thread.start()
        deadline = time.monotonic() + 2.0
        while not self.service.serving and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(self.service.serving)
        _host, port = self.service.server_address
        self.base_url = f"http://127.0.0.1:{port}"

    def tearDown(self):
        self.service.shutdown()
        self.thread.join(timeout=3.0)
        self.assertFalse(self.thread.is_alive())

    def test_health_endpoint_describes_stream(self):
        with urllib.request.urlopen(
            self.base_url + "/healthz", timeout=2.0
        ) as response:
            self.assertEqual(response.status, 200)
            payload = json.load(response)
        self.assertEqual(payload["status"], "ok")
        self.assertEqual(payload["backend"], "synthetic")
        self.assertEqual(payload["width"], 320)
        self.assertEqual(payload["height"], 240)
        self.assertEqual(payload["fps"], 30)

    def test_mjpeg_endpoint_returns_a_jpeg_part(self):
        with urllib.request.urlopen(
            self.base_url + "/stream.mjpg", timeout=2.0
        ) as response:
            self.assertEqual(
                response.headers.get_content_type(),
                "multipart/x-mixed-replace",
            )
            data = response.read(1024)
        self.assertIn(b"--frame\r\n", data)
        self.assertIn(b"Content-Type: image/jpeg\r\n", data)
        self.assertIn(b"\xff\xd8", data)
        self.assertIn(b"\xff\xd9", data)

    def test_unknown_endpoint_is_not_found(self):
        with self.assertRaises(urllib.error.HTTPError) as raised:
            urllib.request.urlopen(self.base_url + "/missing", timeout=2.0)
        self.assertEqual(raised.exception.code, 404)


if __name__ == "__main__":
    unittest.main()
