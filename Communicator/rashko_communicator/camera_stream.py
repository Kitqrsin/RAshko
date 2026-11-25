"""Small HTTP MJPEG server for a Raspberry Pi Camera Module.

The production backend uses Picamera2's hardware JPEG encoder.  A synthetic
backend is provided for commissioning and automated tests; it is never selected
as an implicit fallback when Picamera2 fails.
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple


DEFAULT_BIND_HOST = "0.0.0.0"
DEFAULT_PORT = 8080
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 15
DEFAULT_JPEG_QUALITY = 70

MIN_WIDTH = 160
MAX_WIDTH = 1920
MIN_HEIGHT = 120
MAX_HEIGHT = 1080
MIN_FPS = 1
MAX_FPS = 30
MIN_JPEG_QUALITY = 20
MAX_JPEG_QUALITY = 95

_BOUNDARY = "frame"

# A dependency-free JPEG for dry-run mode.  Camera clients only require each
# multipart member to be a complete JPEG; synthetic mode intentionally avoids
# Pillow and OpenCV on the Pi.
_SYNTHETIC_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAoHBwgHBgoICAgLCgoLDhgQDg0NDh0V"
    "FhEYIx8lJCIfIiEmKzcvJik0KSEiMEExNDk7Pj4+JS5ESUM8SDc9Pjv/2wBDAQoL"
    "Cw4NDhwQEBw7KCIoOzs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7"
    "Ozs7Ozs7Ozs7Ozs7Ozv/wAARCAAQABADASIAAhEBAxEB/8QAFQABAQAAAAAAAAAA"
    "AAAAAAAAAAb/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/8QAFQEBAQAAAAAAAAAAAAAA"
    "AAAAAwT/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oADAMBAAIRAxEAPwCDAUhf/9k="
)


@dataclass(frozen=True)
class CameraStreamConfig:
    """Validated resource limits and network settings for the stream."""

    host: str = DEFAULT_BIND_HOST
    port: int = DEFAULT_PORT
    width: int = DEFAULT_WIDTH
    height: int = DEFAULT_HEIGHT
    fps: int = DEFAULT_FPS
    jpeg_quality: int = DEFAULT_JPEG_QUALITY

    def __post_init__(self) -> None:
        if not self.host:
            raise ValueError("bind host must not be empty")
        _require_range("port", self.port, 0, 65535)
        _require_range("width", self.width, MIN_WIDTH, MAX_WIDTH)
        _require_range("height", self.height, MIN_HEIGHT, MAX_HEIGHT)
        _require_range("fps", self.fps, MIN_FPS, MAX_FPS)
        _require_range(
            "JPEG quality",
            self.jpeg_quality,
            MIN_JPEG_QUALITY,
            MAX_JPEG_QUALITY,
        )


def _require_range(name: str, value: int, minimum: int, maximum: int) -> None:
    if not isinstance(value, int) or isinstance(value, bool):
        raise ValueError(f"{name} must be an integer")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")


class LatestFrame:
    """Thread-safe one-frame mailbox shared by encoders and HTTP clients."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._frame: Optional[bytes] = None
        self._sequence = 0
        self._running = False

    @property
    def running(self) -> bool:
        with self._condition:
            return self._running

    @property
    def ready(self) -> bool:
        with self._condition:
            return self._frame is not None

    def start(self) -> None:
        with self._condition:
            self._running = True
            self._condition.notify_all()

    def stop(self) -> None:
        with self._condition:
            self._running = False
            self._condition.notify_all()

    def publish(self, frame: bytes) -> None:
        if not frame:
            return
        with self._condition:
            if not self._running:
                return
            self._frame = bytes(frame)
            self._sequence += 1
            self._condition.notify_all()

    def wait(
        self, after_sequence: int, timeout: float = 2.0
    ) -> Optional[Tuple[int, bytes]]:
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._running and (
                self._frame is None or self._sequence == after_sequence
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(remaining)
            if (
                not self._running
                or self._sequence == after_sequence
                or self._frame is None
            ):
                return None
            return self._sequence, self._frame


class SyntheticFrameSource:
    """Low-overhead static JPEG producer used only when explicitly requested."""

    name = "synthetic"

    def __init__(self, config: CameraStreamConfig) -> None:
        self.config = config
        self.frames = LatestFrame()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def running(self) -> bool:
        return self.frames.running

    @property
    def frame_ready(self) -> bool:
        return self.frames.ready

    def start(self) -> None:
        if self.running:
            return
        self._stop_event.clear()
        self.frames.start()
        self._thread = threading.Thread(
            target=self._produce,
            name="synthetic-camera",
            daemon=True,
        )
        self._thread.start()

    def _produce(self) -> None:
        interval = 1.0 / self.config.fps
        next_frame = time.monotonic()
        while not self._stop_event.is_set():
            self.frames.publish(_SYNTHETIC_JPEG)
            next_frame += interval
            delay = max(0.0, next_frame - time.monotonic())
            if self._stop_event.wait(delay):
                break

    def wait_for_frame(
        self, after_sequence: int, timeout: float = 2.0
    ) -> Optional[Tuple[int, bytes]]:
        return self.frames.wait(after_sequence, timeout)

    def stop(self) -> None:
        self._stop_event.set()
        self.frames.stop()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None


class _Picamera2Output(io.BufferedIOBase):
    """Turns Picamera2 encoder writes into complete, shared JPEG frames."""

    def __init__(self, frames: LatestFrame) -> None:
        super().__init__()
        self._frames = frames
        self._buffer = io.BytesIO()
        self._lock = threading.Lock()

    def writable(self) -> bool:
        return True

    def write(self, data: bytes) -> int:
        with self._lock:
            if data.startswith(b"\xff\xd8"):
                previous = self._buffer.getvalue()
                if previous:
                    self._frames.publish(previous)
                self._buffer.seek(0)
                self._buffer.truncate()
            return self._buffer.write(data)


class Picamera2FrameSource:
    """Camera Module source using libcamera through Picamera2."""

    name = "picamera2"

    def __init__(self, config: CameraStreamConfig) -> None:
        self.config = config
        self.frames = LatestFrame()
        self._camera = None
        self._lock = threading.Lock()

    @property
    def running(self) -> bool:
        return self.frames.running

    @property
    def frame_ready(self) -> bool:
        return self.frames.ready

    def start(self) -> None:
        with self._lock:
            if self._camera is not None:
                return
            try:
                from picamera2 import Picamera2
                from picamera2.encoders import JpegEncoder
                from picamera2.outputs import FileOutput
            except ImportError as exc:
                raise RuntimeError(
                    "Picamera2 is unavailable; install python3-picamera2 or "
                    "explicitly select the synthetic backend"
                ) from exc

            camera = Picamera2()
            output = _Picamera2Output(self.frames)
            try:
                video_config = camera.create_video_configuration(
                    main={
                        "size": (self.config.width, self.config.height),
                        "format": "RGB888",
                    },
                    controls={"FrameRate": float(self.config.fps)},
                    buffer_count=4,
                )
                camera.configure(video_config)
                self.frames.start()
                camera.start_recording(
                    JpegEncoder(q=self.config.jpeg_quality),
                    FileOutput(output),
                )
            except Exception:
                self.frames.stop()
                camera.close()
                raise
            self._camera = camera

    def wait_for_frame(
        self, after_sequence: int, timeout: float = 2.0
    ) -> Optional[Tuple[int, bytes]]:
        return self.frames.wait(after_sequence, timeout)

    def stop(self) -> None:
        with self._lock:
            camera = self._camera
            self._camera = None
            self.frames.stop()
            if camera is None:
                return
            try:
                camera.stop_recording()
            finally:
                camera.close()


class _CameraHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    max_clients = 16

    def __init__(self, config: CameraStreamConfig, source) -> None:
        self.config = config
        self.source = source
        self.stop_event = threading.Event()
        self._client_slots = threading.BoundedSemaphore(self.max_clients)
        super().__init__((config.host, config.port), _CameraRequestHandler)

    def process_request(self, request, client_address):
        # Bound worker threads even for clients that never finish a request.
        if not self._client_slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self._client_slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self._client_slots.release()


class _CameraRequestHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    timeout = 5.0
    disable_nagle_algorithm = True

    @property
    def camera_server(self) -> _CameraHTTPServer:
        return self.server  # type: ignore[return-value]

    def do_GET(self) -> None:
        path = self.path.partition("?")[0]
        if path == "/healthz":
            self._send_health()
        elif path == "/stream.mjpg":
            self._send_stream()
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Not found")

    def _send_health(self) -> None:
        server = self.camera_server
        config = server.config
        payload = json.dumps(
            {
                "status": "ok" if server.source.running else "stopping",
                "backend": server.source.name,
                "frame_ready": server.source.frame_ready,
                "width": config.width,
                "height": config.height,
                "fps": config.fps,
                "jpeg_quality": config.jpeg_quality,
            },
            separators=(",", ":"),
        ).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _send_stream(self) -> None:
        server = self.camera_server
        self.close_connection = True
        self.send_response(HTTPStatus.OK)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={_BOUNDARY}",
        )
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()

        sequence = -1
        try:
            while not server.stop_event.is_set():
                item = server.source.wait_for_frame(sequence, timeout=1.0)
                if item is None:
                    if not server.source.running:
                        break
                    continue
                sequence, frame = item
                header = (
                    f"--{_BOUNDARY}\r\n"
                    "Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame)}\r\n"
                    "\r\n"
                ).encode("ascii")
                self.wfile.write(b"".join((header, frame, b"\r\n")))
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError):
            pass

    def log_message(self, fmt: str, *args) -> None:
        sys.stderr.write(
            "%s - - [%s] %s\n"
            % (self.client_address[0], self.log_date_time_string(), fmt % args)
        )


class CameraStreamService:
    """Owns the frame source and HTTP server lifecycle."""

    def __init__(self, config: CameraStreamConfig, source) -> None:
        self.config = config
        self.source = source
        self.httpd = _CameraHTTPServer(config, source)
        self._serving = threading.Event()
        self._close_lock = threading.Lock()
        self._closed = False

    @property
    def server_address(self) -> Tuple[str, int]:
        host, port = self.httpd.server_address[:2]
        return str(host), int(port)

    @property
    def serving(self) -> bool:
        return self._serving.is_set()

    def serve_forever(self) -> None:
        try:
            self.source.start()
            self._serving.set()
            self.httpd.serve_forever(poll_interval=0.2)
        finally:
            self._serving.clear()
            self._close()

    def shutdown(self) -> None:
        self.httpd.stop_event.set()
        self.source.stop()
        if self._serving.is_set():
            self.httpd.shutdown()

    def _close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self.httpd.stop_event.set()
            self.source.stop()
            self.httpd.server_close()


def _bounded_arg(name: str, minimum: int, maximum: int):
    def parse(value: str) -> int:
        try:
            number = int(value)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"{name} must be an integer") from exc
        if not minimum <= number <= maximum:
            raise argparse.ArgumentTypeError(
                f"{name} must be between {minimum} and {maximum}"
            )
        return number

    return parse


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Serve the Pi camera as HTTP MJPEG")
    parser.add_argument(
        "--host",
        default=os.getenv("RASHKO_CAMERA_BIND_HOST", DEFAULT_BIND_HOST),
        help="bind address (env: RASHKO_CAMERA_BIND_HOST)",
    )
    parser.add_argument(
        "--port",
        type=_bounded_arg("port", 1, 65535),
        default=os.getenv("RASHKO_CAMERA_PORT", str(DEFAULT_PORT)),
        help="HTTP port (env: RASHKO_CAMERA_PORT)",
    )
    parser.add_argument(
        "--width",
        type=_bounded_arg("width", MIN_WIDTH, MAX_WIDTH),
        default=os.getenv("RASHKO_CAMERA_WIDTH", str(DEFAULT_WIDTH)),
    )
    parser.add_argument(
        "--height",
        type=_bounded_arg("height", MIN_HEIGHT, MAX_HEIGHT),
        default=os.getenv("RASHKO_CAMERA_HEIGHT", str(DEFAULT_HEIGHT)),
    )
    parser.add_argument(
        "--fps",
        type=_bounded_arg("fps", MIN_FPS, MAX_FPS),
        default=os.getenv("RASHKO_CAMERA_FPS", str(DEFAULT_FPS)),
    )
    parser.add_argument(
        "--quality",
        dest="jpeg_quality",
        type=_bounded_arg(
            "JPEG quality", MIN_JPEG_QUALITY, MAX_JPEG_QUALITY
        ),
        default=os.getenv(
            "RASHKO_CAMERA_JPEG_QUALITY", str(DEFAULT_JPEG_QUALITY)
        ),
        help="JPEG quality (env: RASHKO_CAMERA_JPEG_QUALITY)",
    )
    parser.add_argument(
        "--backend",
        choices=("picamera2", "synthetic"),
        default=os.getenv("RASHKO_CAMERA_BACKEND", "picamera2"),
        help="capture backend (env: RASHKO_CAMERA_BACKEND)",
    )
    return parser


def main(argv=None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.backend not in ("picamera2", "synthetic"):
        parser.error("backend must be 'picamera2' or 'synthetic'")

    try:
        config = CameraStreamConfig(
            host=args.host,
            port=args.port,
            width=args.width,
            height=args.height,
            fps=args.fps,
            jpeg_quality=args.jpeg_quality,
        )
    except ValueError as exc:
        parser.error(str(exc))

    source = (
        Picamera2FrameSource(config)
        if args.backend == "picamera2"
        else SyntheticFrameSource(config)
    )
    service = CameraStreamService(config, source)

    def request_shutdown(_signum, _frame) -> None:
        # BaseServer.shutdown must run outside the serve_forever thread.
        threading.Thread(
            target=service.shutdown,
            name="camera-shutdown",
            daemon=True,
        ).start()

    previous_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.signal(signum, request_shutdown)

    host, port = service.server_address
    print(
        f"Starting camera stream on http://{host}:{port}/stream.mjpg "
        f"({args.backend}, {config.width}x{config.height}@{config.fps})",
        flush=True,
    )
    try:
        service.serve_forever()
    except KeyboardInterrupt:
        # CameraStreamService has already closed the source and socket in its
        # finally block.  Calling BaseServer.shutdown here would deadlock
        # because this is the serve_forever thread.
        pass
    except Exception as exc:
        print(f"camera stream failed: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
