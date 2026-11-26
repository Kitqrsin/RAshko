#!/usr/bin/env python3
"""
Camera Node - Publishes frames from a Raspberry Pi or local camera.
Publishes raw images to /camera/image_raw for downstream processing.

Supports (tried in order):
  1. Remote HTTP MJPEG stream (the normal laptop + Pi deployment)
  2. picamera2 (native libcamera, best on Pi OS)
  3. libcamera pipeline via rpicam-vid → OpenCV
  4. OpenCV VideoCapture with V4L2 device scan
  5. A host-shared JPEG file
"""

import array
import fcntl
import os
import shutil
import struct
import subprocess
import threading
import rclpy
import time
import urllib.request
from urllib.parse import urlparse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Header
import numpy as np

from arm_controller.camera_info_loader import (
    load_camera_info_yaml,
    require_matching_resolution,
)
from arm_controller.pi_endpoint import default_camera_url

# Detect if we are running inside a Docker container
IN_DOCKER = os.path.exists('/.dockerenv') or os.path.exists('/run/.containerenv')

# Try picamera2 first (native Pi Camera support), fall back to OpenCV
PICAMERA2_AVAILABLE = False
OPENCV_AVAILABLE = False

if not IN_DOCKER:
    # picamera2 only works on native Pi OS, never inside Docker
    try:
        from picamera2 import Picamera2
        import libcamera  # noqa: F401
        PICAMERA2_AVAILABLE = True
    except (ImportError, ModuleNotFoundError):
        pass

try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError:
    pass

PIL_AVAILABLE = False
try:
    from PIL import Image as PILImage
    PIL_AVAILABLE = True
except ImportError:
    pass


def _default_camera_url():
    """Return an explicit URL or derive one from the configured Pi address."""
    return default_camera_url()


class CameraNode(Node):
    """ROS2 node that obtains camera frames and publishes them."""

    def __init__(self):
        super().__init__('camera_node')

        # --- Parameters ---
        self.declare_parameter('frame_rate', 15.0)       # FPS
        self.declare_parameter('width', 640)              # wider resolution
        self.declare_parameter('height', 480)
        self.declare_parameter('device_id', -1)          # -1 = auto-scan
        self.declare_parameter('camera_backend', 'auto')
        self.declare_parameter('camera_url', _default_camera_url())
        self.declare_parameter('camera_timeout', 3.0)
        self.declare_parameter('camera_reconnect_delay', 2.0)
        self.declare_parameter('camera_max_frame_bytes', 5_000_000)
        self.declare_parameter(
            'camera_info_file',
            os.environ.get('RASHKO_CAMERA_INFO_FILE', ''),
        )
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('camera_preview', False)

        self.fps = max(float(self.get_parameter('frame_rate').value), 0.1)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.device_id = int(self.get_parameter('device_id').value)
        self.backend = str(self.get_parameter('camera_backend').value).strip().lower()
        self.camera_url = str(self.get_parameter('camera_url').value).strip()
        self._dynamic_camera_url = (
            not str(os.environ.get('RASHKO_CAMERA_URL', '')).strip()
        ) and (self.camera_url == _default_camera_url())
        self.camera_timeout = max(
            float(self.get_parameter('camera_timeout').value), 0.25
        )
        self.camera_reconnect_delay = max(
            float(self.get_parameter('camera_reconnect_delay').value), 0.1
        )
        self.camera_max_frame_bytes = max(
            int(self.get_parameter('camera_max_frame_bytes').value), 1024
        )
        self.camera_frame_id = str(
            self.get_parameter('camera_frame_id').value
        ).strip()
        if not self.camera_frame_id:
            raise ValueError('camera_frame_id cannot be empty')
        camera_info_file = str(
            self.get_parameter('camera_info_file').value
        ).strip()
        self.camera_calibration = None
        if camera_info_file:
            self.camera_calibration = load_camera_info_yaml(camera_info_file)
            self.get_logger().info(
                'Loaded camera calibration %s (%dx%d)'
                % (
                    camera_info_file,
                    self.camera_calibration['width'],
                    self.camera_calibration['height'],
                )
            )
        else:
            self.get_logger().warning(
                'No camera_info_file configured; metric cap perception will '
                'remain locked'
            )

        # --- Publishers ---
        # Video is freshness-sensitive. Keep only the newest raw frame so a
        # temporarily busy GUI cannot accumulate a multi-megabyte stale queue.
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.preview_pub = None
        if self.get_parameter('camera_preview').value:
            self.preview_pub = self.create_publisher(
                CompressedImage, '/camera/preview/compressed',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )

        # --- Camera setup ---
        self.camera = None          # picamera2 instance
        self.cv_cap = None          # OpenCV VideoCapture
        self.active_backend = None  # name of working backend
        self._shutdown_event = threading.Event()
        self._network_thread = None
        self._network_response = None
        self._network_response_lock = threading.Lock()
        self._network_data_lock = threading.Lock()
        self._network_jpeg = None
        self._network_sequence = 0
        self._network_consumed_sequence = 0
        self._network_preview_sequence = 0
        self._preview_ready = None
        self._network_state = None
        self._network_state_message = ''
        self._network_state_version = 0
        self._network_reported_version = -1
        self._init_camera()

        # --- Capture timer ---
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self.capture_and_publish)
        self.frame_count = 0
        self._frame_log_interval = max(1, int(round(self.fps * 10.0)))

        self.get_logger().info(
            f'Camera node started — {self.width}x{self.height} @ {self.fps} FPS'
            f' [backend: {self.active_backend or "NONE"}]'
        )

    # ------------------------------------------------------------------ #
    #  Camera initialisation                                               #
    # ------------------------------------------------------------------ #
    def _init_camera(self):
        """Try to initialise the camera with the best available backend."""
        backend = self.backend

        # --- 1. Remote MJPEG stream (normal laptop + Pi deployment) ---
        if backend in ('auto', 'mjpeg', 'network', 'http'):
            if self.camera_url:
                if self._try_network_mjpeg():
                    return
            elif backend != 'auto':
                self.get_logger().warn(
                    'The MJPEG backend requires the camera_url parameter or '
                    'RASHKO_CAMERA_URL'
                )

        # --- 2. picamera2 (native libcamera, host only) ---
        if backend in ('auto', 'picamera2') and PICAMERA2_AVAILABLE:
            if self._try_picamera2():
                return

        # --- 3. libcamera via rpicam-vid pipeline (Pi 5 in Docker) ---
        if backend in ('auto', 'libcamera') and OPENCV_AVAILABLE:
            if self._try_libcamera_pipe():
                return

        # --- 4. Shared JPEG file (best option for Pi 5 inside Docker) ---
        #    Try this before OpenCV when in Docker, since V4L2 raw capture
        #    doesn't work on Pi 5 without libcamera.
        if backend == 'shared-file' or (IN_DOCKER and backend == 'auto'):
            shared_file = '/workspace/.camera_frame.jpg'
            if os.path.isfile(shared_file):
                self._shared_frame_file = shared_file
                self.active_backend = 'shared-file'
                self.get_logger().info(
                    f'Using shared JPEG file backend ({shared_file}). '
                    'Host camera_stream.sh should be running.'
                )
                self._shared_last_mtime = 0.0
                return
            else:
                self.get_logger().warn(
                    f'Shared frame file {shared_file} not found. '
                    'Start host-side camera: ./camera_stream.sh start'
                )
                # Still set up to poll for it
                self._shared_frame_file = shared_file
                self.active_backend = 'shared-file-waiting'
                self._shared_last_mtime = 0.0
                return

        # --- 5. OpenCV direct V4L2 (works on Pi 4 / USB cameras) ---
        if backend in ('auto', 'opencv') and OPENCV_AVAILABLE:
            if self._try_opencv():
                return

        # --- 6. Shared JPEG file (non-Docker fallback) ---
        shared_file = '/workspace/.camera_frame.jpg'
        if os.path.isfile(shared_file):
            self._shared_frame_file = shared_file
            self.active_backend = 'shared-file'
            self.get_logger().info(
                f'Using shared JPEG file backend ({shared_file}). '
                'Run ./camera_stream.sh start on the host.'
            )
            self._shared_last_mtime = 0.0
            return

        self.get_logger().error(
            'No camera backend available!\n'
            '  Checklist:\n'
            '  1. Is RASHKO_CAMERA_URL reachable from the laptop container?\n'
            '  2. For local capture, is the camera detected by rpicam-hello?\n'
            '  3. Install OpenCV or Pillow to decode JPEG frames.\n'
            '  4. Or start host-side camera: ./camera_stream.sh start'
        )

    def _try_network_mjpeg(self):
        """Start a non-blocking worker for an HTTP MJPEG stream."""
        parsed = urlparse(self.camera_url)
        if parsed.scheme not in ('http', 'https') or not parsed.netloc:
            self.get_logger().warn(
                f'Invalid camera_url {self.camera_url!r}; expected an HTTP(S) URL'
            )
            return False
        if not OPENCV_AVAILABLE and not PIL_AVAILABLE:
            self.get_logger().warn(
                'The MJPEG backend needs OpenCV or Pillow to decode JPEG frames'
            )
            return False

        self.active_backend = 'mjpeg'
        if self.preview_pub is not None:
            self._preview_ready = self.create_guard_condition(self._publish_network_preview)
        self._set_network_state(
            'connecting', f'Connecting to Raspberry Pi camera at {self.camera_url}'
        )
        self._network_thread = threading.Thread(
            target=self._network_mjpeg_worker,
            name='rashko-mjpeg-reader',
            daemon=True,
        )
        self._network_thread.start()
        return True

    def _set_network_state(self, state, message):
        """Publish worker state to the ROS timer without logging from its thread."""
        with self._network_data_lock:
            if state == self._network_state and message == self._network_state_message:
                return
            self._network_state = state
            self._network_state_message = message
            self._network_state_version += 1

    def _report_network_state(self):
        """Log the most recent stream state once from the ROS executor thread."""
        with self._network_data_lock:
            if self._network_state_version == self._network_reported_version:
                return
            self._network_reported_version = self._network_state_version
            state = self._network_state
            message = self._network_state_message

        if state == 'connected':
            self.get_logger().info(message)
        elif state == 'connecting':
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message, throttle_duration_sec=10.0)

    def _network_mjpeg_worker(self):
        """Continuously read complete JPEGs, reconnecting after failures."""
        while not self._shutdown_event.is_set():
            self._refresh_camera_url()
            request = urllib.request.Request(
                self.camera_url, headers={'User-Agent': 'RAshko-camera-node/1.0'},
            )
            next_url_check = time.monotonic() + 0.25
            switch_stream = False
            response = None
            try:
                response = urllib.request.urlopen(
                    request, timeout=self.camera_timeout
                )
                with self._network_response_lock:
                    self._network_response = response
                self._set_network_state(
                    'connected',
                    f'Receiving Raspberry Pi camera stream from {self.camera_url}',
                )

                stream_buffer = bytearray()
                while not self._shutdown_event.is_set():
                    # Read available bytes without waiting to fill the buffer.
                    # Small JPEGs otherwise arrive in bursts several frames late.
                    chunk = response.read1(4096)
                    if not chunk:
                        raise EOFError('camera stream closed the connection')
                    now = time.monotonic()
                    if self._dynamic_camera_url and now >= next_url_check:
                        next_url_check = now + 0.25
                        if self._refresh_camera_url():
                            switch_stream = True
                            break
                    stream_buffer.extend(chunk)
                    self._extract_network_frames(stream_buffer)

            except Exception as exc:
                if not self._shutdown_event.is_set():
                    self._set_network_state(
                        'disconnected',
                        f'Camera stream unavailable ({exc}); retrying in '
                        f'{self.camera_reconnect_delay:.1f}s',
                    )
            finally:
                with self._network_response_lock:
                    if self._network_response is response:
                        self._network_response = None
                if response is not None:
                    try:
                        response.close()
                    except Exception:
                        pass

            if not switch_stream:
                self._shutdown_event.wait(self.camera_reconnect_delay)

    def _refresh_camera_url(self):
        if not self._dynamic_camera_url:
            return False
        try:
            latest_url = _default_camera_url()
        except (OSError, ValueError):
            return False
        if not latest_url or latest_url == self.camera_url:
            return False
        if urlparse(latest_url).scheme not in ('http', 'https'):
            return False
        self.camera_url = latest_url
        with self._network_data_lock:
            self._network_jpeg = None
            self._network_consumed_sequence = self._network_sequence
            self._network_preview_sequence = self._network_sequence
        self._set_network_state('connecting', f'Switching camera to {self.camera_url}')
        return True

    def _extract_network_frames(self, stream_buffer):
        """Consume complete JPEGs, retaining only the newest from this read."""
        jpeg_data = None
        while stream_buffer:
            start = stream_buffer.find(b'\xff\xd8')
            if start == -1:
                del stream_buffer[:-1]
                break
            if start:
                del stream_buffer[:start]

            end = stream_buffer.find(b'\xff\xd9', 2)
            if end == -1:
                if len(stream_buffer) > self.camera_max_frame_bytes:
                    stream_buffer.clear()
                    self._set_network_state(
                        'disconnected',
                        'Discarded an oversized or invalid camera frame',
                    )
                break

            frame_end = end + 2
            if frame_end <= self.camera_max_frame_bytes:
                jpeg_data = bytes(memoryview(stream_buffer)[:frame_end])
            del stream_buffer[:frame_end]

        if jpeg_data is not None and not self._shutdown_event.is_set():
            self._set_network_state(
                'connected',
                f'Receiving Raspberry Pi camera stream from {self.camera_url}',
            )
            with self._network_data_lock:
                self._network_jpeg = jpeg_data
                self._network_sequence += 1
            if self._preview_ready is not None:
                self._preview_ready.trigger()

    def _publish_network_preview(self):
        """Publish the latest MJPEG frame on the ROS executor as it arrives."""
        with self._network_data_lock:
            if self._network_sequence == self._network_preview_sequence:
                return
            jpeg_data = self._network_jpeg
            self._network_preview_sequence = self._network_sequence
        if self.preview_pub.get_subscription_count() > 0:
            self._publish_preview(self._frame_header(), jpeg_data)

    def _frame_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.camera_frame_id
        return header

    def _try_picamera2(self):
        """Attempt native picamera2 backend."""
        try:
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration(
                main={'size': (self.width, self.height), 'format': 'RGB888'}
            )
            self.camera.configure(config)
            self.camera.start()
            time.sleep(1.0)  # let auto-exposure settle
            self.active_backend = 'picamera2'
            self.get_logger().info('Using picamera2 backend (native libcamera)')
            return True
        except Exception as e:
            self.get_logger().warn(f'picamera2 failed: {e}')
            self.camera = None
            return False

    def _try_libcamera_pipe(self):
        """
        Use rpicam-vid (Pi 5) or libcamera-vid to stream to stdout,
        then read frames with OpenCV.  Works inside Docker with /dev mounted.
        """
        # Find the right command
        vid_cmd = None
        for cmd in ('rpicam-vid', 'libcamera-vid'):
            if shutil.which(cmd):
                vid_cmd = cmd
                break

        if vid_cmd is None:
            self.get_logger().warn(
                'libcamera pipeline: rpicam-vid / libcamera-vid not found'
            )
            return False

        # Check if any camera is available
        hello_cmd = vid_cmd.replace('-vid', '-hello')
        try:
            result = subprocess.run(
                [hello_cmd, '--list-cameras'],
                capture_output=True, text=True, timeout=5
            )
            if 'No cameras available' in result.stderr or 'No cameras available' in result.stdout:
                self.get_logger().warn('libcamera: no cameras detected by hardware')
                return False
        except Exception:
            pass  # continue trying anyway

        # rpicam-vid outputs h264 by default; request MJPEG for easier decoding.
        command = [
            vid_cmd,
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', str(int(self.fps)),
            '--codec', 'mjpeg',
            '--nopreview',
            '-t', '0',
            '-o', '-',
        ]

        try:
            self._libcam_proc = subprocess.Popen(
                command,
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                bufsize=self.width * self.height * 3
            )
            # Try to read one test frame
            time.sleep(1.0)
            if self._libcam_proc.poll() is not None:
                self.get_logger().warn(
                    f'libcamera pipeline exited immediately (code {self._libcam_proc.returncode})'
                )
                return False

            os.set_blocking(self._libcam_proc.stdout.fileno(), False)
            self.active_backend = 'libcamera-pipe'
            self.get_logger().info(f'Using libcamera pipeline backend ({vid_cmd})')
            self._mjpeg_buffer = b''
            return True
        except Exception as e:
            self.get_logger().warn(f'libcamera pipeline failed: {e}')
            return False

    @staticmethod
    def _is_v4l2_capture_device(dev_path):
        """Check if a /dev/videoN device supports V4L2 VIDEO_CAPTURE via ioctl."""
        VIDIOC_QUERYCAP = 0x80685600  # ioctl number for VIDIOC_QUERYCAP on ARM64
        V4L2_CAP_VIDEO_CAPTURE = 0x00000001
        try:
            fd = os.open(dev_path, os.O_RDWR | os.O_NONBLOCK)
            try:
                buf = bytearray(104)  # struct v4l2_capability size
                fcntl.ioctl(fd, VIDIOC_QUERYCAP, buf)
                # capabilities field is at offset 84, 4 bytes LE
                caps = struct.unpack_from('<I', buf, 84)[0]
                return bool(caps & V4L2_CAP_VIDEO_CAPTURE)
            finally:
                os.close(fd)
        except Exception:
            return False

    def _try_opencv(self):
        """Scan V4L2 devices for a working camera."""
        if self.device_id >= 0:
            candidates = [self.device_id]
        else:
            # Build list from actual /dev/videoN that support VIDEO_CAPTURE
            candidates = []
            for i in range(40):
                dev = f'/dev/video{i}'
                if os.path.exists(dev) and self._is_v4l2_capture_device(dev):
                    candidates.append(i)
            self.get_logger().info(
                'V4L2 capture devices found: '
                + (", ".join(f"/dev/video{d}" for d in candidates) or "none")
            )

        for dev_id in candidates:
            try:
                cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                if cap.isOpened():
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        self.cv_cap = cap
                        self.active_backend = f'opencv (device {dev_id})'
                        self.get_logger().info(
                            f'Using OpenCV backend (/dev/video{dev_id})'
                        )
                        return True
                cap.release()
            except Exception:
                continue

        self.get_logger().warn(
            f'OpenCV: no working camera found (scanned {len(candidates)} devices)'
        )
        return False

    # ------------------------------------------------------------------ #
    #  Frame capture + publish                                             #
    # ------------------------------------------------------------------ #
    def capture_and_publish(self):
        """Publish the latest frame in the formats subscribers need."""
        if self.active_backend == 'mjpeg':
            self._report_network_state()
        preview = self.preview_pub
        wants_preview = (
            preview is not None and self.active_backend != 'mjpeg'
            and preview.get_subscription_count() > 0
        )
        wants_raw = preview is None or self.image_pub.get_subscription_count() > 0
        wants_info = preview is None or self.info_pub.get_subscription_count() > 0
        if not (wants_preview or wants_raw or wants_info):
            return

        header = self._frame_header()
        frame = None
        jpeg_data = None

        # --- Remote HTTP MJPEG ---
        if self.active_backend == 'mjpeg':
            with self._network_data_lock:
                if self._network_sequence == self._network_consumed_sequence:
                    return
                jpeg_data = self._network_jpeg
                self._network_consumed_sequence = self._network_sequence
            frame = self._decode_jpeg(jpeg_data)

        # --- picamera2 ---
        elif self.camera is not None:
            try:
                frame = self.camera.capture_array()  # RGB numpy array
            except Exception as e:
                self.get_logger().error(f'picamera2 capture error: {e}')
                return

        # --- libcamera pipe (MJPEG) ---
        elif hasattr(self, '_libcam_proc') and self._libcam_proc is not None:
            frame = self._read_mjpeg_frame()

        # --- OpenCV V4L2 ---
        elif self.cv_cap is not None:
            ret, bgr = self.cv_cap.read()
            if not ret:
                self.get_logger().warn('OpenCV frame grab failed')
                return
            frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        # --- Shared JPEG file ---
        elif hasattr(self, '_shared_frame_file') and self._shared_frame_file:
            try:
                if not os.path.isfile(self._shared_frame_file):
                    return  # file not yet created
                mtime = os.path.getmtime(self._shared_frame_file)
                if mtime == self._shared_last_mtime:
                    return  # no new frame
                self._shared_last_mtime = mtime
                # Update backend status once the file appears
                if self.active_backend == 'shared-file-waiting':
                    self.active_backend = 'shared-file'
                    self.get_logger().info('Shared camera frame file appeared — streaming.')
                with open(self._shared_frame_file, 'rb') as f:
                    jpeg_data = f.read(self.camera_max_frame_bytes + 1)
                if len(jpeg_data) > self.camera_max_frame_bytes:
                    return
                if len(jpeg_data) < 100:
                    return
                frame = self._decode_jpeg(jpeg_data)
            except Exception as e:
                self.get_logger().warn(f'Shared file read error: {e}', throttle_duration_sec=5.0)
                return

        else:
            return  # no camera

        if frame is None:
            return

        if wants_preview:
            if jpeg_data is None and OPENCV_AVAILABLE:
                success, encoded = cv2.imencode(
                    '.jpg', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                    [cv2.IMWRITE_JPEG_QUALITY, 85],
                )
                if success:
                    jpeg_data = encoded.tobytes()
            if jpeg_data is not None:
                self._publish_preview(header, jpeg_data)

        if wants_raw:
            msg = Image()
            msg.header = header
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = 'rgb8'
            msg.is_bigendian = False
            msg.step = msg.width * 3
            # Use Humble's native uint8[] type to avoid per-element conversion.
            msg.data = array.array('B', frame.tobytes())
            self.image_pub.publish(msg)

        if not wants_info:
            return

        # --- Camera info ---
        info_msg = CameraInfo()
        info_msg.header = header
        info_msg.width = frame.shape[1]
        info_msg.height = frame.shape[0]
        if self.camera_calibration is not None:
            try:
                require_matching_resolution(
                    self.camera_calibration, frame.shape[1], frame.shape[0]
                )
            except ValueError as exc:
                self.get_logger().error(
                    f'Camera calibration rejected at runtime: {exc}',
                    throttle_duration_sec=5.0,
                )
            else:
                info_msg.distortion_model = self.camera_calibration[
                    'distortion_model'
                ]
                info_msg.d = list(self.camera_calibration['d'])
                info_msg.k = list(self.camera_calibration['k'])
                info_msg.r = list(self.camera_calibration['r'])
                info_msg.p = list(self.camera_calibration['p'])
        self.info_pub.publish(info_msg)

        self.frame_count += 1
        if self.frame_count % self._frame_log_interval == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')

    def _publish_preview(self, header, jpeg_data):
        if not jpeg_data or len(jpeg_data) > self.camera_max_frame_bytes:
            return
        msg = CompressedImage()
        msg.header = header
        msg.format = 'jpeg'
        msg.data = array.array('B', jpeg_data)
        self.preview_pub.publish(msg)

    def _decode_jpeg(self, jpeg_data):
        """Decode one JPEG byte string to an RGB numpy array."""
        if not jpeg_data:
            return None
        try:
            if OPENCV_AVAILABLE:
                decoded = cv2.imdecode(
                    np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if decoded is not None:
                    return cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)
            elif PIL_AVAILABLE:
                import io
                with PILImage.open(io.BytesIO(jpeg_data)) as pil_img:
                    return np.array(pil_img.convert('RGB'))
        except Exception as exc:
            self.get_logger().warn(
                f'JPEG decode error: {exc}', throttle_duration_sec=5.0
            )
        return None

    def _read_mjpeg_frame(self):
        """Read one JPEG frame from the rpicam-vid MJPEG stream."""
        try:
            # stdout is non-blocking so the ROS timer and shutdown stay responsive.
            while True:
                try:
                    chunk = os.read(self._libcam_proc.stdout.fileno(), 64 * 1024)
                except BlockingIOError:
                    break
                if not chunk:
                    break
                self._mjpeg_buffer += chunk

            soi = self._mjpeg_buffer.find(b'\xff\xd8')
            if soi == -1:
                self._mjpeg_buffer = self._mjpeg_buffer[-1:]
                return None
            eoi = self._mjpeg_buffer.find(b'\xff\xd9', soi + 2)
            if eoi == -1:
                if len(self._mjpeg_buffer) > self.camera_max_frame_bytes:
                    self._mjpeg_buffer = b''
                return None

            jpeg_data = self._mjpeg_buffer[soi:eoi + 2]
            self._mjpeg_buffer = self._mjpeg_buffer[eoi + 2:]
            return self._decode_jpeg(jpeg_data)
        except Exception as e:
            self.get_logger().warn(f'MJPEG read error: {e}')
            return None

    # ------------------------------------------------------------------ #
    #  Cleanup                                                             #
    # ------------------------------------------------------------------ #
    def destroy_node(self):
        self._shutdown_event.set()
        with self._network_response_lock:
            network_response = self._network_response
        if network_response is not None:
            try:
                network_response.close()
            except Exception:
                pass
        if self._network_thread is not None:
            self._network_thread.join(timeout=1.0)

        if self.camera is not None:
            try:
                self.camera.stop()
            except Exception:
                pass
        if self.cv_cap is not None:
            self.cv_cap.release()
        if hasattr(self, '_libcam_proc') and self._libcam_proc is not None:
            try:
                self._libcam_proc.terminate()
                self._libcam_proc.wait(timeout=1)
            except Exception:
                try:
                    self._libcam_proc.kill()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    if OPENCV_AVAILABLE:
        cv2.setNumThreads(2)
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
