"""ROS wrapper for fail-closed gripper-camera cap inspection.

This first-stage node performs perception and visualization only.  It never
publishes a joint command, resets E-stop, or executes a MoveIt trajectory.
"""

from __future__ import annotations

import array
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from arm_controller.cap_perception import (
    CapPerception,
    ObservationStabilizer,
    SlotOccupancy,
    image_content_digest,
)
from arm_controller.cap_motion_safety import motion_geometry_blockers
from arm_controller.cap_workcell_config import (
    hardware_execution_blockers,
    load_inspection_settings,
)
from arm_controller.joint_contract import JOINT_NAMES, servo_degrees_to_radians
from arm_controller.red_cap_visual_servo import (
    VisualServoError,
    derive_fixed_cap_row_candidate,
    derive_fixed_jig_reference,
    detect_red_blobs,
)


class CapInspectionNode(Node):
    """Inspect the marked battery plane after the arm becomes stationary."""

    def __init__(self) -> None:
        super().__init__("cap_inspection")
        default_config = os.environ.get(
            "RASHKO_CAP_WORKCELL_CONFIG",
            "/workspace/config/cap_workcell.example.yaml",
        )
        config_file = str(
            self.declare_parameter("config_file", default_config).value
        ).strip()
        if not config_file:
            raise ValueError("config_file cannot be empty")
        self.settings = load_inspection_settings(config_file)
        self.perception = CapPerception(
            self.settings.layout,
            self.settings.inspector,
            self.settings.camera_pose_reference,
        )
        self.fixed_plane_points = None
        self.fixed_image_points = None
        if self.settings.fixed_jig_reference is not None:
            x_min, y_min, x_max, y_max = (
                self.settings.layout.workspace_bounds_m
            )
            plane_points = np.asarray(
                (
                    (x_min, y_min),
                    (x_max, y_min),
                    (x_max, y_max),
                    (x_min, y_max),
                )
                + tuple(
                    (slot.x_m, slot.y_m)
                    for slot in self.settings.layout.slots
                ),
                dtype=np.float64,
            )
            self.fixed_plane_points = plane_points
            self.fixed_image_points = cv2.perspectiveTransform(
                plane_points.reshape(-1, 1, 2),
                self.settings.fixed_jig_reference.matrix,
            ).reshape(-1, 2)
        self.stabilizer = ObservationStabilizer(
            self.settings.layout, self.settings.inspector
        )
        self.expected_present_ids = set(self.settings.expected_present_ids)
        self.observation_pose_radians = (
            None
            if self.settings.observation_joint_positions_deg is None
            else np.asarray(
                servo_degrees_to_radians(
                    self.settings.observation_joint_positions_deg
                ),
                dtype=np.float64,
            )
        )
        # Perception is deliberately not an execution authority.  Even a fully
        # commissioned configuration remains locked until a separately audited
        # planner/executor with a runtime operator arm is implemented.
        self.execution_blockers = tuple(
            dict.fromkeys(
                hardware_execution_blockers(self.settings)
                + motion_geometry_blockers(self.settings)
                + (
                    "autonomous hardware execution is disabled in this "
                    "perception-only prototype",
                )
            )
        )

        self.camera_matrix = None
        self.distortion = None
        self.camera_info_size = None
        self.camera_info_frame_id = ""
        self.camera_distortion_model = ""
        self.camera_info_received_at = 0.0
        self.undistort_map_x = None
        self.undistort_map_y = None
        self.undistorted_valid_mask = None
        self.frame_sequence = 0
        self.last_image_received_at = 0.0
        self.last_image_stamp_ns = -1
        self.last_image_content_digest = None
        self.camera_timeout_reported = False
        self.last_optional_camera_info_error = None
        self.last_joint_positions = None
        self.stationary_anchor_positions = None
        self.last_joint_state_at = 0.0
        self.last_joint_motion_at = time.monotonic()
        self.stability_started_at = None
        self.last_status_reason = None

        self.status_pub = self.create_publisher(
            String, "/cap_workcell/status", 10
        )
        self.pose_pub = self.create_publisher(
            PoseArray, "/cap_workcell/cap_poses", 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/cap_workcell/markers", 10
        )
        self.debug_image_pub = self.create_publisher(
            Image, "/cap_workcell/debug_image", 1
        )
        self.create_subscription(
            CameraInfo,
            self.settings.camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.settings.camera_image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self.create_service(
            Trigger, "/cap_workcell/reset_observation", self._reset_callback
        )
        self.create_timer(0.1, self._camera_watchdog)

        self.get_logger().warning(
            "Cap inspection is PERCEPTION ONLY; it cannot move the arm"
        )
        self.get_logger().info(
            f"Loaded {config_file}; SHA-256={self.settings.config_sha256}"
        )
        if self.execution_blockers:
            self.get_logger().warning(
                "Hardware remains locked: "
                + "; ".join(self.execution_blockers)
            )

    def _camera_info_callback(self, message: CameraInfo) -> None:
        try:
            if len(message.k) != 9 or len(message.d) == 0:
                raise ValueError("CameraInfo has no intrinsic calibration")
            matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(message.d, dtype=np.float64)
            if not np.all(np.isfinite(matrix)) or not np.all(
                np.isfinite(distortion)
            ):
                raise ValueError("CameraInfo contains non-finite calibration")
            if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
                raise ValueError("CameraInfo focal lengths are not positive")
            if message.width <= 0 or message.height <= 0:
                raise ValueError("CameraInfo has invalid image dimensions")
            if not message.header.frame_id:
                raise ValueError("CameraInfo frame_id is empty")
            expected_coefficients = {
                "plumb_bob": {5},
                "rational_polynomial": {8},
            }
            allowed_lengths = expected_coefficients.get(
                message.distortion_model
            )
            if (
                allowed_lengths is None
                or len(message.d) not in allowed_lengths
            ):
                raise ValueError(
                    "CameraInfo distortion model/coefficient count is "
                    "unsupported"
                )
            if (
                message.binning_x not in (0, 1)
                or message.binning_y not in (0, 1)
            ):
                raise ValueError("binned CameraInfo is unsupported")
            if any(
                (
                    message.roi.x_offset,
                    message.roi.y_offset,
                    message.roi.height,
                    message.roi.width,
                )
            ) or message.roi.do_rectify:
                raise ValueError("cropped CameraInfo ROI is unsupported")
        except ValueError as exc:
            self.camera_matrix = None
            self.distortion = None
            self.camera_info_size = None
            self.camera_info_frame_id = ""
            self.camera_distortion_model = ""
            self.undistort_map_x = None
            self.undistort_map_y = None
            self.undistorted_valid_mask = None
            if not self.settings.require_calibrated_camera_info:
                reason = str(exc)
                if reason != self.last_optional_camera_info_error:
                    self.last_optional_camera_info_error = reason
                    self.get_logger().warning(
                        "CameraInfo is unavailable; using raw pixels for the "
                        "perception-only cap preview: " + reason
                    )
                return
            self._reset_observation()
            self._publish_locked_status(str(exc))
            return

        changed = (
            self.camera_info_size != (message.width, message.height)
            or self.camera_info_frame_id != message.header.frame_id
            or self.camera_distortion_model != message.distortion_model
            or self.camera_matrix is None
            or self.undistort_map_x is None
            or self.undistort_map_y is None
            or self.undistorted_valid_mask is None
            or not np.array_equal(self.camera_matrix, matrix)
            or not np.array_equal(self.distortion, distortion)
        )
        if changed:
            try:
                map_x, map_y = cv2.initUndistortRectifyMap(
                    matrix,
                    distortion,
                    None,
                    matrix,
                    (message.width, message.height),
                    cv2.CV_32FC1,
                )
                source_valid = np.full(
                    (message.height, message.width), 255, dtype=np.uint8
                )
                undistorted_valid = cv2.remap(
                    source_valid,
                    map_x,
                    map_y,
                    cv2.INTER_NEAREST,
                    borderMode=cv2.BORDER_CONSTANT,
                    borderValue=0,
                )
            except cv2.error as exc:
                self.camera_matrix = None
                self.distortion = None
                self.camera_info_size = None
                self.camera_info_frame_id = ""
                self.camera_distortion_model = ""
                self.undistort_map_x = None
                self.undistort_map_y = None
                self.undistorted_valid_mask = None
                self._reset_observation()
                self._publish_locked_status(
                    f"CameraInfo rectification-map creation failed: {exc}"
                )
                return
            self.undistort_map_x = map_x
            self.undistort_map_y = map_y
            self.undistorted_valid_mask = undistorted_valid
        self.camera_matrix = matrix
        self.distortion = distortion
        self.camera_info_size = (message.width, message.height)
        self.camera_info_frame_id = message.header.frame_id
        self.camera_distortion_model = message.distortion_model
        self.camera_info_received_at = time.monotonic()
        self.last_optional_camera_info_error = None
        if changed:
            self._reset_observation()
            self.get_logger().info(
                "Accepted calibrated CameraInfo "
                f"{message.width}x{message.height}"
            )

    def _joint_state_callback(self, message: JointState) -> None:
        if tuple(message.name) != tuple(JOINT_NAMES) or len(
            message.position
        ) != len(JOINT_NAMES):
            self._invalidate_joint_feedback()
            self._publish_locked_status(
                "waiting for complete canonical six-joint feedback"
            )
            return
        values = np.asarray(message.position, dtype=np.float64)
        if not np.all(np.isfinite(values)):
            self._invalidate_joint_feedback()
            self._publish_locked_status("joint feedback is non-finite")
            return
        age = self._header_age_seconds(message)
        if age < -0.05 or age > 0.5:
            self._invalidate_joint_feedback()
            self._publish_locked_status(
                "joint feedback timestamp is stale or in the future"
            )
            return
        now = time.monotonic()
        threshold = math.radians(self.settings.joint_motion_threshold_deg)
        if self.stationary_anchor_positions is None or float(
            np.max(np.abs(values - self.stationary_anchor_positions))
        ) > threshold:
            self.last_joint_motion_at = now
            self.stationary_anchor_positions = values.copy()
            self._reset_observation()
        self.last_joint_positions = values
        self.last_joint_state_at = now

    def _invalidate_joint_feedback(self) -> None:
        self.last_joint_positions = None
        self.stationary_anchor_positions = None
        self.last_joint_state_at = 0.0
        self.last_joint_motion_at = time.monotonic()
        self._reset_observation()

    def _reset_callback(self, _request, response):
        self._reset_observation()
        response.success = True
        response.message = (
            "observation history cleared; reacquiring stable frames"
        )
        return response

    def _image_callback(self, message: Image) -> None:
        self.frame_sequence += 1
        now = time.monotonic()
        self.last_image_received_at = now
        self.camera_timeout_reported = False
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp_ns <= 0 or stamp_ns <= self.last_image_stamp_ns:
            self._reset_observation()
            self._publish_locked_status(
                "camera image timestamp is zero, stale, or repeated"
            )
            return
        self.last_image_stamp_ns = stamp_ns
        try:
            image_bgr = self._decode_image(message)
        except ValueError as exc:
            self._reset_observation()
            self._publish_locked_status(str(exc))
            return
        content_digest = image_content_digest(image_bgr)
        if content_digest == self.last_image_content_digest:
            self._reset_observation()
            self._publish_locked_status(
                "camera delivered exactly repeated decoded image content"
            )
            return
        self.last_image_content_digest = content_digest

        image_age = self._message_age_seconds(message)
        if image_age < -0.05:
            self._reset_observation()
            self._publish_locked_status(
                "camera image timestamp is in the future"
            )
            return
        if image_age > self.settings.maximum_image_age_s:
            self._reset_observation()
            self._publish_locked_status(
                f"camera image is stale ({image_age:.3f}s)"
            )
            return
        camera_gate_reason = self._camera_gate_reason(message, now)
        camera_calibrated = camera_gate_reason is None
        source_valid_mask = None
        if camera_calibrated:
            try:
                image_bgr = cv2.remap(
                    image_bgr,
                    self.undistort_map_x,
                    self.undistort_map_y,
                    cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                    borderValue=(0, 0, 0),
                )
                source_valid_mask = self.undistorted_valid_mask
            except cv2.error as exc:
                camera_calibrated = False
                camera_gate_reason = f"camera undistortion failed: {exc}"
        if (
            not camera_calibrated
            and self.settings.require_calibrated_camera_info
        ):
            self._reset_observation()
            self._publish_locked_status(str(camera_gate_reason))
            return

        image_points = None
        plane_points = None
        perception_camera_matrix = self.camera_matrix
        if self.settings.localization_mode == "fixed_cap_row":
            self._publish_fixed_cap_row_candidate(
                image_bgr,
                message,
                camera_calibrated=camera_calibrated,
            )
            return
        if self.settings.localization_mode == "fixed_red_jig":
            perception_camera_matrix = None
            reference = self.settings.fixed_jig_reference
            if reference is None:
                self._publish_fixed_jig_teaching_candidate(image_bgr, message)
                return
            if reference.image_size != (message.width, message.height):
                self._reset_observation()
                self._publish_locked_status(
                    "fixed-jig reference resolution does not match the camera"
                )
                return
            image_points = self.fixed_image_points
            plane_points = self.fixed_plane_points

        observation = self.perception.inspect_bgr(
            image_bgr,
            frame_sequence=self.frame_sequence,
            expected_present_ids=self.expected_present_ids,
            image_points_px=image_points,
            plane_points_m=plane_points,
            source_valid_mask=source_valid_mask,
            camera_matrix=perception_camera_matrix,
            include_debug_images=True,
        )
        self._publish_debug_image(observation, message)

        # Perception can be expensive.  Fresh feedback at callback entry is
        # not sufficient evidence at callback exit, so re-run every live gate
        # before this frame is admitted to temporal stability.
        finished_at = time.monotonic()
        arm_gate_reason = self._arm_gate_reason(finished_at)
        if arm_gate_reason:
            self._reset_observation()
            self._publish_diagnostic_locked_status(
                observation,
                (
                    observation.reason
                    if not observation.fixture_valid
                    else "camera pose diagnostic only; " + arm_gate_reason
                ),
            )
            return
        camera_after_reason = self._camera_gate_reason(message, finished_at)
        if camera_after_reason is not None:
            self._reset_observation()
            if (
                camera_calibrated
                or self.settings.require_calibrated_camera_info
            ):
                self._publish_locked_status(
                    "CameraInfo gate changed while perception was running: "
                    + camera_after_reason
                )
                return
        age_after_processing = self._message_age_seconds(message)
        if (
            age_after_processing < -0.05
            or age_after_processing > self.settings.maximum_image_age_s
        ):
            self._reset_observation()
            self._publish_locked_status(
                "camera image expired while perception was running"
            )
            return

        # Optional calibration mode exists only to inspect the debug image.  A
        # frame that was not rectified with matching fresh CameraInfo can never
        # enter the stabilizer or publish cap poses/fixture markers.
        if not camera_calibrated:
            self._reset_observation()
            self._publish_locked_status(
                "uncalibrated debug image only; vision stability is locked: "
                + str(camera_gate_reason)
            )
            return

        if observation.ready_for_stability:
            if self.stability_started_at is None:
                self.stability_started_at = finished_at
        else:
            self.stability_started_at = None
        stable = self.stabilizer.add(observation)
        if not stable.safe_to_move and not stable.frame_sequences:
            self.stability_started_at = None
        self._publish_markers(observation)

        elapsed_stability = (
            0.0
            if self.stability_started_at is None
            else finished_at - self.stability_started_at
        )
        vision_stable = (
            camera_calibrated
            and stable.safe_to_move
            and elapsed_stability
            >= self.settings.minimum_stability_duration_s
        )
        hardware_authorized = False
        reason = stable.reason
        if stable.safe_to_move and not vision_stable:
            reason = (
                "frame count is stable; waiting for minimum observation "
                f"duration ({elapsed_stability:.2f}/"
                f"{self.settings.minimum_stability_duration_s:.2f}s)"
            )
        if vision_stable and self.execution_blockers:
            reason = "vision stable; hardware locked: " + "; ".join(
                self.execution_blockers
            )
        status = {
            "frame_sequence": self.frame_sequence,
            "observation_stamp_ros_ns": stamp_ns,
            "vision_stable": vision_stable,
            "camera_calibrated": camera_calibrated,
            "hardware_motion_authorized": hardware_authorized,
            "reason": reason,
            "fixture_valid": observation.fixture_valid,
            "classification_valid": observation.classification_valid,
            "expected_present_ids": sorted(self.expected_present_ids),
            "observed_present_ids": list(observation.present_ids),
            "slot_observations": [
                {
                    "id": slot.slot_id,
                    "occupancy": slot.occupancy.value,
                    "confidence": round(slot.confidence, 4),
                    "centroid_m": (
                        None
                        if slot.centroid_m is None
                        else [
                            round(float(slot.centroid_m[0]), 6),
                            round(float(slot.centroid_m[1]), 6),
                        ]
                    ),
                }
                for slot in stable.slots
            ],
            "reprojection_rms_px": (
                None
                if not math.isfinite(stable.worst_reprojection_rms_px)
                else round(stable.worst_reprojection_rms_px, 4)
            ),
            "minimum_slot_confidence": round(
                stable.minimum_slot_confidence, 4
            ),
            "camera_fixture_pose": self._fixture_pose_payload(
                stable.fixture_pose
            ),
            "config_sha256": self.settings.config_sha256,
            "execution_blockers": list(self.execution_blockers),
        }
        self._publish_status(status)
        if vision_stable:
            self._publish_pose_array(stable, message)
        else:
            self._publish_empty_pose_array()

    @staticmethod
    def _fixture_pose_payload(pose) -> dict | None:
        """Serialize metric pose without losing commissioning detail."""

        if pose is None:
            return None
        return {
            "position_fixture_m": [
                round(float(value), 7) for value in pose.position_fixture_m
            ],
            "orientation_fixture_from_camera_xyzw": [
                round(float(value), 8)
                for value in pose.orientation_fixture_from_camera_xyzw
            ],
            "camera_optical_axis_fixture": [
                round(float(value), 8)
                for value in pose.camera_optical_axis_fixture
            ],
            "optical_axis_tilt_deg": round(pose.optical_axis_tilt_deg, 4),
            "pnp_reprojection_rms_px": round(
                pose.reprojection_rms_px, 4
            ),
            "minimum_depth_m": round(pose.minimum_depth_m, 6),
            "reference_translation_error_m": (
                None
                if pose.reference_translation_error_m is None
                else round(pose.reference_translation_error_m, 7)
            ),
            "reference_rotation_error_deg": (
                None
                if pose.reference_rotation_error_deg is None
                else round(pose.reference_rotation_error_deg, 4)
            ),
        }

    def _camera_gate_reason(self, message: Image, now: float) -> str | None:
        if (
            self.camera_matrix is None
            or self.camera_info_size is None
            or self.undistort_map_x is None
            or self.undistort_map_y is None
            or self.undistorted_valid_mask is None
        ):
            return "waiting for calibrated CameraInfo"
        if self.camera_info_size != (message.width, message.height):
            return "CameraInfo resolution does not match the image"
        if self.camera_info_frame_id != message.header.frame_id:
            return "CameraInfo and image frame IDs do not match"
        if now - self.camera_info_received_at > 1.0:
            return "CameraInfo stream is stale"
        return None

    def _arm_gate_reason(self, now: float) -> str | None:
        if self.observation_pose_radians is None:
            return "stationary camera observation pose is not measured"
        if (
            self.last_joint_positions is None
            or now - self.last_joint_state_at > 0.5
        ):
            return "arm needs fresh complete joint feedback"
        if (
            now - self.last_joint_motion_at
            < self.settings.joint_settle_time_s
        ):
            return "arm is still settling at the observation pose"
        pose_error = math.degrees(
            float(
                np.max(
                    np.abs(
                        self.last_joint_positions
                        - self.observation_pose_radians
                    )
                )
            )
        )
        if pose_error > self.settings.observation_pose_tolerance_deg:
            return (
                f"arm is {pose_error:.3f} degrees away from the measured "
                "observation pose"
            )
        return None

    def _reset_observation(self) -> None:
        self.stabilizer.reset()
        self.stability_started_at = None

    def _camera_watchdog(self) -> None:
        if self.last_image_received_at <= 0.0 or self.camera_timeout_reported:
            return
        age = time.monotonic() - self.last_image_received_at
        if age > self.settings.maximum_image_age_s:
            self.camera_timeout_reported = True
            self._reset_observation()
            self._publish_locked_status(
                f"camera stream stopped ({age:.3f}s without an image)"
            )

    def _header_age_seconds(self, message) -> float:
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) / 1e9
        )
        if stamp <= 0.0:
            return math.inf
        return self.get_clock().now().nanoseconds / 1e9 - stamp

    def _message_age_seconds(self, message: Image) -> float:
        return self._header_age_seconds(message)

    @staticmethod
    def _decode_image(message: Image) -> np.ndarray:
        if message.encoding not in ("rgb8", "bgr8"):
            raise ValueError(
                f"unsupported camera encoding {message.encoding!r}; "
                "need rgb8/bgr8"
            )
        row_bytes = int(message.width) * 3
        if (
            message.height <= 0
            or message.width <= 0
            or message.step < row_bytes
        ):
            raise ValueError("camera image dimensions/step are invalid")
        raw = np.frombuffer(message.data, dtype=np.uint8)
        required = int(message.height) * int(message.step)
        if raw.size < required:
            raise ValueError("camera image data is truncated")
        rows = raw[:required].reshape(int(message.height), int(message.step))
        image = rows[:, :row_bytes].reshape(
            int(message.height), int(message.width), 3
        )
        if message.encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image.copy()

    def _publish_locked_status(self, reason: str) -> None:
        self._clear_observation_outputs()
        self._publish_status(
            {
                "frame_sequence": self.frame_sequence,
                "vision_stable": False,
                "camera_calibrated": False,
                "hardware_motion_authorized": False,
                "reason": reason,
                "config_sha256": self.settings.config_sha256,
                "execution_blockers": list(self.execution_blockers),
            }
        )

    def _publish_fixed_jig_teaching_candidate(
        self, image_bgr: np.ndarray, source: Image
    ) -> None:
        """Publish a copy-ready homography without authorizing motion."""

        self._reset_observation()
        try:
            blobs = detect_red_blobs(image_bgr)
            candidate = derive_fixed_jig_reference(
                {
                    slot.slot_id: (slot.x_m, slot.y_m)
                    for slot in self.settings.layout.slots
                },
                blobs,
                (source.width, source.height),
                require_fixture_x_right_y_up=True,
            )
        except VisualServoError as exc:
            self._publish_status(
                {
                    "frame_sequence": self.frame_sequence,
                    "vision_stable": False,
                    "camera_calibrated": True,
                    "hardware_motion_authorized": False,
                    "localization_mode": "fixed_red_jig",
                    "red_blob_count": (
                        len(blobs) if "blobs" in locals() else 0
                    ),
                    "reason": "fixed-jig teaching locked: " + str(exc),
                    "config_sha256": self.settings.config_sha256,
                    "execution_blockers": list(self.execution_blockers),
                }
            )
            return
        self._publish_status(
            {
                "frame_sequence": self.frame_sequence,
                "vision_stable": False,
                "camera_calibrated": True,
                "hardware_motion_authorized": False,
                "localization_mode": "fixed_red_jig",
                "red_blob_count": len(blobs),
                "reason": (
                    "fixed-jig candidate found; copy the homography into the "
                    "YAML and keep verified=false until repeatability is "
                    "checked"
                ),
                "fixed_jig_candidate": {
                    "plane_to_image_homography": [
                        round(value, 10)
                        for value in candidate.reference.plane_to_image
                    ],
                    "image_size_px": list(candidate.reference.image_size),
                    "reprojection_rms_px": round(
                        candidate.reprojection_rms_px, 4
                    ),
                    "second_best_rms_px": round(
                        candidate.second_best_rms_px, 4
                    ),
                    "slot_pixels": {
                        slot_id: [round(point[0], 3), round(point[1], 3)]
                        for slot_id, point in candidate.slot_pixels
                    },
                },
                "config_sha256": self.settings.config_sha256,
                "execution_blockers": list(self.execution_blockers),
            }
        )

    def _publish_fixed_cap_row_candidate(
        self,
        image_bgr: np.ndarray,
        source: Image,
        *,
        camera_calibrated: bool,
    ) -> None:
        """Preview/order the row without claiming a metric 2-D pose."""

        self._reset_observation()
        try:
            blobs = detect_red_blobs(image_bgr)
            self._publish_red_cap_debug(image_bgr, blobs, source)
            if len(blobs) != len(self.settings.layout.slots):
                self._publish_status(
                    {
                        "frame_sequence": self.frame_sequence,
                        "vision_stable": False,
                        "camera_calibrated": camera_calibrated,
                        "hardware_motion_authorized": False,
                        "localization_mode": "fixed_cap_row",
                        "red_blob_count": len(blobs),
                        "reason": (
                            f"detected {len(blobs)} red cap candidate(s) in "
                            "the current view; local pixel-centering preview "
                            "is available, but all seven must be visible to "
                            "validate whole-row ordering"
                        ),
                        "red_blob_centers_px": [
                            [
                                round(blob.center_px[0], 3),
                                round(blob.center_px[1], 3),
                            ]
                            for blob in blobs
                        ],
                        "config_sha256": self.settings.config_sha256,
                        "execution_blockers": list(self.execution_blockers),
                    }
                )
                return
            candidate = derive_fixed_cap_row_candidate(
                {
                    slot.slot_id: (slot.x_m, slot.y_m)
                    for slot in self.settings.layout.slots
                },
                blobs,
                (source.width, source.height),
            )
        except VisualServoError as exc:
            self._publish_status(
                {
                    "frame_sequence": self.frame_sequence,
                    "vision_stable": False,
                    "camera_calibrated": camera_calibrated,
                    "hardware_motion_authorized": False,
                    "localization_mode": "fixed_cap_row",
                    "red_blob_count": (
                        len(blobs) if "blobs" in locals() else 0
                    ),
                    "reason": "fixed cap-row check locked: " + str(exc),
                    "config_sha256": self.settings.config_sha256,
                    "execution_blockers": list(self.execution_blockers),
                }
            )
            return
        self._publish_status(
            {
                "frame_sequence": self.frame_sequence,
                "vision_stable": False,
                "camera_calibrated": camera_calibrated,
                "hardware_motion_authorized": False,
                "localization_mode": "fixed_cap_row",
                "red_blob_count": len(blobs),
                "reason": (
                    "seven-cap row ordered; this validates row repeatability "
                    "only, not the battery's full 2D pose"
                ),
                "fixed_cap_row_candidate": {
                    "image_size_px": list(candidate.image_size),
                    "row_direction_px": [
                        round(value, 8)
                        for value in candidate.row_direction_px
                    ],
                    "row_straightness_rms_px": round(
                        candidate.row_straightness_rms_px, 4
                    ),
                    "minimum_neighbor_spacing_px": round(
                        candidate.minimum_neighbor_spacing_px, 3
                    ),
                    "slot_pixels": {
                        slot_id: [round(point[0], 3), round(point[1], 3)]
                        for slot_id, point in candidate.slot_pixels
                    },
                },
                "config_sha256": self.settings.config_sha256,
                "execution_blockers": list(self.execution_blockers),
            }
        )

    def _publish_red_cap_debug(
        self,
        image_bgr: np.ndarray,
        blobs,
        source: Image,
    ) -> None:
        """Publish color pixel overlay; never a motion authorization."""

        debug = image_bgr.copy()
        target = (debug.shape[1] // 2, debug.shape[0] // 2)
        cv2.drawMarker(
            debug,
            target,
            (255, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=2,
        )
        ordered = sorted(blobs, key=lambda blob: blob.center_px[0])
        for index, blob in enumerate(ordered):
            center = tuple(int(round(value)) for value in blob.center_px)
            radius = max(3, int(round(blob.equivalent_radius_px)))
            cv2.circle(debug, center, radius, (0, 255, 0), 2)
            cv2.putText(
                debug,
                str(index),
                (center[0] + 5, center[1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            debug,
            f"RED CAPS: {len(ordered)}/7  PIXEL PREVIEW ONLY",
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        self._publish_debug_bgr(debug, source)

    def _publish_diagnostic_locked_status(
        self, observation, reason: str
    ) -> None:
        """Expose camera geometry while keeping every motion lease invalid."""

        self._publish_empty_pose_array()
        self._publish_markers(observation)
        self._publish_status(
            {
                "frame_sequence": self.frame_sequence,
                "vision_stable": False,
                "camera_calibrated": True,
                "hardware_motion_authorized": False,
                "reason": reason,
                "fixture_valid": observation.fixture_valid,
                "classification_valid": observation.classification_valid,
                "observed_present_ids": list(observation.present_ids),
                "camera_fixture_pose": self._fixture_pose_payload(
                    observation.fixture_pose
                ),
                "config_sha256": self.settings.config_sha256,
                "execution_blockers": list(self.execution_blockers),
            }
        )

    def _publish_status(self, payload: dict) -> None:
        now_ns = self.get_clock().now().nanoseconds
        payload["status_published_ros_ns"] = now_ns
        if payload.get("vision_stable"):
            observed_ns = int(payload.get("observation_stamp_ros_ns", now_ns))
            payload["valid_until_ros_ns"] = observed_ns + int(
                self.settings.maximum_image_age_s * 1e9
            )
        else:
            payload["valid_until_ros_ns"] = now_ns
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(message)
        reason = payload.get("reason")
        if reason != self.last_status_reason:
            self.last_status_reason = reason
            if payload.get("hardware_motion_authorized"):
                self.get_logger().info(str(reason))
            else:
                self.get_logger().warning(str(reason))

    def _publish_empty_pose_array(self) -> None:
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.settings.fixture_frame_id
        self.pose_pub.publish(message)

    def _clear_observation_outputs(self) -> None:
        self._publish_empty_pose_array()
        markers = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)
        self.marker_pub.publish(markers)

    def _publish_debug_image(self, observation, source: Image) -> None:
        debug = observation.annotated_rectified_bgr
        if debug is None:
            return
        self._publish_debug_bgr(debug, source)

    def _publish_debug_bgr(self, debug: np.ndarray, source: Image) -> None:
        message = Image()
        message.header = source.header
        message.height, message.width = debug.shape[:2]
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = array.array("B", np.ascontiguousarray(debug).tobytes())
        self.debug_image_pub.publish(message)

    def _publish_pose_array(self, stable, source: Image) -> None:
        message = PoseArray()
        message.header.stamp = source.header.stamp
        message.header.frame_id = self.settings.fixture_frame_id
        for slot in stable.slots:
            if slot.occupancy is not SlotOccupancy.PRESENT:
                continue
            pose = Pose()
            pose.position.x = slot.centroid_m[0]
            pose.position.y = slot.centroid_m[1]
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            message.poses.append(pose)
        self.pose_pub.publish(message)

    def _publish_markers(self, observation) -> None:
        marker_array = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        marker_array.markers.append(delete)

        x_min, y_min, x_max, y_max = self.settings.layout.workspace_bounds_m
        boundary = Marker()
        boundary.header.frame_id = self.settings.fixture_frame_id
        boundary.header.stamp = self.get_clock().now().to_msg()
        boundary.ns = "cap_fixture"
        boundary.id = 0
        boundary.type = Marker.LINE_STRIP
        boundary.action = Marker.ADD
        boundary.scale.x = 0.0015
        boundary.color.r = 0.2
        boundary.color.g = 0.7
        boundary.color.b = 1.0
        boundary.color.a = 1.0
        boundary.pose.orientation.w = 1.0
        for x_value, y_value in (
            (x_min, y_min),
            (x_max, y_min),
            (x_max, y_max),
            (x_min, y_max),
            (x_min, y_min),
        ):
            boundary.points.append(Point(x=x_value, y=y_value, z=0.0))
        marker_array.markers.append(boundary)

        by_id = {item.slot_id: item for item in observation.slots}
        for index, slot in enumerate(self.settings.layout.slots, start=1):
            result = by_id.get(slot.slot_id)
            marker = Marker()
            marker.header = boundary.header
            marker.ns = "cap_slots"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = (
                result.centroid_m[0]
                if result is not None and result.centroid_m is not None
                else slot.x_m
            )
            marker.pose.position.y = (
                result.centroid_m[1]
                if result is not None and result.centroid_m is not None
                else slot.y_m
            )
            marker.pose.orientation.w = 1.0
            diameter = 2.0 * slot.expected_radius_m
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = 0.006
            marker.color.a = 0.9
            if result is None or result.occupancy is SlotOccupancy.AMBIGUOUS:
                marker.color.r = 1.0
                marker.color.g = 0.1
            elif result.occupancy is SlotOccupancy.PRESENT:
                marker.color.r = 0.9
                marker.color.g = 0.1
                marker.color.b = 0.1
            else:
                marker.color.r = 0.3
                marker.color.g = 0.3
                marker.color.b = 0.3
            marker_array.markers.append(marker)

        pose = observation.fixture_pose
        if pose is not None:
            position = np.asarray(pose.position_fixture_m, dtype=np.float64)
            axis = np.asarray(
                pose.camera_optical_axis_fixture, dtype=np.float64
            )
            arrow = Marker()
            arrow.header = boundary.header
            arrow.ns = "camera_fixture_pose"
            arrow.id = 100
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.003
            arrow.scale.y = 0.007
            arrow.scale.z = 0.010
            arrow.color.r = 1.0
            arrow.color.g = 0.8
            arrow.color.b = 0.1
            arrow.color.a = 1.0
            arrow.points.append(
                Point(
                    x=float(position[0]),
                    y=float(position[1]),
                    z=float(position[2]),
                )
            )
            endpoint = position + axis * 0.060
            arrow.points.append(
                Point(
                    x=float(endpoint[0]),
                    y=float(endpoint[1]),
                    z=float(endpoint[2]),
                )
            )
            marker_array.markers.append(arrow)

            label = Marker()
            label.header = boundary.header
            label.ns = "camera_fixture_pose"
            label.id = 101
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = arrow.points[0]
            label.pose.position.z += 0.012
            label.pose.orientation.w = 1.0
            label.scale.z = 0.009
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = (
                f"camera tilt={pose.optical_axis_tilt_deg:.2f}deg "
                f"PnP={pose.reprojection_rms_px:.2f}px"
            )
            marker_array.markers.append(label)
        self.marker_pub.publish(marker_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CapInspectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
