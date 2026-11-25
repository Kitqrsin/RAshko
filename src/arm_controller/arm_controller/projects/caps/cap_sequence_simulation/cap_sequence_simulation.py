"""Collision-planned, offline-only seven-cap pick/place simulation.

This node deliberately has no ``/joint_commands`` or ``/servo_speed``
publisher.  Every arm or gripper movement is sent through the configured
FollowJointTrajectory action after the complete ROS graph has been verified as
the simulator-only executor -> servo_node path.  The synthetic scene is kept in
its own configuration and must never be treated as a physical measurement.
"""

from __future__ import annotations

from collections.abc import Sequence
import copy
import json
import math
import os
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPlanningScene,
    GetStateValidity,
)
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from arm_controller.projects.caps.cap_sequence_simulation.cap_sequence_simulation_config import (
    BoxVolume,
    CapSceneSlot,
    CylinderVolume,
    Point3,
    load_simulation_config,
)
from arm_controller.projects.caps.cap_sequence_simulation.cap_sequence_simulation_safety import (
    ACTION_STATUS_SUCCEEDED,
    FOLLOW_JOINT_TRAJECTORY_SUCCESS,
    CapSequenceSafetyError,
    CapSequenceState,
    OfflineBackendGraph,
    StageChecks,
    TransferStage,
    require_action_success,
    require_cartesian_path_fraction,
    require_motion_plan_success,
    require_offline_backend_graph,
)
from arm_controller.joint_contract import (
    HOME_DEGREES,
    JOINT_NAMES,
    servo_degrees_to_radians,
)
from arm_controller.projects.caps.workcell.red_cap_visual_servo import (
    RedBlobDetectorConfig,
    VisualServoController,
    detect_red_blobs,
    require_vertical_motion,
    select_alignment_blob,
)
from arm_controller.tool_pose import TOOL_LINK, tool_quaternion
from arm_controller.urdf_kinematics import UrdfKinematics


ARM_JOINT_NAMES = tuple(JOINT_NAMES[:5])
CAP_TOUCH_LINKS = ("gripper_base", "servo_disc_3")
SCENE_COMPONENTS = (
    PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
)
DEFAULT_TIMEOUT_S = 15.0


class CapSequenceSimulationNode(Node):
    """Run one deterministic seven-cap sequence against the offline backend."""

    def __init__(self) -> None:
        super().__init__("cap_sequence_simulation")
        config_path = str(
            self.declare_parameter(
                "config_file", "/workspace/config/cap_sequence.simulation.yaml"
            ).value
        ).strip()
        if not config_path:
            raise ValueError("config_file cannot be empty")
        self.exit_on_complete = bool(
            self.declare_parameter("exit_on_complete", False).value
        )
        self.config = load_simulation_config(config_path)
        urdf_path = Path(
            os.environ.get(
                "RASHKO_URDF_PATH",
                "/workspace/robot_urdf/robot_fixed.urdf",
            )
        )
        self.kinematics = UrdfKinematics(
            urdf_path.read_text(encoding="utf-8")
        )
        self.sequence = CapSequenceState(
            tuple(item.slot_id for item in self.config.sources)
        )
        self._joint_lock = threading.Lock()
        self._joint_state: JointState | None = None
        self._joint_state_received_at = 0.0
        self._failed = False
        self._finished = False
        self.exit_code = 1
        self._removed_unattached_cap: str | None = None
        self._attached_object: AttachedCollisionObject | None = None

        self.status_publisher = self.create_publisher(
            String, "/cap_sequence_simulation/status", 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/cap_sequence_simulation/markers", 10
        )
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 20
        )

        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.get_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self.state_validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )

        self.get_logger().warning(
            "OFFLINE SIMULATION ONLY: the scene uses supplied battery/cap "
            "measurements plus explicitly provisional depth, row-setback, "
            "post, and destination geometry; hardware is disabled"
        )
        self._publish_status("starting", "waiting for offline backend")

    @property
    def finished(self) -> bool:
        return self._finished

    def run(self) -> None:
        """Run the complete fail-fast sequence on the caller thread."""

        try:
            self._wait_for_backend()
            self._require_offline_graph()
            self._reset_synthetic_scene()
            self._require_current_state_valid()
            self._publish_markers()
            self._require_vertical_pick_reachability()

            destinations = {
                slot.slot_id: slot for slot in self.config.destinations
            }
            sources = {slot.slot_id: slot for slot in self.config.sources}
            for source_id, destination_id in (
                self.config.source_destination_pairs
            ):
                self._run_transfer(
                    sources[source_id], destinations[destination_id]
                )

            self.sequence.assert_complete()
            self.exit_code = 0
            self._finished = True
            self._publish_markers()
            self._publish_status(
                "complete", "all seven synthetic caps placed", result="pass"
            )
            self.get_logger().info(
                "PASS: all seven synthetic caps were collision-planned "
                "and placed"
            )
        except (CapSequenceSafetyError, RuntimeError, ValueError) as exc:
            self._abort(str(exc))

    def _run_transfer(
        self, source: CapSceneSlot, destination: CapSceneSlot
    ) -> None:
        cap_id = source.slot_id
        self._publish_status(
            "planning_pregrasp", f"{cap_id} -> {destination.slot_id}"
        )
        offsets = dict(self.config.visual_servo.coarse_offsets_xy_m)
        coarse_offset = offsets[cap_id]
        source_pregrasp = self._tool_pose(
            source.cap_center.x + coarse_offset[0],
            source.cap_center.y + coarse_offset[1],
            self.config.safe_transit_z_m,
        )
        # The open fingers already surround the upper part of the selected
        # cap at pregrasp. Permit contact only with that cap and only on the
        # two physical gripper links; neighboring caps remain obstacles.
        self._set_cap_touch_collisions(cap_id, True)
        plan_code, action_status, action_code = self._plan_and_execute(
            source_pregrasp
        )
        self._normalize_safe_transit_z()
        self.sequence.advance_checked(
            cap_id,
            TransferStage.PREGRASP_REACHED,
            StageChecks(
                motion_plan_error_code=plan_code,
                action_status=action_status,
                action_result_error_code=action_code,
                expected_action_result_error_code=(
                    FOLLOW_JOINT_TRAJECTORY_SUCCESS
                ),
            ),
        )

        aligned_x, aligned_y = self._visual_align_source(source)

        # The target cap is intentionally removed only after the collision-
        # checked pregrasp.  It represents the one object the fingers are
        # allowed to contact; all other caps remain collision objects.
        self._remove_world_cap(cap_id)
        grasp_pose = self._tool_pose(
            aligned_x,
            aligned_y,
            self.config.grasp_tool_z_m,
        )
        fraction, path_code, action_status, action_code = (
            self._cartesian_and_execute(grasp_pose)
        )
        self.sequence.advance_checked(
            cap_id,
            TransferStage.GRASP_REACHED,
            StageChecks(
                motion_plan_error_code=path_code,
                cartesian_fraction=fraction,
                action_status=action_status,
                action_result_error_code=action_code,
                expected_action_result_error_code=(
                    FOLLOW_JOINT_TRAJECTORY_SUCCESS
                ),
            ),
        )

        action_status, action_code = self._set_gripper(
            self.config.gripper_closed_degrees
        )
        self.sequence.advance_checked(
            cap_id,
            TransferStage.GRIPPER_CLOSED,
            StageChecks(
                action_status=action_status,
                action_result_error_code=action_code,
                expected_action_result_error_code=(
                    FOLLOW_JOINT_TRAJECTORY_SUCCESS
                ),
            ),
        )
        self._set_allowed_collision(
            cap_id, self.config.battery.object_id, True
        )
        self._attach_cap(cap_id, source)
        self.sequence.advance(cap_id, TransferStage.ATTACHED)

        lift_pose = self._tool_pose(
            aligned_x,
            aligned_y,
            self.config.safe_transit_z_m,
        )
        fraction, path_code, action_status, action_code = (
            self._cartesian_and_execute(lift_pose)
        )
        self.sequence.advance_checked(
            cap_id,
            TransferStage.LIFTED,
            StageChecks(
                motion_plan_error_code=path_code,
                cartesian_fraction=fraction,
                action_status=action_status,
                action_result_error_code=action_code,
                expected_action_result_error_code=(
                    FOLLOW_JOINT_TRAJECTORY_SUCCESS
                ),
            ),
        )
        self._set_allowed_collision(
            cap_id, self.config.battery.object_id, False
        )

        destination_above = self._tool_pose(
            destination.cap_center.x,
            destination.cap_center.y,
            self.config.safe_transit_z_m,
        )
        plan_code, action_status, action_code = self._plan_and_execute(
            destination_above
        )
        # Position-only OMPL goals may finish a few millimetres from the
        # requested TCP point while still satisfying their joint tolerance.
        # Remove that residual on the collision-checked transit plane before
        # the vertical placement gate is evaluated.
        self._normalize_safe_transit_z()
        self._refine_safe_height_xy(destination_above)
        self.sequence.advance_checked(
            cap_id,
            TransferStage.PLACE_ABOVE_REACHED,
            StageChecks(
                motion_plan_error_code=plan_code,
                action_status=action_status,
                action_result_error_code=action_code,
                expected_action_result_error_code=(
                    FOLLOW_JOINT_TRAJECTORY_SUCCESS
                ),
            ),
        )

        self._set_allowed_collision(cap_id, self.config.tray.object_id, True)
        place_pose = self._tool_pose(
            destination.cap_center.x,
            destination.cap_center.y,
            self.config.place_tool_z_m,
        )
        fraction, path_code, action_status, action_code = (
            self._cartesian_and_execute(place_pose)
        )
        self.sequence.advance_checked(
            cap_id,
            TransferStage.PLACE_REACHED,
            StageChecks(
                motion_plan_error_code=path_code,
                cartesian_fraction=fraction,
                action_status=action_status,
                action_result_error_code=action_code,
                expected_action_result_error_code=(
                    FOLLOW_JOINT_TRAJECTORY_SUCCESS
                ),
            ),
        )

        action_status, action_code = self._set_gripper(
            self.config.gripper_open_degrees
        )
        self.sequence.advance_checked(
            cap_id,
            TransferStage.GRIPPER_OPENED,
            StageChecks(
                action_status=action_status,
                action_result_error_code=action_code,
                expected_action_result_error_code=(
                    FOLLOW_JOINT_TRAJECTORY_SUCCESS
                ),
            ),
        )
        self._detach_cap_at(cap_id, destination)
        self.sequence.advance(cap_id, TransferStage.DETACHED)
        # Moving an object between world and attached state can cause MoveIt
        # to prune object-specific ACM entries. Reassert only the released
        # cap's intended finger contact until the vertical retreat completes.
        self._set_cap_touch_collisions(cap_id, True)
        self._set_allowed_collision(cap_id, self.config.tray.object_id, False)

        fraction, path_code, action_status, action_code = (
            self._cartesian_and_execute(destination_above)
        )
        self.sequence.advance_checked(
            cap_id,
            TransferStage.RETREATED,
            StageChecks(
                motion_plan_error_code=path_code,
                cartesian_fraction=fraction,
                action_status=action_status,
                action_result_error_code=action_code,
                expected_action_result_error_code=(
                    FOLLOW_JOINT_TRAJECTORY_SUCCESS
                ),
            ),
        )
        self._set_cap_touch_collisions(cap_id, False)
        self.sequence.advance(cap_id, TransferStage.COMPLETE)
        self._publish_markers()
        self._publish_status(
            "cap_complete",
            f"placed {cap_id} at {destination.slot_id}",
        )

    def _wait_for_backend(self) -> None:
        services = (
            self.apply_scene_client,
            self.get_scene_client,
            self.plan_client,
            self.cartesian_client,
            self.state_validity_client,
        )
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and time.monotonic() < deadline:
            services_ready = all(
                client.service_is_ready() for client in services
            )
            action_ready = self.trajectory_client.server_is_ready()
            with self._joint_lock:
                state_ready = self._joint_state is not None
            if services_ready and action_ready and state_ready:
                return
            time.sleep(0.05)
        raise RuntimeError(
            "offline MoveIt, trajectory action, or joint state is unavailable"
        )

    def _visual_align_source(
        self, source: CapSceneSlot
    ) -> tuple[float, float]:
        """Centre one rendered red cap using only bounded safe-height XY moves."""

        settings = self.config.visual_servo
        controller = VisualServoController(settings.controller)
        detector = RedBlobDetectorConfig(
            min_area_px2=50.0,
            max_area_px2=5000.0,
        )
        width, height = settings.image_size_px
        target_u, target_v = settings.controller.target_pixel
        metres_u, metres_v = settings.metres_per_pixel_xy
        while True:
            current = self._current_joint_state()
            tool = self.kinematics.forward(current.position[:5]).position
            if abs(float(tool[2]) - self.config.safe_transit_z_m) > 0.001:
                raise RuntimeError(
                    "visual servoing is permitted only at safe transit Z"
                )
            image = np.full((height, width, 3), 45, dtype=np.uint8)
            rendered_source = False
            completed = set(self.sequence.completed_cap_ids)
            for visible_cap in self.config.sources:
                if visible_cap.slot_id in completed:
                    continue
                pixel_u = target_u + (
                    visible_cap.cap_center.x - float(tool[0])
                ) / metres_u
                pixel_v = target_v - (
                    visible_cap.cap_center.y - float(tool[1])
                ) / metres_v
                margin = settings.cap_radius_px + 1
                if (
                    margin <= pixel_u < width - margin
                    and margin <= pixel_v < height - margin
                ):
                    cv2.circle(
                        image,
                        (int(round(pixel_u)), int(round(pixel_v))),
                        settings.cap_radius_px,
                        (0, 0, 255),
                        -1,
                        cv2.LINE_AA,
                    )
                    rendered_source |= visible_cap.slot_id == source.slot_id
            if not rendered_source:
                raise RuntimeError("synthetic target cap left the camera image")
            blobs = detect_red_blobs(image, detector)
            selected = select_alignment_blob(
                blobs,
                settings.controller.target_pixel,
                acquisition_radius_px=settings.controller.acquisition_radius_px,
                ambiguity_margin_px=settings.controller.ambiguity_margin_px,
            )
            command = controller.observe(selected)
            self._publish_status(
                "visual_align",
                (
                    f"{source.slot_id} iteration={command.iteration} "
                    f"error=({command.error_px[0]:.2f},"
                    f"{command.error_px[1]:.2f})px"
                ),
            )
            if command.alignment_verified:
                return float(tool[0]), float(tool[1])
            correction_x, correction_y = command.correction_xy_m
            if command.centered:
                continue
            correction_target = self._tool_pose(
                float(tool[0]) + correction_x,
                float(tool[1]) + correction_y,
                self.config.safe_transit_z_m,
            )
            self._safe_height_cartesian_xy_and_execute(correction_target)

    def _require_offline_graph(self) -> None:
        graph = OfflineBackendGraph(
            node_names=tuple(
                self._qualified_name(name, namespace)
                for name, namespace in self.get_node_names_and_namespaces()
            ),
            joint_command_publishers=self._topic_publishers("/joint_commands"),
            joint_command_subscribers=self._topic_subscribers(
                "/joint_commands"
            ),
            joint_state_publishers=self._topic_publishers("/joint_states"),
            raspberry_status_publishers=self._topic_publishers(
                "/raspberry/status"
            ),
        )
        require_offline_backend_graph(graph)

    @staticmethod
    def _qualified_name(name: str, namespace: str) -> str:
        namespace = namespace.rstrip("/")
        return f"{namespace}/{name}" if namespace else f"/{name}"

    def _topic_publishers(self, topic: str) -> tuple[str, ...]:
        return tuple(
            sorted(
                self._qualified_name(info.node_name, info.node_namespace)
                for info in self.get_publishers_info_by_topic(topic)
            )
        )

    def _topic_subscribers(self, topic: str) -> tuple[str, ...]:
        return tuple(
            sorted(
                self._qualified_name(info.node_name, info.node_namespace)
                for info in self.get_subscriptions_info_by_topic(topic)
            )
        )

    def _joint_state_callback(self, message: JointState) -> None:
        if (
            len(message.name) != len(message.position)
            or not set(JOINT_NAMES).issubset(message.name)
            or any(not math.isfinite(value) for value in message.position)
        ):
            return
        positions = dict(zip(message.name, message.position))
        canonical = JointState()
        canonical.header = message.header
        canonical.name = list(JOINT_NAMES)
        canonical.position = [float(positions[name]) for name in JOINT_NAMES]
        with self._joint_lock:
            self._joint_state = canonical
            self._joint_state_received_at = time.monotonic()

    def _current_joint_state(self) -> JointState:
        with self._joint_lock:
            if (
                self._joint_state is None
                or time.monotonic() - self._joint_state_received_at > 0.5
            ):
                raise RuntimeError("simulated joint state is missing or stale")
            return copy.deepcopy(self._joint_state)

    def _current_robot_state(self) -> RobotState:
        state = RobotState()
        state.joint_state = self._current_joint_state()
        if self._attached_object is not None:
            state.attached_collision_objects = [
                copy.deepcopy(self._attached_object)
            ]
        state.is_diff = False
        return state

    def _require_vertical_pick_reachability(self) -> None:
        """Reject the scene before planning if a real-height grasp has no IK."""

        seed = tuple(self._current_joint_state().position[:5])
        unreachable = []
        for source in self.config.sources:
            solution = self.kinematics.solve(
                (
                    source.cap_center.x,
                    source.cap_center.y,
                    self.config.grasp_tool_z_m,
                ),
                math.radians(self.config.tool_elevation_deg),
                seed,
                wrist_roll=0.0,
                max_iterations=200,
            )
            if solution is None:
                unreachable.append(source.slot_id)
            else:
                seed = solution.radians
        if unreachable:
            raise RuntimeError(
                "measurement-informed top-down grasp is outside the exact "
                "URDF/joint-limit workspace at "
                f"tool Z={self.config.grasp_tool_z_m:.4f} m for: "
                + ", ".join(unreachable)
                + ". Keep hardware disabled; change the physical height/"
                "placement or prove a different grasp geometry first"
            )

    def _tool_pose(self, x: float, y: float, z: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.config.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        quaternion = tool_quaternion(
            math.atan2(y, x),
            math.radians(self.config.tool_elevation_deg),
            0.0,
        )
        pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        )
        return pose

    def _solve_feasible_target(
        self,
        target: PoseStamped,
        *,
        seed_radians: Sequence[float] | None = None,
        log_result: bool = True,
    ) -> tuple[tuple[float, ...], PoseStamped]:
        if seed_radians is None:
            seed_radians = self._current_joint_state().position[:5]
        desired = target.pose.position
        solution = self.kinematics.solve(
            (desired.x, desired.y, desired.z),
            math.radians(self.config.tool_elevation_deg),
            seed_radians,
            wrist_roll=0.0,
            max_iterations=160,
        )
        if solution is None:
            raise RuntimeError(
                "exact URDF solver found no feasible top-down target at "
                f"({desired.x:.4f}, {desired.y:.4f}, {desired.z:.4f})"
            )
        if solution.position_error > self.config.cap.radius_m:
            raise RuntimeError(
                "exact URDF target residual exceeds the synthetic cap radius: "
                f"{solution.position_error * 1000.0:.2f} mm at "
                f"({desired.x:.4f}, {desired.y:.4f}, {desired.z:.4f})"
            )

        forward = self.kinematics.forward(solution.radians)
        feasible = PoseStamped()
        feasible.header = target.header
        feasible.pose.position.x = float(forward.position[0])
        feasible.pose.position.y = float(forward.position[1])
        feasible.pose.position.z = float(forward.position[2])
        quaternion = self._rotation_matrix_quaternion(forward.rotation)
        feasible.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        )
        if log_result:
            self.get_logger().info(
                "Feasible target residual: "
                f"{solution.position_error * 1000.0:.2f} mm, "
                f"{math.degrees(solution.direction_error):.2f} deg"
            )
        return solution.radians, feasible

    @staticmethod
    def _rotation_matrix_quaternion(rotation) -> tuple[float, ...]:
        """Convert the exact-URDF rotation matrix to normalized XYZW."""

        m00, m01, m02 = rotation[0]
        m10, m11, m12 = rotation[1]
        m20, m21, m22 = rotation[2]
        trace = m00 + m11 + m22
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            values = (
                (m21 - m12) / scale,
                (m02 - m20) / scale,
                (m10 - m01) / scale,
                0.25 * scale,
            )
        elif m00 > m11 and m00 > m22:
            scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            values = (
                0.25 * scale,
                (m01 + m10) / scale,
                (m02 + m20) / scale,
                (m21 - m12) / scale,
            )
        elif m11 > m22:
            scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            values = (
                (m01 + m10) / scale,
                0.25 * scale,
                (m12 + m21) / scale,
                (m02 - m20) / scale,
            )
        else:
            scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            values = (
                (m02 + m20) / scale,
                (m12 + m21) / scale,
                0.25 * scale,
                (m10 - m01) / scale,
            )
        norm = math.sqrt(sum(value * value for value in values))
        return tuple(float(value / norm) for value in values)

    def _plan_and_execute(self, target: PoseStamped) -> tuple[int, int, int]:
        self._require_offline_graph()
        goal_positions, _ = self._solve_feasible_target(target)
        self._require_joint_target_valid(goal_positions)
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name = "arm"
        plan.start_state = self._current_robot_state()
        plan.num_planning_attempts = 8
        plan.allowed_planning_time = 6.0
        plan.max_velocity_scaling_factor = 0.25
        plan.max_acceleration_scaling_factor = 0.25
        plan.planner_id = "RRTConnectkConfigDefault"
        constraints = Constraints()
        constraints.name = "cap_sequence_joint_goal"
        for name, position in zip(ARM_JOINT_NAMES, goal_positions):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.003
            constraint.tolerance_below = 0.003
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        plan.goal_constraints = [constraints]

        response = self._call(
            self.plan_client, request, 10.0, "MoveIt motion plan"
        ).motion_plan_response
        require_motion_plan_success(response.error_code.val)
        trajectory = response.trajectory.joint_trajectory
        if len(trajectory.points) == 1:
            if self._trajectory_point_matches_current(trajectory):
                self._require_tool_position(target.pose.position)
                return (
                    response.error_code.val,
                    ACTION_STATUS_SUCCEEDED,
                    FOLLOW_JOINT_TRAJECTORY_SUCCESS,
                )
            # MoveIt can legitimately omit a separate start waypoint for a
            # short joint-space plan. The target is still planner-produced;
            # prepend fresh feedback so the action contract has a start and
            # an end instead of rejecting a safe small correction.
            self._prepend_current_trajectory_point(trajectory)
        self._require_trajectory(trajectory)
        action_status, action_code = self._execute_trajectory(trajectory)
        return response.error_code.val, action_status, action_code

    def _require_joint_target_valid(
        self, arm_positions: Sequence[float]
    ) -> None:
        state = self._current_robot_state()
        positions = dict(
            zip(state.joint_state.name, state.joint_state.position)
        )
        positions.update(zip(ARM_JOINT_NAMES, arm_positions))
        state.joint_state.position = [
            float(positions[name]) for name in state.joint_state.name
        ]
        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = "arm"
        response = self._call(
            self.state_validity_client,
            request,
            5.0,
            "target state validity",
        )
        if response.valid:
            return
        contacts = [
            (
                f"{item.contact_body_1}<->{item.contact_body_2} "
                f"at ({item.position.x:.4f}, {item.position.y:.4f}, "
                f"{item.position.z:.4f}), depth={item.depth:.4f}m"
            )
            for item in response.contacts
        ]
        raise RuntimeError(
            "MoveIt rejected the feasible target state: "
            + ("; ".join(contacts) if contacts else "collision/constraint")
        )

    def _cartesian_and_execute(
        self, target: PoseStamped
    ) -> tuple[float, int, int, int]:
        self._require_offline_graph()
        start_state = self._current_robot_state()
        current_tool = self.kinematics.forward(
            start_state.joint_state.position[:5]
        )
        require_vertical_motion(
            current_tool.position,
            (
                target.pose.position.x,
                target.pose.position.y,
                target.pose.position.z,
            ),
        )
        return self._cartesian_from_state(target, start_state, current_tool)

    def _safe_height_cartesian_xy_and_execute(
        self, target: PoseStamped
    ) -> tuple[float, int, int, int]:
        """Execute one bounded collision-checked XY nudge at transit height."""

        self._require_offline_graph()
        start_state = self._current_robot_state()
        current_tool = self.kinematics.forward(
            start_state.joint_state.position[:5]
        )
        target_xyz = np.asarray(
            (
                target.pose.position.x,
                target.pose.position.y,
                target.pose.position.z,
            ),
            dtype=np.float64,
        )
        if (
            abs(float(current_tool.position[2]) - self.config.safe_transit_z_m)
            > 0.001
            or abs(float(target_xyz[2]) - self.config.safe_transit_z_m) > 1e-9
        ):
            raise RuntimeError(
                "visual-servo XY correction left the safe transit plane"
            )
        xy_step = float(
            np.linalg.norm(target_xyz[:2] - current_tool.position[:2])
        )
        if xy_step > self.config.visual_servo.controller.max_step_m + 1e-9:
            raise RuntimeError(
                f"high-plane Cartesian step {xy_step * 1000.0:.2f}mm "
                "exceeds its configured bound"
            )
        return self._cartesian_from_state(target, start_state, current_tool)

    def _normalize_safe_transit_z(self) -> None:
        """Reach the exact transit plane through a vertical checked segment."""

        current = self._current_joint_state()
        tool = self.kinematics.forward(current.position[:5]).position
        if abs(float(tool[2]) - self.config.safe_transit_z_m) <= 0.0005:
            return
        target = self._tool_pose(
            float(tool[0]),
            float(tool[1]),
            self.config.safe_transit_z_m,
        )
        self._cartesian_and_execute(target)

    def _refine_safe_height_xy(self, target: PoseStamped) -> None:
        """Remove a high-plane OMPL residual through bounded Cartesian steps."""

        maximum_step = self.config.visual_servo.controller.max_step_m
        for _ in range(6):
            current = self._current_joint_state()
            tool = self.kinematics.forward(current.position[:5]).position
            target_xy = np.asarray(
                (target.pose.position.x, target.pose.position.y),
                dtype=np.float64,
            )
            error = target_xy - tool[:2]
            distance = float(np.linalg.norm(error))
            if distance <= 0.0005:
                return
            step = error
            if distance > maximum_step:
                step = error * (maximum_step / distance)
            intermediate = self._tool_pose(
                float(tool[0] + step[0]),
                float(tool[1] + step[1]),
                self.config.safe_transit_z_m,
            )
            self._safe_height_cartesian_xy_and_execute(intermediate)
        raise RuntimeError(
            "bounded high-plane refinement did not reach the destination XY"
        )

    def _cartesian_from_state(
        self,
        target: PoseStamped,
        start_state: RobotState,
        current_tool,
    ) -> tuple[float, int, int, int]:
        """Plan and execute one collision-checked single-waypoint path."""

        current_quaternion = self._rotation_matrix_quaternion(
            current_tool.rotation
        )
        waypoint = copy.deepcopy(target.pose)
        # Position-only IK deliberately leaves orientation free. Reuse the
        # current FK orientation so interpolation is driven only by the
        # requested vertical translation.
        waypoint.orientation = Quaternion(
            x=current_quaternion[0],
            y=current_quaternion[1],
            z=current_quaternion[2],
            w=current_quaternion[3],
        )
        request = GetCartesianPath.Request()
        request.header.frame_id = self.config.frame_id
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state = start_state
        request.group_name = "arm"
        request.link_name = TOOL_LINK
        request.waypoints = [waypoint]
        request.max_step = 0.001
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        response = self._call(
            self.cartesian_client,
            request,
            10.0,
            "MoveIt vertical Cartesian path",
        )
        require_motion_plan_success(response.error_code.val)
        require_cartesian_path_fraction(response.fraction)
        trajectory = response.solution.joint_trajectory
        if len(trajectory.points) == 1:
            if self._trajectory_point_matches_current(trajectory):
                self._require_tool_position(waypoint.position)
                return (
                    float(response.fraction),
                    response.error_code.val,
                    ACTION_STATUS_SUCCEEDED,
                    FOLLOW_JOINT_TRAJECTORY_SUCCESS,
                )
            self._prepend_current_trajectory_point(trajectory)
        self._require_trajectory(trajectory)
        action_status, action_code = self._execute_trajectory(trajectory)
        self._require_tool_position(waypoint.position)
        return (
            float(response.fraction),
            response.error_code.val,
            action_status,
            action_code,
        )

    def _require_tool_position(self, target: Point) -> None:
        current = self._current_joint_state()
        actual = self.kinematics.forward(current.position[:5]).position
        error = math.sqrt(
            (float(actual[0]) - target.x) ** 2
            + (float(actual[1]) - target.y) ** 2
            + (float(actual[2]) - target.z) ** 2
        )
        if error > 0.001:
            raise RuntimeError(
                "position-only Cartesian segment missed its XYZ target by "
                f"{error * 1000.0:.2f} mm"
            )

    def _trajectory_point_matches_current(
        self, trajectory: JointTrajectory
    ) -> bool:
        current = self._current_joint_state()
        values = dict(zip(current.name, current.position))
        target = trajectory.points[0]
        if len(target.positions) != len(trajectory.joint_names):
            return False
        return all(
            name in values and abs(values[name] - position) <= 1e-4
            for name, position in zip(
                trajectory.joint_names, target.positions
            )
        )

    def _prepend_current_trajectory_point(
        self, trajectory: JointTrajectory
    ) -> None:
        current = self._current_joint_state()
        values = dict(zip(current.name, current.position))
        if any(name not in values for name in trajectory.joint_names):
            raise RuntimeError(
                "Cartesian trajectory names are absent from joint feedback"
            )
        start = JointTrajectoryPoint()
        start.positions = [
            float(values[name]) for name in trajectory.joint_names
        ]
        target = trajectory.points[0]
        if (
            target.time_from_start.sec == 0
            and target.time_from_start.nanosec == 0
        ):
            target.time_from_start.sec = 1
        trajectory.points.insert(0, start)

    @staticmethod
    def _require_trajectory(trajectory: JointTrajectory) -> None:
        if tuple(trajectory.joint_names) != ARM_JOINT_NAMES:
            raise RuntimeError(
                "MoveIt trajectory joint order does not match the "
                "canonical arm"
            )
        if len(trajectory.points) < 2:
            raise RuntimeError(
                "MoveIt returned an empty or one-point trajectory"
            )

    def _execute_trajectory(
        self, trajectory: JointTrajectory
    ) -> tuple[int, int]:
        self._require_offline_graph()
        if not self.trajectory_client.server_is_ready():
            raise RuntimeError("trajectory action server disappeared")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal_future = self.trajectory_client.send_goal_async(goal)
        goal_handle = self._wait_future(goal_future, 5.0, "trajectory goal")
        if not goal_handle.accepted:
            raise RuntimeError("trajectory executor rejected a planned path")
        result_wrapper = self._wait_future(
            goal_handle.get_result_async(), 45.0, "trajectory result"
        )
        require_action_success(
            result_wrapper.status,
            result_wrapper.result.error_code,
            expected_result_error_code=FOLLOW_JOINT_TRAJECTORY_SUCCESS,
        )
        return result_wrapper.status, result_wrapper.result.error_code

    def _set_gripper(self, target_degrees: float) -> tuple[int, int]:
        current = self._current_joint_state()
        target_values = list(HOME_DEGREES)
        target_values[5] = float(target_degrees)
        target_radians = servo_degrees_to_radians(target_values)[5]
        trajectory = JointTrajectory()
        trajectory.joint_names = [JOINT_NAMES[5]]
        start = JointTrajectoryPoint()
        start.positions = [current.position[5]]
        target = JointTrajectoryPoint()
        target.positions = [target_radians]
        target.time_from_start.sec = 1
        trajectory.points = [start, target]
        return self._execute_trajectory(trajectory)

    def _reset_synthetic_scene(self) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        current_scene = self._get_scene()
        existing_attached = {
            item.object.id
            for item in current_scene.robot_state.attached_collision_objects
        }
        for cap_id in existing_attached:
            attached = AttachedCollisionObject()
            attached.link_name = TOOL_LINK
            attached.object.id = cap_id
            attached.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(attached)

        scene.world.collision_objects.extend(
            (
                self._box_object(self.config.table),
                self._box_object(self.config.battery),
                self._box_object(self.config.tray),
            )
        )
        scene.world.collision_objects.extend(
            self._cylinder_object(item)
            for item in self.config.protected_objects
        )
        scene.world.collision_objects.extend(
            self._world_cap_object(slot.slot_id, slot.cap_center)
            for slot in self.config.sources
        )
        self._apply_scene(scene)
        expected_world = {
            self.config.table.object_id,
            self.config.battery.object_id,
            self.config.tray.object_id,
            *(item.object_id for item in self.config.protected_objects),
            *(item.slot_id for item in self.config.sources),
        }
        self._confirm_scene(
            expected_world=expected_world, expected_attached=set()
        )
        self._attached_object = None

    def _require_current_state_valid(self) -> None:
        request = GetStateValidity.Request()
        request.robot_state = self._current_robot_state()
        request.group_name = "arm"
        response = self._call(
            self.state_validity_client, request, 5.0, "state validity"
        )
        if not response.valid:
            contacts = sorted(
                {
                    f"{item.contact_body_1}<->{item.contact_body_2}"
                    for item in response.contacts
                }
            )
            raise RuntimeError(
                "synthetic scene places the starting arm in collision: "
                + (", ".join(contacts) if contacts else "unknown collision")
            )

    def _remove_world_cap(self, cap_id: str) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        removal = CollisionObject()
        removal.header.frame_id = self.config.frame_id
        removal.id = cap_id
        removal.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(removal)
        self._apply_scene(scene)
        self._removed_unattached_cap = cap_id
        self._confirm_object_state(cap_id, world=False, attached=False)

    def _attach_cap(self, cap_id: str, source: CapSceneSlot) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        attached = AttachedCollisionObject()
        attached.link_name = TOOL_LINK
        attached.touch_links = list(CAP_TOUCH_LINKS)
        attached.object.header.frame_id = TOOL_LINK
        attached.object.id = cap_id
        attached.object.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [
            self.config.cap.height_m,
            self.config.cap.radius_m,
        ]
        # Preserve the world cap pose at the moment it is attached. This is
        # more accurate than assuming tool_tip is at the cap top: the actual
        # CAD fingers extend below tool_tip and the 5-DOF IK has a small,
        # explicitly bounded Cartesian residual.
        current = self._current_joint_state()
        tool = self.kinematics.forward(current.position[:5])
        world_delta = (
            source.cap_center.x - float(tool.position[0]),
            source.cap_center.y - float(tool.position[1]),
            source.cap_center.z - float(tool.position[2]),
        )
        pose = Pose()
        relative = tuple(
            sum(
                float(tool.rotation[row][column]) * world_delta[row]
                for row in range(3)
            )
            for column in range(3)
        )
        pose.position.x, pose.position.y, pose.position.z = relative
        inverse_rotation = tuple(
            tuple(float(tool.rotation[column][row]) for column in range(3))
            for row in range(3)
        )
        quaternion = self._rotation_matrix_quaternion(inverse_rotation)
        pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        )
        attached.object.primitives = [primitive]
        attached.object.primitive_poses = [pose]
        scene.robot_state.attached_collision_objects = [attached]
        self._apply_scene(scene)
        self._removed_unattached_cap = None
        self._confirm_object_state(cap_id, world=False, attached=True)
        self._attached_object = copy.deepcopy(attached)

    def _detach_cap_at(self, cap_id: str, destination: CapSceneSlot) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        attached = AttachedCollisionObject()
        attached.link_name = TOOL_LINK
        attached.object.id = cap_id
        attached.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects = [attached]
        scene.world.collision_objects = [
            self._world_cap_object(cap_id, destination.cap_center)
        ]
        self._apply_scene(scene)
        self._confirm_object_state(cap_id, world=True, attached=False)
        self._attached_object = None

    def _set_cap_touch_collisions(self, cap_id: str, allowed: bool) -> None:
        for link_name in CAP_TOUCH_LINKS:
            self._set_allowed_collision(cap_id, link_name, allowed)

    def _set_allowed_collision(
        self, left_name: str, right_name: str, allowed: bool
    ) -> None:
        scene = self._get_scene()
        matrix = scene.allowed_collision_matrix
        names = list(matrix.entry_names)
        rows = [list(row.enabled) for row in matrix.entry_values]
        if (
            any(len(row) != len(names) for row in rows)
            or len(rows) != len(names)
        ):
            raise RuntimeError(
                "MoveIt returned a malformed allowed-collision matrix"
            )
        for name in (left_name, right_name):
            if name not in names:
                names.append(name)
                for row in rows:
                    row.append(False)
                rows.append([False] * len(names))
        left_index = names.index(left_name)
        right_index = names.index(right_name)
        rows[left_index][right_index] = allowed
        rows[right_index][left_index] = allowed

        update = PlanningScene()
        update.is_diff = True
        update.allowed_collision_matrix.entry_names = names
        update.allowed_collision_matrix.entry_values = [
            AllowedCollisionEntry(enabled=row) for row in rows
        ]
        update.allowed_collision_matrix.default_entry_names = list(
            matrix.default_entry_names
        )
        update.allowed_collision_matrix.default_entry_values = list(
            matrix.default_entry_values
        )
        self._apply_scene(update)

        confirmed = self._get_scene().allowed_collision_matrix
        confirmed_names = list(confirmed.entry_names)
        if (
            left_name not in confirmed_names
            or right_name not in confirmed_names
        ):
            defaults = dict(
                zip(
                    confirmed.default_entry_names,
                    confirmed.default_entry_values,
                )
            )
            default_allowed = bool(defaults.get(left_name, False)) or bool(
                defaults.get(right_name, False)
            )
            if not allowed and not default_allowed:
                # MoveIt may prune an explicit false pair because absence is
                # already the fail-closed default.
                return
            raise RuntimeError(
                "MoveIt did not retain the collision-pair update"
            )
        left_index = confirmed_names.index(left_name)
        right_index = confirmed_names.index(right_name)
        confirmed_value = confirmed.entry_values[left_index].enabled[
            right_index
        ]
        if bool(confirmed_value) != allowed:
            raise RuntimeError("MoveIt collision-pair confirmation disagrees")

    def _apply_scene(self, scene: PlanningScene) -> None:
        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = self._call(
            self.apply_scene_client, request, 5.0, "apply planning scene"
        )
        if not response.success:
            raise RuntimeError("MoveIt rejected the synthetic planning scene")

    def _get_scene(self) -> PlanningScene:
        request = GetPlanningScene.Request()
        request.components.components = SCENE_COMPONENTS
        return self._call(
            self.get_scene_client, request, 5.0, "get planning scene"
        ).scene

    def _confirm_scene(
        self, *, expected_world: set[str], expected_attached: set[str]
    ) -> None:
        scene = self._get_scene()
        world = {item.id for item in scene.world.collision_objects}
        attached = {
            item.object.id
            for item in scene.robot_state.attached_collision_objects
        }
        missing_world = expected_world - world
        if missing_world or attached != expected_attached:
            raise RuntimeError(
                "planning scene confirmation failed: "
                f"missing_world={sorted(missing_world)}, "
                f"attached={sorted(attached)}"
            )

    def _confirm_object_state(
        self, cap_id: str, *, world: bool, attached: bool
    ) -> None:
        scene = self._get_scene()
        world_ids = {item.id for item in scene.world.collision_objects}
        attached_ids = {
            item.object.id
            for item in scene.robot_state.attached_collision_objects
        }
        world_matches = (cap_id in world_ids) is world
        attached_matches = (cap_id in attached_ids) is attached
        if not world_matches or not attached_matches:
            raise RuntimeError(
                f"planning scene did not confirm {cap_id}: "
                f"world={cap_id in world_ids}, "
                f"attached={cap_id in attached_ids}"
            )

    def _box_object(self, volume: BoxVolume) -> CollisionObject:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [volume.size.x, volume.size.y, volume.size.z]
        return self._collision_object(
            volume.object_id, primitive, volume.center
        )

    def _cylinder_object(self, volume: CylinderVolume) -> CollisionObject:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [volume.height_m, volume.radius_m]
        return self._collision_object(
            volume.object_id, primitive, volume.center
        )

    def _world_cap_object(
        self, cap_id: str, center: Point3
    ) -> CollisionObject:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER
        primitive.dimensions = [
            self.config.cap.height_m,
            self.config.cap.radius_m,
        ]
        return self._collision_object(cap_id, primitive, center)

    def _collision_object(
        self, object_id: str, primitive: SolidPrimitive, center: Point3
    ) -> CollisionObject:
        item = CollisionObject()
        item.header.frame_id = self.config.frame_id
        item.id = object_id
        item.operation = CollisionObject.ADD
        pose = Pose()
        pose.position.x = center.x
        pose.position.y = center.y
        pose.position.z = center.z
        pose.orientation.w = 1.0
        item.primitives = [primitive]
        item.primitive_poses = [pose]
        return item

    def _call(self, client, request, timeout: float, description: str):
        if not client.service_is_ready():
            raise RuntimeError(f"{description} service disappeared")
        return self._wait_future(
            client.call_async(request), timeout, description
        )

    @staticmethod
    def _wait_future(future, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                exception = future.exception()
                if exception is not None:
                    raise RuntimeError(f"{description} failed: {exception}")
                result = future.result()
                if result is None:
                    raise RuntimeError(f"{description} returned no result")
                return result
            time.sleep(0.01)
        raise RuntimeError(f"timed out waiting for {description}")

    def _publish_status(
        self, stage: str, message: str, *, result: str = "running"
    ) -> None:
        if stage in {"planning_pregrasp", "cap_complete", "complete"}:
            self.get_logger().info(f"{stage}: {message}")
        payload = {
            "mode": "synthetic_offline_simulation",
            "simulation_only": True,
            "camera_driven": bool(self.config.visual_servo.enabled),
            "camera_source": "synthetic_red_cap_renderer",
            "hardware_execution_permitted": False,
            "stage": stage,
            "message": message,
            "active_cap_id": self.sequence.active_cap_id,
            "completed_cap_ids": list(self.sequence.completed_cap_ids),
            "completed_count": len(self.sequence.completed_cap_ids),
            "result": result,
            "config_sha256": self.config.config_sha256,
        }
        self.status_publisher.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )

    def _publish_markers(self) -> None:
        markers = MarkerArray()
        completed = set(self.sequence.completed_cap_ids)
        destinations = dict(self.config.source_destination_pairs)
        destination_slots = {
            item.slot_id: item for item in self.config.destinations
        }
        for index, source in enumerate(self.config.sources):
            destination = destination_slots[destinations[source.slot_id]]
            marker = Marker()
            marker.header.frame_id = self.config.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "synthetic_transfer_paths"
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.002
            marker.color = (
                ColorRGBA(r=0.1, g=0.8, b=0.2, a=0.7)
                if source.slot_id in completed
                else ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.45)
            )
            marker.points = [
                Point(
                    x=source.cap_center.x,
                    y=source.cap_center.y,
                    z=self.config.safe_transit_z_m,
                ),
                Point(
                    x=source.cap_center.x,
                    y=source.cap_center.y,
                    z=self.config.grasp_tool_z_m,
                ),
                Point(
                    x=source.cap_center.x,
                    y=source.cap_center.y,
                    z=self.config.safe_transit_z_m,
                ),
                Point(
                    x=destination.cap_center.x,
                    y=destination.cap_center.y,
                    z=self.config.safe_transit_z_m,
                ),
                Point(
                    x=destination.cap_center.x,
                    y=destination.cap_center.y,
                    z=self.config.place_tool_z_m,
                ),
                Point(
                    x=destination.cap_center.x,
                    y=destination.cap_center.y,
                    z=self.config.safe_transit_z_m,
                ),
            ]
            markers.markers.append(marker)
        self.marker_publisher.publish(markers)

    def _abort(self, reason: str) -> None:
        self.sequence.abort(reason)
        self._failed = True
        self._finished = True
        self.exit_code = 1
        self._publish_status("aborted", reason, result="fail")
        self.get_logger().error(f"ABORTED: {reason}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CapSequenceSimulationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run()
        # Keep the successful final scene visible in RViz until the launch is
        # stopped.  A failed run exits promptly with a non-zero status.
        if node.exit_code == 0 and not node.exit_on_complete:
            while rclpy.ok():
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = node.exit_code
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
