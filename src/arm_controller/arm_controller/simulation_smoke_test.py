"""Automated end-to-end check for the offline ROS simulation."""

from __future__ import annotations

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from arm_controller.joint_contract import (
    HOME_DEGREES,
    JOINT_NAMES,
    merge_joint_radians,
    servo_degrees_to_radians,
)
from arm_controller.tool_pose import TOOL_LINK


class SimulationSmokeTest(Node):
    """Checks feedback motion, rate limiting, E-stop recovery, TF, and MoveIt IK."""

    FIRST_TARGET = (110.0, 70.0, 120.0, 60.0, 115.0, 105.0)
    SECOND_TARGET = (75.0, 105.0, 65.0, 115.0, 70.0, 80.0)
    TRAJECTORY_TARGET = (100.0, 82.0, 108.0, 76.0, 98.0, 92.0)

    def __init__(self) -> None:
        super().__init__("simulation_smoke_test")
        self.command_publisher = self.create_publisher(Float32MultiArray, "/joint_commands", 10)
        self.speed_publisher = self.create_publisher(Float32, "/servo_speed", 10)
        self.estop_publisher = self.create_publisher(Bool, "/emergency_stop", 10)
        self.result_publisher = self.create_publisher(String, "/simulation/test_result", 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._step)

        self.current_degrees = None
        self.initial_degrees = None
        self.freeze_degrees = None
        self.motion_observed = False
        self.stage = "waiting"
        self.stage_started = time.monotonic()
        self.overall_deadline = self.stage_started + 45.0
        self.ik_future = None
        self.trajectory_goal_future = None
        self.trajectory_result_future = None
        self.done = False
        self.exit_code = 1
        self.get_logger().info("Simulation smoke test waiting for servo feedback and MoveIt")

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            seed = self.current_degrees if self.current_degrees is not None else HOME_DEGREES
            self.current_degrees = merge_joint_radians(message.name, message.position, seed)
        except ValueError as exc:
            self._fail(f"invalid /joint_states feedback: {exc}")

    def _step(self) -> None:
        if self.done:
            return
        now = time.monotonic()
        if now > self.overall_deadline:
            self._fail(f"timed out in stage '{self.stage}'")
            return

        if self.stage == "waiting":
            if (
                self.current_degrees is None
                or not self.ik_client.service_is_ready()
                or not self.trajectory_client.server_is_ready()
            ):
                return
            self.initial_degrees = self.current_degrees
            self._publish_speed(100.0)
            self._publish_command(self.FIRST_TARGET)
            self._enter("first_motion")
            return

        if self.stage == "first_motion":
            if self.current_degrees is None:
                return
            moved = max(abs(a - b) for a, b in zip(self.current_degrees, self.initial_degrees))
            remaining = max(abs(a - b) for a, b in zip(self.current_degrees, self.FIRST_TARGET))
            if moved > 0.5 and remaining > 0.5:
                self.motion_observed = True
            if remaining < 0.35:
                if not self.motion_observed:
                    self._fail("target jumped instantly; rate limiting was not observed")
                    return
                self._publish_estop(True)
                self._enter("estop_latching")
            return

        if self.stage == "estop_latching" and now - self.stage_started > 0.35:
            self.freeze_degrees = self.current_degrees
            self._publish_command(self.SECOND_TARGET)
            self._enter("estop_check")
            return

        if self.stage == "estop_check" and now - self.stage_started > 1.0:
            drift = max(abs(a - b) for a, b in zip(self.current_degrees, self.freeze_degrees))
            if drift > 0.1:
                self._fail(f"emergency stop allowed {drift:.3f} degrees of motion")
                return
            self._publish_estop(False)
            self._enter("reset_settling")
            return

        if self.stage == "reset_settling" and now - self.stage_started > 0.35:
            self._publish_command(self.SECOND_TARGET)
            self._enter("second_motion")
            return

        if self.stage == "second_motion":
            remaining = max(abs(a - b) for a, b in zip(self.current_degrees, self.SECOND_TARGET))
            if remaining < 0.35:
                self._request_ik()
            return

        if self.stage == "ik_pending" and self.ik_future.done():
            try:
                response = self.ik_future.result()
            except Exception as exc:
                self._fail(f"/compute_ik call failed: {exc}")
                return
            if response.error_code.val != 1:
                self._fail(f"MoveIt IK returned error code {response.error_code.val}")
                return
            solution_names = set(response.solution.joint_state.name)
            missing = set(JOINT_NAMES[:5]) - solution_names
            if missing:
                self._fail(f"MoveIt IK solution omitted joints: {sorted(missing)}")
                return
            # The action server requires exclusive ownership of motor command
            # topics. Release the smoke test's manual publishers first and wait
            # for DDS graph discovery to reflect that handoff.
            self.destroy_publisher(self.command_publisher)
            self.destroy_publisher(self.speed_publisher)
            self.command_publisher = None
            self.speed_publisher = None
            self._enter("manual_publishers_released")
            return

        if self.stage == "manual_publishers_released":
            if (
                self.count_publishers("/joint_commands") == 1
                and self.count_publishers("/servo_speed") == 1
            ):
                self._request_trajectory()
            return

        if self.stage == "trajectory_goal_pending" and self.trajectory_goal_future.done():
            goal_handle = self.trajectory_goal_future.result()
            if not goal_handle.accepted:
                self._fail("trajectory executor rejected the smoke-test goal")
                return
            self.trajectory_result_future = goal_handle.get_result_async()
            self._enter("trajectory_result_pending")
            return

        if self.stage == "trajectory_result_pending" and self.trajectory_result_future.done():
            result = self.trajectory_result_future.result().result
            if result.error_code != 0:
                self._fail(
                    f"trajectory executor returned {result.error_code}: {result.error_string}"
                )
                return
            self._enter("trajectory_motion")
            return

        if self.stage == "trajectory_motion":
            remaining = max(
                abs(a - b) for a, b in zip(self.current_degrees, self.TRAJECTORY_TARGET)
            )
            if remaining < 0.35:
                self._pass()

    def _request_ik(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform("base_link", TOOL_LINK, Time())
        except TransformException:
            return

        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.ik_link_name = TOOL_LINK
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout.sec = 2
        request.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.robot_state.joint_state.name = list(JOINT_NAMES)
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose.position.x = transform.transform.translation.x
        request.ik_request.pose_stamped.pose.position.y = transform.transform.translation.y
        request.ik_request.pose_stamped.pose.position.z = transform.transform.translation.z
        request.ik_request.pose_stamped.pose.orientation = transform.transform.rotation

        # JointState positions are radians in the same order published by servo_node.
        request.ik_request.robot_state.joint_state.position = list(
            servo_degrees_to_radians(self.current_degrees)
        )
        self.ik_future = self.ik_client.call_async(request)
        self._enter("ik_pending")

    def _request_trajectory(self) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        start = JointTrajectoryPoint()
        start.positions = list(servo_degrees_to_radians(self.current_degrees))
        target = JointTrajectoryPoint()
        target.positions = list(servo_degrees_to_radians(self.TRAJECTORY_TARGET))
        target.time_from_start.sec = 5
        goal.trajectory.points = [start, target]
        self.trajectory_goal_future = self.trajectory_client.send_goal_async(goal)
        self._enter("trajectory_goal_pending")

    def _publish_command(self, target) -> None:
        message = Float32MultiArray()
        message.data = list(target)
        self.command_publisher.publish(message)

    def _publish_speed(self, speed: float) -> None:
        message = Float32()
        message.data = speed
        self.speed_publisher.publish(message)

    def _publish_estop(self, active: bool) -> None:
        message = Bool()
        message.data = active
        self.estop_publisher.publish(message)

    def _enter(self, stage: str) -> None:
        self.stage = stage
        self.stage_started = time.monotonic()
        self.get_logger().info(f"Smoke test stage: {stage}")

    def _pass(self) -> None:
        self.exit_code = 0
        self._finish(
            "PASS: servo motion, E-stop, recovery, TF, MoveIt IK, and trajectory execution are operational"
        )

    def _fail(self, reason: str) -> None:
        if not self.done:
            self.exit_code = 1
            self._finish(f"FAIL: {reason}")

    def _finish(self, result: str) -> None:
        self.done = True
        message = String()
        message.data = result
        self.result_publisher.publish(message)
        if self.exit_code == 0:
            self.get_logger().info(result)
        else:
            self.get_logger().error(result)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulationSmokeTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
