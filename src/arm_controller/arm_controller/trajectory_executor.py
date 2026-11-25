#!/usr/bin/env python3
"""
Trajectory Executor Node
Bridges MoveIt trajectory commands to hardware servo control
Executes multi-waypoint trajectories by feeding individual points to servo_node
"""

import json
import math
import time
import threading

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray, Float32, Bool, String
from sensor_msgs.msg import JointState

from arm_controller.joint_contract import (
    HOME_DEGREES,
    JOINT_NAMES,
    merge_joint_radians,
    servo_degrees_to_radians,
)
from arm_controller.trajectory_safety import (
    TrajectoryValidationError,
    estimate_motion_timeout_seconds,
    max_target_error_degrees,
    validate_hardware_status_payload,
    validate_trajectory_rows,
)


class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')

        # Callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()

        # Joint names matching your arm configuration
        self.joint_names = list(JOINT_NAMES)

        # Current joint positions are intentionally unknown until the simulator
        # or authenticated Pi bridge publishes a complete six-joint state.
        self.current_positions = None
        self.position_lock = threading.Lock()
        self.position_condition = threading.Condition(self.position_lock)
        self.last_joint_state_at = 0.0
        self.joint_state_sample_id = 0

        # Trajectory execution state
        self.executing = False
        self.execution_lock = threading.Lock()
        # Linearization point shared by command publication, action success,
        # cancellation, and E-stop state changes.  A cancel/E-stop that acquires
        # this lock first is guaranteed to prevent the next servo command or a
        # successful terminal action transition.
        self.command_lock = threading.RLock()
        self.should_cancel = False

        # Execution gates.  Hardware mode adds the signed Raspberry Pi status
        # requirement; simulation keeps that gate disabled.
        self.require_hardware_status = bool(
            self.declare_parameter('require_hardware_status', False).value
        )
        self.current_speed = float(
            self.declare_parameter('speed_percent', 10.0).value
        )
        self.feedback_max_age = float(
            self.declare_parameter('feedback_max_age_sec', 0.75).value
        )
        self.status_max_age = float(
            self.declare_parameter('status_max_age_sec', 0.75).value
        )
        self.waypoint_tolerance_deg = float(
            self.declare_parameter('waypoint_tolerance_deg', 0.5).value
        )
        self.goal_tolerance_deg = float(
            self.declare_parameter('goal_tolerance_deg', 0.25).value
        )
        self.start_tolerance_deg = float(
            self.declare_parameter('start_tolerance_deg', 0.5).value
        )
        self.maximum_speed_deg_per_second = float(
            self.declare_parameter(
                'maximum_speed_deg_per_second', 180.0
            ).value
        )
        self.timeout_scale = float(
            self.declare_parameter('motion_timeout_scale', 1.5).value
        )
        self.timeout_margin = float(
            self.declare_parameter('motion_timeout_margin_sec', 1.0).value
        )
        self.maximum_checkpoint_timeout = float(
            self.declare_parameter(
                'maximum_checkpoint_timeout_sec', 30.0
            ).value
        )
        self.maximum_trajectory_horizon = float(
            self.declare_parameter('maximum_trajectory_horizon_sec', 120.0).value
        )
        self.settle_time = float(
            self.declare_parameter('settle_time_sec', 0.15).value
        )
        self.estop_on_fault = bool(
            self.declare_parameter(
                'estop_on_fault', True
            ).value
        )
        if not self.estop_on_fault:
            self.get_logger().warning(
                'estop_on_fault=false is deprecated and ignored; action faults '
                'always stop the servo target'
            )
        self.require_exclusive_command_topics = bool(
            self.declare_parameter(
                'require_exclusive_command_topics', True
            ).value
        )
        numeric_parameters = {
            'speed_percent': (self.current_speed, 1.0, 100.0),
            'feedback_max_age_sec': (self.feedback_max_age, 0.05, 10.0),
            'status_max_age_sec': (self.status_max_age, 0.05, 10.0),
            'waypoint_tolerance_deg': (
                self.waypoint_tolerance_deg, 0.05, 20.0
            ),
            'goal_tolerance_deg': (self.goal_tolerance_deg, 0.05, 20.0),
            'start_tolerance_deg': (self.start_tolerance_deg, 0.05, 5.0),
            'maximum_speed_deg_per_second': (
                self.maximum_speed_deg_per_second, 1.0, 720.0
            ),
            'motion_timeout_scale': (self.timeout_scale, 1.0, 5.0),
            'motion_timeout_margin_sec': (self.timeout_margin, 0.0, 10.0),
            'maximum_checkpoint_timeout_sec': (
                self.maximum_checkpoint_timeout, 1.0, 120.0
            ),
            'maximum_trajectory_horizon_sec': (
                self.maximum_trajectory_horizon, 1.0, 600.0
            ),
            'settle_time_sec': (self.settle_time, 0.0, 5.0),
        }
        for name, (value, minimum, maximum) in numeric_parameters.items():
            if not math.isfinite(value) or not minimum <= value <= maximum:
                raise ValueError(
                    f'{name} must be finite and within {minimum}..{maximum}'
                )

        # Emergency stop state
        self.emergency_stop_active = False
        self.estop_reset_requested = False
        self.hardware_status = None
        self.last_hardware_status_at = 0.0
        self.last_hardware_status_sequence = None
        self.highest_hardware_status_sequence = None
        self.status_lock = threading.Lock()

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/joint_commands',
            10
        )

        self.speed_pub = self.create_publisher(
            Float32,
            '/servo_speed',
            10
        )

        self.estop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )

        self.estop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.estop_callback,
            10,
            callback_group=self.callback_group
        )

        self.status_sub = self.create_subscription(
            String,
            '/raspberry/status',
            self.hardware_status_callback,
            10,
            callback_group=self.callback_group
        )

        # Action server for MoveIt trajectory execution
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_trajectory_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Trajectory Executor initialized')
        self.get_logger().info(
            'Action server: /arm_controller/follow_joint_trajectory'
        )
        self.get_logger().info('Publishing commands to: /joint_commands')
        self.get_logger().info(
            f'Trajectory safety: speed={self.current_speed:.1f}%, '
            f'hardware_status_required={self.require_hardware_status}, '
            f'estop_on_fault={self.estop_on_fault}'
        )

    def joint_state_callback(self, msg):
        """Update current joint positions from feedback"""
        try:
            if tuple(msg.name) != tuple(JOINT_NAMES) or len(msg.position) != len(
                JOINT_NAMES
            ):
                raise ValueError(
                    'expected the complete canonical six-joint state in order'
                )
            positions = list(
                merge_joint_radians(msg.name, msg.position, HOME_DEGREES)
            )
            with self.position_condition:
                self.current_positions = positions
                self.last_joint_state_at = time.monotonic()
                self.joint_state_sample_id += 1
                self.position_condition.notify_all()
        except Exception as e:
            self.get_logger().error(f'Joint state callback error: {e}')

    def hardware_status_callback(self, msg):
        """Track the bridge's authenticated Raspberry Pi acknowledgement."""
        try:
            status = json.loads(msg.data)
            sequence, code, states, _angles = validate_hardware_status_payload(
                status,
            )
        except (json.JSONDecodeError, TrajectoryValidationError) as exc:
            # Reordered UDP ACKs are expected occasionally. They must never
            # replace the most recent authenticated state used by a goal.
            self.get_logger().error(f'Rejected Raspberry Pi status: {exc}')
            return
        with self.status_lock:
            if (
                self.highest_hardware_status_sequence is not None
                and sequence <= self.highest_hardware_status_sequence
            ):
                self.get_logger().debug(
                    f'Ignored old Raspberry Pi status sequence {sequence}'
                )
                return
            self.highest_hardware_status_sequence = sequence
            if code == 'STALE_SEQUENCE':
                self.get_logger().debug(
                    f'Ignored stale-sequence ACK {sequence}'
                )
                return
            self.hardware_status = status
            self.last_hardware_status_at = time.monotonic()
            self.last_hardware_status_sequence = sequence
        with self.command_lock:
            if 'ESTOP_LATCHED' in states:
                self.emergency_stop_active = True
            elif code == 'OK' and self.estop_reset_requested:
                # In hardware mode a ROS reset request is only complete after
                # the authenticated Pi acknowledgement reports the latch clear.
                self.emergency_stop_active = False
                self.estop_reset_requested = False
                self.get_logger().info(
                    'Emergency-stop reset confirmed by Raspberry Pi'
                )

    def estop_callback(self, msg):
        """Handle emergency stop state changes"""
        requested = bool(msg.data)
        with self.command_lock:
            if requested:
                self.emergency_stop_active = True
                self.estop_reset_requested = False
            elif self.require_hardware_status:
                # Keep motion gated until the bridge publishes the authenticated
                # acknowledgement of its RESET_ESTOP packet.
                self.emergency_stop_active = True
                self.estop_reset_requested = True
            else:
                self.emergency_stop_active = False
                self.estop_reset_requested = False
        if requested:
            self.get_logger().warning('Emergency stop ACTIVE - Stopping trajectory')
        else:
            self.get_logger().warning(
                'Emergency-stop reset requested; waiting for confirmation'
                if self.require_hardware_status
                else 'Emergency stop reset'
            )

    def goal_callback(self, goal_request):
        """Accept or reject trajectory goals"""
        gate_reason = self._execution_gate_reason()
        if gate_reason:
            self.get_logger().warning(
                f'Trajectory goal rejected - {gate_reason}'
            )
            return GoalResponse.REJECT

        try:
            self._validate_goal_contract(goal_request)
            with self.position_lock:
                current = tuple(self.current_positions)
            targets = self._validated_targets(goal_request.trajectory, current)
            start_error = max_target_error_degrees(
                current,
                targets[0],
                goal_request.trajectory.joint_names,
            )
            if start_error > self.start_tolerance_deg:
                raise TrajectoryValidationError(
                    f'trajectory starts {start_error:.3f} degrees away from '
                    'the current state'
                )
        except (TrajectoryValidationError, TypeError) as exc:
            self.get_logger().error(f'Trajectory goal rejected - {exc}')
            return GoalResponse.REJECT

        # Reserve the single command stream before returning ACCEPT.  This is
        # inside the goal callback so two reentrant callbacks cannot both pass.
        with self.execution_lock:
            if self.executing:
                self.get_logger().warning(
                    'Trajectory goal rejected - Already executing'
                )
                return GoalResponse.REJECT
            self.executing = True
            with self.command_lock:
                self.should_cancel = False

        self.get_logger().info(
            f'Trajectory goal accepted: '
            f'{len(goal_request.trajectory.points)} waypoints'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle trajectory cancellation requests"""
        self.get_logger().warning('Trajectory cancellation requested')
        # Cancellation is a physical stop operation.  Publish it immediately;
        # waiting for the execute loop to wake leaves the prior servo target
        # active and permits a command-publication race.
        self._latch_emergency_stop('trajectory cancellation requested')
        return CancelResponse.ACCEPT

    async def execute_trajectory_callback(self, goal_handle: ServerGoalHandle):
        """Execute a validated trajectory and wait for applied-state feedback."""
        self.get_logger().info('Starting trajectory execution')

        try:
            trajectory = goal_handle.request.trajectory
            stop_reason = self._runtime_stop_reason(goal_handle)
            if stop_reason:
                return self._finish_wait_failure(
                    goal_handle,
                    f'Execution gate failed: {stop_reason}',
                    FollowJointTrajectory.Result.INVALID_GOAL,
                )

            with self.position_lock:
                current = tuple(self.current_positions)
            try:
                targets = self._validated_targets(trajectory, current)
                start_error = max_target_error_degrees(
                    current, targets[0], trajectory.joint_names
                )
                if start_error > self.start_tolerance_deg:
                    raise TrajectoryValidationError(
                        f'start state changed by {start_error:.3f} degrees '
                        'after goal acceptance'
                    )
            except TrajectoryValidationError as exc:
                return self._finish_with_fault(
                    goal_handle,
                    f'Invalid trajectory: {exc}',
                    error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                )

            speed_msg = Float32()
            speed_msg.data = self.current_speed
            self.speed_pub.publish(speed_msg)

            feedback_msg = FollowJointTrajectory.Feedback()
            started_at = time.monotonic()
            self.get_logger().info(
                'Executing conservative sequential checkpoints; trajectory '
                'timestamps are validation horizons, not open-loop send times'
            )

            for index, (point, joint_commands) in enumerate(
                zip(trajectory.points, targets)
            ):
                stop_reason = self._runtime_stop_reason(goal_handle)
                if stop_reason:
                    return self._finish_wait_failure(
                        goal_handle,
                        stop_reason,
                        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                    )
                with self.position_lock:
                    command_start = tuple(self.current_positions)
                    minimum_joint_sample_id = self.joint_state_sample_id
                with self.status_lock:
                    minimum_status_sequence = self.last_hardware_status_sequence
                try:
                    timeout = estimate_motion_timeout_seconds(
                        command_start,
                        joint_commands,
                        trajectory.joint_names,
                        speed_percent=self.current_speed,
                        maximum_speed_deg_per_second=(
                            self.maximum_speed_deg_per_second
                        ),
                        scale=self.timeout_scale,
                        margin_s=self.timeout_margin,
                        maximum_timeout_s=self.maximum_checkpoint_timeout,
                    )
                except ValueError as exc:
                    return self._finish_with_fault(
                        goal_handle,
                        f'Checkpoint timeout rejected: {exc}',
                        FollowJointTrajectory.Result.INVALID_GOAL,
                    )
                if index == len(targets) - 1:
                    timeout += self._goal_time_tolerance_seconds(
                        goal_handle.request
                    )

                cmd_msg = Float32MultiArray()
                cmd_msg.data = list(joint_commands)
                with self.command_lock:
                    stop_reason = self._runtime_stop_reason(goal_handle)
                    if not stop_reason:
                        self.joint_cmd_pub.publish(cmd_msg)
                if stop_reason:
                    return self._finish_wait_failure(
                        goal_handle,
                        stop_reason,
                        FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                    )
                final_checkpoint = index == len(targets) - 1
                reached, reason = self._wait_for_target(
                    goal_handle,
                    joint_commands,
                    trajectory.joint_names,
                    (
                        self.goal_tolerance_deg
                        if final_checkpoint
                        else self.waypoint_tolerance_deg
                    ),
                    time.monotonic() + timeout,
                    settle_time=self.settle_time if final_checkpoint else 0.0,
                    minimum_joint_sample_id=minimum_joint_sample_id,
                    minimum_status_sequence=minimum_status_sequence,
                )
                if not reached:
                    return self._finish_wait_failure(
                        goal_handle,
                        reason,
                        (
                            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                            if final_checkpoint
                            else FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        ),
                    )
                self._publish_action_feedback(
                    goal_handle,
                    feedback_msg,
                    trajectory,
                    point,
                    started_at,
                )
                self.get_logger().debug(
                    f'Acknowledged checkpoint {index + 1}/'
                    f'{len(trajectory.points)}'
                )

            with self.command_lock:
                stop_reason = self._runtime_stop_reason(goal_handle)
                if not stop_reason:
                    self.get_logger().info(
                        'Trajectory execution completed with acknowledged final state'
                    )
                    goal_handle.succeed()
            if stop_reason:
                return self._finish_wait_failure(
                    goal_handle,
                    stop_reason,
                    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                )
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.SUCCESSFUL,
                error_string='Trajectory executed and final state acknowledged'
            )

        except Exception as e:
            self.get_logger().error(f'Trajectory execution failed: {str(e)}')
            return self._finish_with_fault(
                goal_handle,
                f'Execution error: {str(e)}',
                error_code=FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
            )

        finally:
            with self.execution_lock:
                self.executing = False
                with self.command_lock:
                    self.should_cancel = False

    @staticmethod
    def _point_time_seconds(point: JointTrajectoryPoint) -> float:
        return (
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) / 1e9
        )

    def _validate_goal_contract(self, goal_request):
        stamp = goal_request.trajectory.header.stamp
        if stamp.sec != 0 or stamp.nanosec != 0:
            raise TrajectoryValidationError(
                'nonzero trajectory header timestamps are unsupported'
            )
        if (
            goal_request.path_tolerance
            or goal_request.goal_tolerance
            or getattr(goal_request, 'component_path_tolerance', ())
            or getattr(goal_request, 'component_goal_tolerance', ())
        ):
            raise TrajectoryValidationError(
                'per-joint and per-component action tolerances are unsupported; use the '
                'audited executor parameters'
            )
        multi_dof = getattr(goal_request, 'multi_dof_trajectory', None)
        if multi_dof is not None and (
            multi_dof.joint_names or multi_dof.points
        ):
            raise TrajectoryValidationError(
                'multi-DOF trajectories are unsupported'
            )
        self._goal_time_tolerance_seconds(goal_request)

    @staticmethod
    def _goal_time_tolerance_seconds(goal_request):
        duration = goal_request.goal_time_tolerance
        seconds = float(duration.sec) + float(duration.nanosec) / 1e9
        if not math.isfinite(seconds) or not 0.0 <= seconds <= 10.0:
            raise TrajectoryValidationError(
                'goal_time_tolerance must be within 0..10 seconds'
            )
        return seconds

    def _validated_targets(self, trajectory, current):
        rows = [tuple(point.positions) for point in trajectory.points]
        times = [self._point_time_seconds(point) for point in trajectory.points]
        for point_index, point in enumerate(trajectory.points):
            for field_name in ('velocities', 'accelerations'):
                values = tuple(getattr(point, field_name))
                if values and len(values) != len(trajectory.joint_names):
                    raise TrajectoryValidationError(
                        f'point {point_index} {field_name} count does not match joints'
                    )
                if any(not math.isfinite(float(value)) for value in values):
                    raise TrajectoryValidationError(
                        f'point {point_index} contains non-finite {field_name}'
                    )
        return validate_trajectory_rows(
            trajectory.joint_names,
            rows,
            times,
            current,
            maximum_horizon_s=self.maximum_trajectory_horizon,
        )

    def _execution_gate_reason(self):
        with self.command_lock:
            if self.emergency_stop_active:
                return 'emergency stop is active'
        if self.require_exclusive_command_topics:
            command_publishers = self.count_publishers('/joint_commands')
            speed_publishers = self.count_publishers('/servo_speed')
            if command_publishers != 1 or speed_publishers != 1:
                return (
                    'trajectory executor does not exclusively own the command '
                    f'topics (joint={command_publishers}, speed={speed_publishers})'
                )
        now = time.monotonic()
        with self.position_lock:
            if self.current_positions is None:
                return 'no complete joint-state feedback has been received'
            if now - self.last_joint_state_at > self.feedback_max_age:
                return 'joint-state feedback is stale'
        if self.require_hardware_status:
            with self.status_lock:
                status = self.hardware_status
                status_age = now - self.last_hardware_status_at
            if status is None:
                return 'no authenticated Raspberry Pi status has been received'
            if status_age > self.status_max_age:
                return 'authenticated Raspberry Pi status is stale'
            if status.get('code') != 'OK':
                return f"Raspberry Pi acknowledgement is {status.get('code')!r}"
            states = set(status.get('state', ()))
            if 'HARDWARE_READY' not in states:
                return 'Raspberry Pi does not report HARDWARE_READY'
            if 'ESTOP_LATCHED' in states:
                return 'Raspberry Pi E-stop is latched'
        return None

    def _wait_for_target(
        self,
        goal_handle,
        target,
        controlled_joint_names,
        tolerance_deg,
        deadline,
        settle_time,
        minimum_joint_sample_id,
        minimum_status_sequence,
    ):
        within_since = None
        within_sample_id = None
        while time.monotonic() <= deadline:
            reason = self._runtime_stop_reason(goal_handle)
            if reason:
                return False, reason
            with self.position_lock:
                current = tuple(self.current_positions)
                sample_id = self.joint_state_sample_id
            if sample_id <= minimum_joint_sample_id:
                with self.position_condition:
                    self.position_condition.wait(timeout=0.02)
                continue
            if self.require_hardware_status:
                with self.status_lock:
                    status_sequence = self.last_hardware_status_sequence
                if (
                    status_sequence is None
                    or minimum_status_sequence is None
                    or status_sequence <= minimum_status_sequence
                ):
                    with self.position_condition:
                        self.position_condition.wait(timeout=0.02)
                    continue
            error = max_target_error_degrees(
                current, target, controlled_joint_names
            )
            now = time.monotonic()
            if error <= tolerance_deg:
                if within_since is None:
                    within_since = now
                    within_sample_id = sample_id
                if (
                    now - within_since >= settle_time
                    and (
                        settle_time == 0.0
                        or sample_id > within_sample_id
                    )
                ):
                    return True, None
            else:
                within_since = None
                within_sample_id = None
            with self.position_condition:
                self.position_condition.wait(timeout=0.02)
        return False, (
            f'applied-command estimate did not reach the target within '
            f'{tolerance_deg:.2f} degrees'
        )

    def _runtime_stop_reason(self, goal_handle):
        with self.command_lock:
            if goal_handle.is_cancel_requested or self.should_cancel:
                return 'trajectory cancellation requested'
            if self.emergency_stop_active:
                return 'emergency stop became active'
        return self._execution_gate_reason()

    def _publish_action_feedback(
        self, goal_handle, feedback_msg, trajectory, point, started_at
    ):
        feedback_msg.header.stamp = self.get_clock().now().to_msg()
        feedback_msg.joint_names = list(trajectory.joint_names)
        with self.position_lock:
            current_radians = servo_degrees_to_radians(self.current_positions)
        indexes = {name: index for index, name in enumerate(self.joint_names)}
        feedback_msg.actual.positions = [
            current_radians[indexes[name]] for name in trajectory.joint_names
        ]
        feedback_msg.desired.positions = list(point.positions)
        feedback_msg.error.positions = [
            desired - actual
            for actual, desired in zip(
                feedback_msg.actual.positions, point.positions
            )
        ]
        elapsed = max(0.0, time.monotonic() - started_at)
        seconds = int(elapsed)
        feedback_msg.actual.time_from_start = Duration(
            sec=seconds,
            nanosec=int((elapsed - seconds) * 1e9),
        )
        feedback_msg.desired.time_from_start = point.time_from_start
        goal_handle.publish_feedback(feedback_msg)

    def _finish_wait_failure(self, goal_handle, reason, error_code):
        with self.command_lock:
            canceled = (
                goal_handle.is_cancel_requested
                or self.should_cancel
                or reason == 'trajectory cancellation requested'
            )
        if canceled:
            # Cancel is a physical stop request, not merely an action-state
            # change.  Always latch E-stop so the previous servo target cannot
            # continue moving after CANCELED.
            self._latch_emergency_stop(reason)
            goal_handle.canceled()
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.SUCCESSFUL,
                error_string=reason,
            )
        return self._finish_with_fault(goal_handle, reason, error_code)

    def _finish_with_fault(self, goal_handle, reason, error_code):
        self.get_logger().error(reason)
        self._latch_emergency_stop(reason)
        goal_handle.abort()
        return FollowJointTrajectory.Result(
            error_code=error_code,
            error_string=reason,
        )

    def _latch_emergency_stop(self, reason):
        with self.command_lock:
            self.get_logger().error(f'Latching emergency stop: {reason}')
            self.emergency_stop_active = True
            self.estop_reset_requested = False
            self.should_cancel = True
            message = Bool()
            message.data = True
            self.estop_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)

    # The action callback deliberately blocks while feedback/ACK callbacks must
    # keep running.  Provision enough workers explicitly instead of relying on
    # CPU-affinity-dependent automatic sizing.
    executor = MultiThreadedExecutor(num_threads=4)
    node = TrajectoryExecutor()

    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
