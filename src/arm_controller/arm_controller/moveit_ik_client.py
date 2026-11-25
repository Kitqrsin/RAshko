#!/usr/bin/env python3
"""
MoveIt IK Client for Robotic Arm
Uses MoveIt's compute_ik service for accurate inverse kinematics
"""

import math
import time
from typing import Optional, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException

from arm_controller.joint_contract import (
    HOME_DEGREES,
    JOINT_NAMES as CONTRACT_JOINT_NAMES,
    SERVO_MAX_DEGREES,
    radians_to_servo_degrees,
    servo_degrees_to_radians,
)
from arm_controller.tool_pose import ARM_LINKS, TOOL_LINK, tool_elevation, tool_quaternion


class MoveItIKClient:
    """
    MoveIt IK Client that provides inverse and forward kinematics
    using MoveIt's KDL solver configured for the arm.
    """
    
    # Joint names in the arm group (from SRDF)
    JOINT_NAMES = list(CONTRACT_JOINT_NAMES[:5])
    
    # Planning group name (from SRDF)
    PLANNING_GROUP = 'arm'
    
    # End effector link (tip of the kinematic chain)
    END_EFFECTOR_LINK = TOOL_LINK
    
    def __init__(self, node: Node):
        """
        Initialize the MoveIt IK client.
        
        Args:
            node: The ROS2 node to use for service clients and TF
        """
        self.node = node
        self.logger = node.get_logger()
        
        # Create callback group for service calls
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create IK service client
        self.ik_client = node.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self.callback_group
        )
        
        # TF buffer for forward kinematics via transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        
        # Calibrated logical-servo home (maps to URDF zero radians).
        self.home_position = {
            'base': HOME_DEGREES[0],
            'shoulder': HOME_DEGREES[1],
            'elbow': HOME_DEGREES[2],
            'wrist': HOME_DEGREES[3],
            'wrist_rotate': HOME_DEGREES[4],
            'gripper': HOME_DEGREES[5],
        }
        
        # Track if MoveIt is available
        self._moveit_available = False
        self._check_moveit_availability()
    
    def _check_moveit_availability(self):
        """Check if MoveIt's compute_ik service is available."""
        self._moveit_available = self.ik_client.wait_for_service(timeout_sec=2.0)
        if self._moveit_available:
            self.logger.info("MoveIt IK service is available!")
        else:
            self.logger.warning("MoveIt IK service NOT available. Is MoveIt running?")
    
    def is_available(self) -> bool:
        """Check if MoveIt IK service is available."""
        self._moveit_available = self.ik_client.service_is_ready()
        return self._moveit_available
    
    def wait_for_service(self, timeout_sec: float = 5.0) -> bool:
        """Wait for MoveIt IK service to become available."""
        self._moveit_available = self.ik_client.wait_for_service(timeout_sec=timeout_sec)
        return self._moveit_available
    
    def solve_ik(
        self,
        x: float,
        y: float, 
        z: float,
        pitch: float = 0.0,
        roll: float = 0.0,
        gripper_angle: Optional[float] = None,
        seed_state: Optional[Dict[str, float]] = None
    ) -> Optional[Dict[str, float]]:
        """
        Solve inverse kinematics for a target position.
        
        Args:
            x, y, z: Target position in meters (world frame)
            pitch: Physical tool elevation in radians (+pi/2 up, 0 horizontal,
                -pi/2 down)
            roll: Rotation about the physical tool direction in radians
            gripper_angle: Gripper opening angle (degrees), None to keep current
            seed_state: Optional seed joint state for IK solver (degrees)
            
        Returns:
            Dict of joint angles in logical servo degrees,
            or None if IK fails
        """
        if not self.is_available():
            self.logger.error("MoveIt IK service not available!")
            return None
        
        # Build the IK request
        request = GetPositionIK.Request()
        
        # Set the planning group
        request.ik_request.group_name = self.PLANNING_GROUP
        
        # Build target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.header.stamp = self.node.get_clock().now().to_msg()
        
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        
        azimuth = math.atan2(y, x)
        quat = tool_quaternion(azimuth, pitch, roll)
        target_pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        
        request.ik_request.pose_stamped = target_pose
        
        # Set the tip link
        request.ik_request.ik_link_name = self.END_EFFECTOR_LINK
        
        # ALWAYS set seed state - MoveIt needs this for proper IK solving
        robot_state = RobotState()
        robot_state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        robot_state.joint_state.name = self.JOINT_NAMES
        
        # Convert from calibrated logical servo degrees to URDF radians.
        # Use provided seed_state or default to home position
        seed_degrees = [
            (seed_state or {}).get(name, self.home_position[name])
            for name in self.JOINT_NAMES
        ]
        seed_degrees.append(
            gripper_angle if gripper_angle is not None else HOME_DEGREES[5]
        )
        positions = list(servo_degrees_to_radians(seed_degrees)[:5])
        
        robot_state.joint_state.position = positions
        request.ik_request.robot_state = robot_state
        
        # IK solver attempts - increase timeout for better solutions
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = int(0.5 * 1e9)  # 500ms timeout
        
        try:
            # Call the service synchronously
            future = self.ik_client.call_async(request)
            
            # The GUI already spins this node on its ROS thread. Spinning the
            # same node here from Tk's thread can corrupt the executor state.
            deadline = time.monotonic() + 1.0
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)

            if not future.done():
                self.logger.error("IK service call timed out after 1.0 second")
                return None
            
            if future.result() is None:
                self.logger.error("IK service call failed (no result)")
                return None
            
            response = future.result()
            
            if response.error_code.val != response.error_code.SUCCESS:
                self.logger.warning(f"IK failed with error code: {response.error_code.val}")
                return None
            
            # Extract joint positions
            joint_state = response.solution.joint_state
            
            # Build result dictionary with proper GUI joint names
            # GUI uses: base, shoulder, elbow, forearm (=URDF wrist), wrist (=URDF wrist_rotate), gripper
            result = {}
            for i, name in enumerate(joint_state.name):
                if name in self.JOINT_NAMES:
                    rad_value = joint_state.position[i]
                    joint_index = CONTRACT_JOINT_NAMES.index(name)
                    deg_value = radians_to_servo_degrees(joint_index, rad_value)
                    deg_value = max(
                        0.0,
                        min(SERVO_MAX_DEGREES[joint_index], deg_value),
                    )
                    
                    # Map URDF joint names to GUI joint names
                    # URDF 'wrist' -> GUI 'forearm' (index 3)
                    # URDF 'wrist_rotate' -> GUI 'wrist' (index 4)
                    if name == 'wrist':
                        result['forearm'] = deg_value
                    elif name == 'wrist_rotate':
                        result['wrist'] = deg_value
                    else:
                        result[name] = deg_value
            
            # Add gripper (not controlled by MoveIt)
            result['gripper'] = (
                gripper_angle if gripper_angle is not None else HOME_DEGREES[5]
            )
            
            self.logger.info(f"IK solution found: {result}")
            return result
            
        except Exception as e:
            self.logger.error(f"IK service call exception: {e}")
            return None
    
    def forward_kinematics(
        self,
        base: float,
        shoulder: float,
        elbow: float,
        forearm: float,
        wrist: float
    ) -> Tuple[float, float, float, float, float]:
        """
        Calculate end effector position from joint angles using TF.
        
        Args:
            Joint angles in calibrated logical servo degrees
            
        Returns:
            Tuple of (x, y, z, pitch, roll) position and orientation
        """
        try:
            # Try to get transform from TF
            trans = self.tf_buffer.lookup_transform(
                'base_link',
                self.END_EFFECTOR_LINK,
                rclpy.time.Time()
            )
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            
            # Report the physical gripper elevation, not the CAD link's Euler pitch.
            quat = trans.transform.rotation
            pitch = tool_elevation((quat.x, quat.y, quat.z, quat.w))
            roll = 0.0
            
            return (x, y, z, pitch, roll)
            
        except TransformException as e:
            self.logger.debug(f"TF lookup failed: {e}, using fallback FK")
            # Fallback to simple geometric FK
            return self._fallback_forward_kinematics(base, shoulder, elbow, forearm, wrist)

    def link_positions(self) -> Optional[list[tuple[float, float, float]]]:
        """Return exact URDF link origins in ``base_link`` coordinates."""
        positions = []
        try:
            for link_name in ARM_LINKS:
                if link_name == "base_link":
                    positions.append((0.0, 0.0, 0.0))
                    continue
                transform = self.tf_buffer.lookup_transform(
                    "base_link", link_name, rclpy.time.Time()
                )
                translation = transform.transform.translation
                positions.append((translation.x, translation.y, translation.z))
        except TransformException:
            return None
        return positions
    
    def _fallback_forward_kinematics(
        self,
        base: float,
        shoulder: float,
        elbow: float,
        forearm: float,
        wrist: float
    ) -> Tuple[float, float, float, float, float]:
        """
        Fallback forward kinematics using geometric model with URDF link lengths.
        Used when TF is not available.
        """
        # Link lengths from URDF measurements (in meters)
        L_base = 0.059    # Base to shoulder height
        L_upper = 0.036   # Shoulder to elbow
        L_lower = 0.084   # Elbow to wrist
        L_wrist = 0.086   # Wrist to gripper
        
        # Convert through the canonical calibrated mapping.
        base_rad, shoulder_rad, elbow_rad, forearm_rad, wrist_rad = (
            servo_degrees_to_radians(
                (base, shoulder, elbow, forearm, wrist, HOME_DEGREES[5])
            )[:5]
        )
        
        # Calculate positions using vertical-based system (matching Tkinter)
        elbow_x = L_upper * math.sin(shoulder_rad)
        elbow_z = L_base + L_upper * math.cos(shoulder_rad)
        
        arm_angle = shoulder_rad + elbow_rad
        wrist_x = elbow_x + L_lower * math.sin(arm_angle)
        wrist_z = elbow_z + L_lower * math.cos(arm_angle)
        
        pitch = arm_angle + forearm_rad
        end_x = wrist_x + L_wrist * math.sin(pitch)
        end_z = wrist_z + L_wrist * math.cos(pitch)
        
        x = end_x * math.cos(base_rad)
        y = end_x * math.sin(base_rad)
        z = end_z
        
        return (x, y, z, pitch, wrist_rad)
    
    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert Euler angles (radians) to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q
    
    def _quaternion_to_pitch_roll(self, q: Quaternion) -> Tuple[float, float]:
        """Extract pitch and roll from quaternion."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        return (pitch, roll)
