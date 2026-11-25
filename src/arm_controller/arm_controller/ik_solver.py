#!/usr/bin/env python3
"""
Simple Inverse Kinematics Solver for 6-DOF Robotic Arm
Uses geometric approach for position-based control
"""

import numpy as np
import math

from arm_controller.joint_contract import (
    HOME_DEGREES,
    SERVO_MAX_DEGREES,
    SERVO_MIN_DEGREES,
    radians_to_servo_degrees,
    servo_degrees_to_radians,
)


class ArmIKSolver:
    """
    Inverse kinematics solver for the 6-DOF arm
    Assumes arm configuration: Base -> Shoulder -> Elbow -> Forearm -> Wrist -> Gripper
    """
    
    def __init__(self):
        # Link lengths from URDF joint origin xyz values (in meters)
        # Measured from robot_fixed.urdf - MUST MATCH arm_3d_visualization.py
        self.L1 = 0.059   # Base to shoulder height (base joint z=0.0592971)
        self.L2 = 0.084   # Shoulder to elbow (elbow joint y=0.0841198) - main upper arm!
        self.L3 = 0.086   # Elbow to wrist (wrist joint dist ~0.086)
        self.L4 = 0.061   # Wrist to gripper (wrist_rotate joint dist ~0.061)
        
        # Home position (neutral/straight forward)
        self.home_position = {
            'base': HOME_DEGREES[0],
            'shoulder': HOME_DEGREES[1],
            'elbow': HOME_DEGREES[2],
            'forearm': HOME_DEGREES[3],
            'wrist': HOME_DEGREES[4],
            'gripper': HOME_DEGREES[5],
        }
        
        # Cache for previous solution to speed up solution selection
        self.previous_solution = None
    
    def solve_ik_to_point(self, x, y, z, roll=0, gripper_angle=None, seed_state=None):
        """Solve IK so the gripper tip lands exactly on (x, y, z).

        Unlike solve_ik() which requires the caller to supply a pitch angle,
        this method searches over a range of pitch values and picks the one
        whose FK-verified gripper position is closest to the target.

        It prefers solutions that keep joints near their current position
        (smooth motion) and penalises extreme pitch values.

        Returns:
            dict of joint angles in degrees, or None if unreachable.
        """
        L_base = self.L1
        r_xy = math.sqrt(x**2 + y**2)
        z_above = z - L_base

        # Build candidate pitch angles over the full circle.  The old 0..160°
        # sweep rejected valid poses whenever the wrist needed to point behind
        # the shoulder plane.
        pitch_candidates = []

        # Start with an initial geometric guess (angle from shoulder to target)
        if r_xy > 0.001 or abs(z_above) > 0.001:
            guess = math.atan2(r_xy, z_above)
            guess = max(-math.pi, min(math.pi, guess))
        else:
            guess = 0.0

        # Dense sweep around the guess, plus a broad sweep for coverage
        for offset in [i * 0.02 for i in range(-30, 31)]:  # ±0.6 rad around guess
            p = guess + offset
            if -math.pi <= p <= math.pi:
                pitch_candidates.append(p)

        # Add broad sweep to not miss solutions far from the guess
        for deg in range(-180, 181, 5):
            pitch_candidates.append(math.radians(deg))

        # Remove duplicates and sort
        pitch_candidates = sorted(set(round(p, 4) for p in pitch_candidates))

        candidates = []

        for pitch in pitch_candidates:
            sol = self.solve_ik(x, y, z, pitch=pitch, roll=roll,
                                gripper_angle=gripper_angle, seed_state=seed_state)
            if sol is None:
                continue

            # Verify with FK: where does the gripper actually end up?
            fk = self.forward_kinematics(
                sol['base'], sol['shoulder'], sol['elbow'],
                sol['forearm'], sol['wrist']
            )
            fk_x, fk_y, fk_z = fk[0], fk[1], fk[2]
            pos_err = math.sqrt((fk_x - x)**2 + (fk_y - y)**2 + (fk_z - z)**2)

            # Penalise extreme pitch (prefer more natural poses)
            pitch_penalty = 0.0002 * abs(pitch - math.radians(45))

            # Penalise large jumps from current joint positions (smoothness)
            jump_penalty = 0.0
            if seed_state:
                for jname in ('shoulder', 'elbow', 'forearm'):
                    if jname in seed_state:
                        jump_penalty += 0.00005 * abs(sol[jname] - seed_state[jname])

            posture_score = pitch_penalty + jump_penalty
            candidates.append((pos_err, posture_score, sol))

        # Reachability is determined only by the FK endpoint error.  Pitch and
        # continuity are preferences, not extra constraints.  Among solutions
        # within 1 mm of the most accurate one, choose the smoothest posture.
        accurate = [candidate for candidate in candidates if candidate[0] < 0.015]
        if accurate:
            minimum_error = min(candidate[0] for candidate in accurate)
            equally_accurate = [
                candidate
                for candidate in accurate
                if candidate[0] <= minimum_error + 0.001
            ]
            _position_error, _posture_score, best_solution = min(
                equally_accurate,
                key=lambda candidate: candidate[1],
            )
            self.previous_solution = best_solution.copy()
            return best_solution

        # Never report a nearby point as success.  That old fallback is what
        # made the green gripper stop separately from the orange target.
        return None

    def solve_ik(self, x, y, z, pitch=0, roll=0, gripper_angle=None, seed_state=None):
        """Solve one geometric XYZ/pitch target within physical servo limits."""
        if seed_state is None:
            seed_state = self.previous_solution or self.home_position

        L_base = self.L1
        L_upper = self.L2
        L_forearm = self.L3
        L_wrist = self.L4

        # A Cartesian point has two equivalent planar representations:
        #   yaw toward the point with +radius, or yaw away with -radius.
        # The second branch is essential when the arm folds behind its base;
        # rejecting it discarded reachable poses purely because of base limits.
        r_xy = math.hypot(x, y)
        base_seed = float(seed_state.get('base', HOME_DEGREES[0]))
        base_branches = []
        if r_xy < 0.001:
            base_branches.append((
                max(
                    SERVO_MIN_DEGREES[0],
                    min(SERVO_MAX_DEGREES[0], base_seed),
                ),
                0.0,
            ))
        else:
            target_yaw = math.atan2(y, x)
            for yaw, signed_radius in (
                (target_yaw, r_xy),
                (target_yaw + math.pi, -r_xy),
            ):
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                raw_degrees = radians_to_servo_degrees(0, yaw)
                logical_options = (
                    raw_degrees,
                    raw_degrees + 360.0,
                    raw_degrees - 360.0,
                )
                valid_options = [
                    value
                    for value in logical_options
                    if (
                        SERVO_MIN_DEGREES[0] - 1e-9
                        <= value
                        <= SERVO_MAX_DEGREES[0] + 1e-9
                    )
                ]
                if not valid_options:
                    continue
                base_degrees = min(
                    valid_options,
                    key=lambda value: abs(value - base_seed),
                )
                branch = (base_degrees, signed_radius)
                if branch not in base_branches:
                    base_branches.append(branch)

        if not base_branches:
            return None

        wrist_z = (z - L_base) - L_wrist * math.cos(pitch)
        best_config = None
        best_score = (float('inf'), float('inf'))

        for base_degrees, signed_radius in base_branches:
            wrist_r = signed_radius - L_wrist * math.sin(pitch)
            d = math.hypot(wrist_r, wrist_z)
            if (
                d > L_upper + L_forearm + 1e-9
                or d < abs(L_upper - L_forearm) - 1e-9
                or d < 0.001
            ):
                continue

            cos_elbow = (
                d**2 - L_upper**2 - L_forearm**2
            ) / (2.0 * L_upper * L_forearm)
            elbow_magnitude = math.acos(float(np.clip(cos_elbow, -1.0, 1.0)))
            gamma = math.atan2(wrist_r, wrist_z)

            for elbow_rad in (elbow_magnitude, -elbow_magnitude):
                shoulder_rad = gamma - math.atan2(
                    L_forearm * math.sin(elbow_rad),
                    L_upper + L_forearm * math.cos(elbow_rad),
                )
                arm_angle = shoulder_rad + elbow_rad
                candidate = {
                    'base': base_degrees,
                    'shoulder': radians_to_servo_degrees(1, shoulder_rad),
                    'elbow': radians_to_servo_degrees(2, elbow_rad),
                    'forearm': radians_to_servo_degrees(
                        3,
                        pitch - arm_angle,
                    ),
                    'wrist': radians_to_servo_degrees(4, roll),
                    'gripper': (
                        gripper_angle
                        if gripper_angle is not None
                        else HOME_DEGREES[5]
                    ),
                }

                clamp_error = 0.0
                movement = 0.0
                for index, joint_name in enumerate(
                    ('base', 'shoulder', 'elbow', 'forearm', 'wrist')
                ):
                    value = candidate[joint_name]
                    low = SERVO_MIN_DEGREES[index]
                    high = SERVO_MAX_DEGREES[index]
                    if value < low:
                        clamp_error += low - value
                    elif value > high:
                        clamp_error += value - high
                    movement += abs(
                        value - float(seed_state.get(joint_name, value))
                    )

                score = (clamp_error, movement)
                if score < best_score:
                    best_score = score
                    best_config = candidate

        if best_config is None:
            return None

        joint_angles = self._clamp_angles(best_config)

        # Cache solution for next iteration (faster incremental moves)
        self.previous_solution = joint_angles.copy()

        return joint_angles
    
    def _clamp_angles(self, angles):
        """Clamp joint angles to safe servo ranges"""
        joint_order = ('base', 'shoulder', 'elbow', 'forearm', 'wrist', 'gripper')
        limits = {
            name: (SERVO_MIN_DEGREES[index], SERVO_MAX_DEGREES[index])
            for index, name in enumerate(joint_order)
        }
        
        clamped = {}
        for joint, angle in angles.items():
            min_angle, max_angle = limits.get(joint, (0, 180))
            clamped[joint] = max(min_angle, min(max_angle, angle))
        
        return clamped
    
    def forward_kinematics(self, base, shoulder, elbow, forearm, wrist):
        """
        Calculate end effector position from joint angles.
        MUST match arm_3d_visualization.calculate_arm_positions() exactly!
        Uses the canonical calibrated logical-degree/URDF-radian mapping.
        
        Args:
            Joint angles in logical servo degrees
            
        Returns:
            tuple: (x, y, z, pitch, roll) position and orientation
        """
        # Link lengths from URDF - MUST MATCH arm_3d_visualization.py EXACTLY
        L_base = self.L1
        L_upper = self.L2
        L_forearm = self.L3
        L_wrist = self.L4

        base_rad, shoulder_rad, elbow_rad, forearm_rad, wrist_rad = (
            servo_degrees_to_radians(
                (base, shoulder, elbow, forearm, wrist, HOME_DEGREES[5])
            )[:5]
        )
        
        # Calculate positions using SAME math as arm_3d_visualization.py
        # Elbow joint - uses sin/cos with vertical reference
        elbow_x = L_upper * math.sin(shoulder_rad)
        elbow_z = L_base + L_upper * math.cos(shoulder_rad)
        
        # Wrist joint - cumulative angle
        arm_angle = shoulder_rad + elbow_rad
        wrist_x = elbow_x + L_forearm * math.sin(arm_angle)
        wrist_z = elbow_z + L_forearm * math.cos(arm_angle)
        
        # Gripper position - includes forearm pitch
        full_angle = arm_angle + forearm_rad
        gripper_x = wrist_x + L_wrist * math.sin(full_angle)
        gripper_z = wrist_z + L_wrist * math.cos(full_angle)
        
        # Apply base rotation for 3D position
        x = gripper_x * math.cos(base_rad)
        y = gripper_x * math.sin(base_rad)
        z = gripper_z
        
        pitch = full_angle
        
        return (x, y, z, pitch, wrist_rad)


class PredefinedPoses:
    """Library of predefined poses in Cartesian space"""
    
    @staticmethod
    def get_home():
        """Neutral position - arm straight forward"""
        return {
            'name': 'home',
            'position': (0.25, 0.0, 0.08),  # Adjusted for smaller arm
            'orientation': (0, 0),
            'gripper': HOME_DEGREES[5]
        }
    
    @staticmethod
    def get_reach_forward():
        """Reach forward"""
        return {
            'name': 'reach_forward',
            'position': (0.28, 0.0, 0.08),  # Adjusted
            'orientation': (0, 0),
            'gripper': 90
        }
    
    @staticmethod
    def get_reach_up():
        """Reach upward"""
        return {
            'name': 'reach_up',
            'position': (0.15, 0.0, 0.20),  # Adjusted
            'orientation': (math.pi/4, 0),
            'gripper': 90
        }
    
    @staticmethod
    def get_reach_down():
        """Reach down to pick something"""
        return {
            'name': 'reach_down',
            'position': (0.25, 0.0, 0.05),
            'orientation': (-math.pi/4, 0),
            'gripper': 120  # Open gripper
        }
    
    @staticmethod
    def get_reach_left():
        """Reach to the left"""
        return {
            'name': 'reach_left',
            'position': (0.2, 0.2, 0.15),
            'orientation': (0, 0),
            'gripper': 90
        }
    
    @staticmethod
    def get_reach_right():
        """Reach to the right"""
        return {
            'name': 'reach_right',
            'position': (0.2, -0.2, 0.15),
            'orientation': (0, 0),
            'gripper': 90
        }
    
    @staticmethod
    def get_all_poses():
        """Get all predefined poses"""
        return {
            'home': PredefinedPoses.get_home(),
            'reach_forward': PredefinedPoses.get_reach_forward(),
            'reach_up': PredefinedPoses.get_reach_up(),
            'reach_down': PredefinedPoses.get_reach_down(),
            'reach_left': PredefinedPoses.get_reach_left(),
            'reach_right': PredefinedPoses.get_reach_right(),
        }


if __name__ == '__main__':
    # Test the IK solver
    solver = ArmIKSolver()
    poses = PredefinedPoses()
    
    print("Testing Inverse Kinematics Solver\n")
    print("=" * 50)
    
    for pose_name, pose_data in poses.get_all_poses().items():
        print(f"\nPose: {pose_name}")
        print(f"Target: {pose_data['position']}")
        
        x, y, z = pose_data['position']
        pitch, roll = pose_data['orientation']
        gripper = pose_data['gripper']
        
        joint_angles = solver.solve_ik(x, y, z, pitch, roll, gripper)
        
        if joint_angles:
            print("Joint Angles (degrees):")
            for joint, angle in joint_angles.items():
                print(f"  {joint}: {angle:.1f}°")
        else:
            print("  UNREACHABLE!")
