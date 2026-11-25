#!/usr/bin/env python3
"""
3D Visualization Canvas with Draggable IK Target Point
This module provides a 3D visualization of the robot arm with an interactive
target point that can be dragged to control the arm using inverse kinematics.
"""

import tkinter as tk
from tkinter import ttk
import math
import numpy as np

from arm_controller.joint_contract import HOME_DEGREES, servo_degrees_to_radians


class Arm3DVisualization:
    """
    3D visualization of the robot arm with a draggable IK target point.
    Uses a simple orthographic projection to render the arm and target.
    """

    RENDER_INTERVAL_MS = 16
    DRAG_IK_INTERVAL_MS = 33
    DRAW_ARM = 1
    DRAW_TARGET = 2
    DRAW_FULL = 4
    JOINT_COLORS = ('#888888', '#6666aa', '#66aa66', '#aa6666', '#66aaaa')
    LINK_COLORS = ('#888888', '#666699', '#669966', '#996666')
    
    def __init__(self, parent_frame, ik_solver, on_target_move_callback=None):
        """
        Initialize the 3D visualization.
        
        Args:
            parent_frame: Tkinter parent frame to embed the canvas
            ik_solver: IK solver instance for forward/inverse kinematics
            on_target_move_callback: Callback function when target is moved
                                     Called with (x, y, z) in meters
        """
        self.parent = parent_frame
        self.ik_solver = ik_solver
        self.on_target_move = on_target_move_callback
        
        # Canvas dimensions
        self.canvas_width = 500
        self.canvas_height = 420
        
        # View parameters
        self.view_azimuth = 45  # Degrees around vertical axis
        self.view_elevation = 25  # Degrees above horizontal
        self.zoom = 550  # Pixels per meter (lower = wider FOV)
        self.center_offset_x = self.canvas_width // 2
        self.center_offset_y = self.canvas_height - 60
        self._projection_view_key = None
        self._projection_trig = None
        
        # Target position (meters)
        # Increased for 5-joint arm with extended reach
        self.target_x = 0.25
        self.target_y = 0.0
        self.target_z = 0.22
        
        # Current joint angles (degrees)
        self.joint_angles = list(HOME_DEGREES)

        # Actual gripper position from FK/TF (meters) - separate from computed position
        self.actual_gripper_x = 0.0
        self.actual_gripper_y = 0.0
        self.actual_gripper_z = 0.15
        
        # Link lengths (from IK solver)
        self.L1 = self.ik_solver.L1
        self.L2 = self.ik_solver.L2
        self.L3 = self.ik_solver.L3
        self.L4 = self.ik_solver.L4
        
        # Dragging state
        self.dragging = False
        self.drag_start_x = 0
        self.drag_start_y = 0
        self.rotating_view = False
        
        # Auto-follow mode: gripper tracks target in real-time during drag
        self.auto_follow = False
        self._drag_ik_after_id = None
        self._draw_after_id = None
        self._draw_dirty = 0
        self._arm_item_ids = []
        self._gripper_item_ids = []
        self._target_item_ids = []
        self._position_label_text = None
        
        # Callback for when user wants to move arm to target
        self.on_move_to_target = None
        
        # Create UI
        self.setup_ui()
        
        # Initial draw - set target to a reasonable default position
        self.sync_target_to_gripper()
        self.draw_scene()
    
    def setup_ui(self):
        """Setup the visualization UI"""
        
        # Main frame
        self.frame = tk.LabelFrame(
            self.parent,
            text="3D View - Drag Orange Point for IK",
            font=('Arial', 10, 'bold'),
            bg='#3b3b3b',
            fg='white',
            bd=2
        )
        self.frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Canvas for 3D rendering
        self.canvas = tk.Canvas(
            self.frame,
            width=self.canvas_width,
            height=self.canvas_height,
            bg='#1a1a2e',
            highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Bind mouse events
        self.canvas.bind('<Button-1>', self.on_mouse_down)
        self.canvas.bind('<B1-Motion>', self.on_mouse_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_mouse_up)
        self.canvas.bind('<Button-3>', self.on_right_mouse_down)
        self.canvas.bind('<B3-Motion>', self.on_right_mouse_drag)
        self.canvas.bind('<ButtonRelease-3>', self.on_right_mouse_up)
        self.canvas.bind('<MouseWheel>', self.on_mouse_wheel)
        self.canvas.bind('<Button-4>', self.on_mouse_wheel_up)
        self.canvas.bind('<Button-5>', self.on_mouse_wheel_down)
        
        # View controls frame
        controls_frame = tk.Frame(self.frame, bg='#3b3b3b')
        controls_frame.pack(fill=tk.X, padx=5, pady=2)
        
        # View preset buttons
        tk.Button(
            controls_frame,
            text="Front",
            font=('Arial', 8),
            bg='#4a4a4a',
            fg='white',
            command=lambda: self.set_view(0, 0)
        ).pack(side=tk.LEFT, padx=2)
        
        tk.Button(
            controls_frame,
            text="Side",
            font=('Arial', 8),
            bg='#4a4a4a',
            fg='white',
            command=lambda: self.set_view(90, 0)
        ).pack(side=tk.LEFT, padx=2)
        
        tk.Button(
            controls_frame,
            text="Top",
            font=('Arial', 8),
            bg='#4a4a4a',
            fg='white',
            command=lambda: self.set_view(0, 90)
        ).pack(side=tk.LEFT, padx=2)
        
        tk.Button(
            controls_frame,
            text="Isometric",
            font=('Arial', 8),
            bg='#4a4a4a',
            fg='white',
            command=lambda: self.set_view(45, 25)
        ).pack(side=tk.LEFT, padx=2)
        
        tk.Button(
            controls_frame,
            text="Reset Target",
            font=('Arial', 8),
            bg='#006600',
            fg='white',
            command=self.reset_target
        ).pack(side=tk.RIGHT, padx=2)
        
        # Position display
        position_text = (
            f"Target: X={self.target_x*1000:.0f} "
            f"Y={self.target_y*1000:.0f} Z={self.target_z*1000:.0f} mm"
        )
        self.pos_label = tk.Label(
            self.frame,
            text=position_text,
            font=('Arial', 9),
            bg='#3b3b3b',
            fg='#00ffff'
        )
        self.pos_label.pack(pady=2)
        self._position_label_text = position_text
        
        # Instructions (updated when auto_follow changes)
        self.instructions_label = tk.Label(
            self.frame,
            text="Drag orange dot to set target | Press Start to enable auto-follow",
            font=('Arial', 8),
            bg='#3b3b3b',
            fg='#888888'
        )
        self.instructions_label.pack(pady=1)
    
    def project_3d_to_2d(self, x, y, z):
        """
        Project a 3D point to 2D screen coordinates using orthographic projection.
        
        Args:
            x, y, z: 3D coordinates in meters
            
        Returns:
            tuple: (screen_x, screen_y)
        """
        view_key = (self.view_azimuth, self.view_elevation)
        if getattr(self, '_projection_view_key', None) != view_key:
            az = math.radians(self.view_azimuth)
            el = math.radians(self.view_elevation)
            self._projection_view_key = view_key
            self._projection_trig = (
                math.cos(az),
                math.sin(az),
                math.cos(el),
                math.sin(el),
            )
        cos_az, sin_az, cos_el, sin_el = self._projection_trig
        
        # Rotate around vertical axis (azimuth)
        x_rot = x * cos_az - y * sin_az
        y_rot = x * sin_az + y * cos_az
        z_rot = z
        
        # Rotate around horizontal axis (elevation)
        y_final = y_rot * cos_el - z_rot * sin_el
        z_final = y_rot * sin_el + z_rot * cos_el
        
        # Project to screen (orthographic)
        screen_x = self.center_offset_x + x_rot * self.zoom
        screen_y = self.center_offset_y - z_final * self.zoom
        
        return screen_x, screen_y
    
    def draw_scene(self):
        """Draw the complete 3D scene"""
        self.canvas.delete('all')
        self._arm_item_ids = []
        self._gripper_item_ids = []
        self._target_item_ids = []
        
        # Draw grid on XY plane
        self.draw_grid()
        
        # Draw coordinate axes
        self.draw_axes()
        
        # Draw the robot arm
        self.draw_arm()
        
        # Draw workspace boundary (sphere showing reachable area)
        self.draw_workspace()
        
        # Draw the current gripper position (green)
        self.draw_gripper_marker()
        
        # Draw the target point (orange - where user wants to move)
        self.draw_target()

        self._draw_dirty = 0

    def request_draw(self, components=None):
        """Coalesce redraws while retaining unchanged canvas layers."""
        if components is None:
            components = self.DRAW_FULL

        dirty = getattr(self, '_draw_dirty', 0)
        if components & self.DRAW_FULL:
            self._draw_dirty = self.DRAW_FULL
        elif not dirty & self.DRAW_FULL:
            self._draw_dirty = dirty | components

        if self._draw_after_id is not None:
            return False
        self._draw_after_id = self.canvas.after(
            self.RENDER_INTERVAL_MS,
            self._flush_draw,
        )
        return True

    def _flush_draw(self):
        self._draw_after_id = None
        dirty = getattr(self, '_draw_dirty', self.DRAW_FULL)
        self._draw_dirty = 0

        if dirty & self.DRAW_FULL:
            self.draw_scene()
            return

        if dirty & self.DRAW_ARM:
            self.draw_arm()
            self.draw_gripper_marker()

        if dirty & self.DRAW_TARGET:
            self.draw_target()
            self.update_position_label()

    def cancel_pending_callbacks(self):
        """Cancel callbacks using the canvas that registered them."""
        self._cancel_drag_callback()
        if self._draw_after_id is not None:
            self.canvas.after_cancel(self._draw_after_id)
            self._draw_after_id = None

    def _cancel_drag_callback(self):
        if self._drag_ik_after_id is not None:
            self.canvas.after_cancel(self._drag_ik_after_id)
            self._drag_ik_after_id = None

    @staticmethod
    def _items_exist(items, expected_count):
        """Return whether a retained canvas layer is ready for coordinate updates."""
        return len(items) == expected_count

    def draw_grid(self):
        """Draw a grid on the XY plane"""
        grid_size = 0.3  # 300mm
        grid_step = 0.05  # 50mm
        
        for i in np.arange(-grid_size, grid_size + grid_step, grid_step):
            # Lines parallel to X
            x1, y1 = self.project_3d_to_2d(i, -grid_size, 0)
            x2, y2 = self.project_3d_to_2d(i, grid_size, 0)
            self.canvas.create_line(
                x1, y1, x2, y2, fill='#333355', width=1, tags='grid'
            )
            
            # Lines parallel to Y
            x1, y1 = self.project_3d_to_2d(-grid_size, i, 0)
            x2, y2 = self.project_3d_to_2d(grid_size, i, 0)
            self.canvas.create_line(
                x1, y1, x2, y2, fill='#333355', width=1, tags='grid'
            )
    
    def draw_axes(self):
        """Draw coordinate axes"""
        origin = self.project_3d_to_2d(0, 0, 0)
        axis_length = 0.1  # 100mm
        
        # X-axis (red)
        x_end = self.project_3d_to_2d(axis_length, 0, 0)
        self.canvas.create_line(origin[0], origin[1], x_end[0], x_end[1], 
                               fill='#ff4444', width=2, arrow=tk.LAST,
                               tags='axes')
        self.canvas.create_text(
            x_end[0] + 10, x_end[1], text='X', fill='#ff4444',
            font=('Arial', 9), tags='axes'
        )
        
        # Y-axis (green)
        y_end = self.project_3d_to_2d(0, axis_length, 0)
        self.canvas.create_line(origin[0], origin[1], y_end[0], y_end[1], 
                               fill='#44ff44', width=2, arrow=tk.LAST,
                               tags='axes')
        self.canvas.create_text(
            y_end[0] + 10, y_end[1], text='Y', fill='#44ff44',
            font=('Arial', 9), tags='axes'
        )
        
        # Z-axis (blue)
        z_end = self.project_3d_to_2d(0, 0, axis_length)
        self.canvas.create_line(origin[0], origin[1], z_end[0], z_end[1], 
                               fill='#4444ff', width=2, arrow=tk.LAST,
                               tags='axes')
        self.canvas.create_text(
            z_end[0] + 10, z_end[1], text='Z', fill='#4444ff',
            font=('Arial', 9), tags='axes'
        )
    
    def draw_workspace(self):
        """Draw approximate reachable workspace boundary"""
        # Draw a few circles at different heights to show workspace
        # Increased by 1.5x to accommodate new 5th joint
        max_reach = (self.L2 + self.L3 + self.L4) * 1.5
        
        for z in [0.05, 0.15, 0.25, 0.35]:
            # Calculate radius at this height
            z_from_shoulder = z - self.L1
            if abs(z_from_shoulder) < max_reach:
                radius = math.sqrt(max_reach**2 - z_from_shoulder**2) if max_reach > abs(z_from_shoulder) else 0
                if radius > 0:
                    # Draw circle using line segments
                    points = []
                    for angle in range(0, 361, 15):
                        rad = math.radians(angle)
                        px = radius * math.cos(rad)
                        py = radius * math.sin(rad)
                        sx, sy = self.project_3d_to_2d(px, py, z)
                        points.extend([sx, sy])
                    
                    if len(points) >= 4:
                        self.canvas.create_line(
                            points, fill='#334433', width=1, smooth=True,
                            tags='workspace'
                        )
    
    def draw_arm(self):
        """Draw the robot arm based on current joint angles.
        
        Draws 5 motor joints:
        1. Base (gray) - ground level
        2. Shoulder (blue) - top of base
        3. Elbow (green) - end of upper arm
        4. Wrist (red) - end of forearm
        5. Gripper (cyan) - where gripper attaches
        """
        # Get joint positions using forward kinematics
        positions = self.calculate_arm_positions()
        
        projected = [self.project_3d_to_2d(*position) for position in positions]
        item_ids = getattr(self, '_arm_item_ids', [])

        # Create the layer once. Subsequent frames only update coordinates,
        # avoiding delete/create traffic through Tk (and through X11 on Windows).
        if not self._items_exist(item_ids, 9):
            item_ids = []
            # Draw links first (behind joints)
            for i in range(len(positions) - 1):
                x1, y1 = projected[i]
                x2, y2 = projected[i + 1]
                item_ids.append(self.canvas.create_line(
                    x1, y1, x2, y2,
                    fill=self.LINK_COLORS[i % len(self.LINK_COLORS)], width=6,
                    tags='arm'
                ))

            # Draw all 5 joints
            for i, (jx, jy) in enumerate(projected):
                color = (
                    self.JOINT_COLORS[i]
                    if i < len(self.JOINT_COLORS)
                    else '#aaaaaa'
                )
                radius = 7 if i == 4 else 5
                item_ids.append(self.canvas.create_oval(
                    jx - radius, jy - radius, jx + radius, jy + radius,
                    fill=color, outline='#ffffff', width=1, tags='arm'
                ))
            self._arm_item_ids = item_ids
            return

        # Retained-mode update: four links followed by five joint circles.
        for i in range(len(positions) - 1):
            x1, y1 = projected[i]
            x2, y2 = projected[i + 1]
            self.canvas.coords(item_ids[i], x1, y1, x2, y2)

        for i, (jx, jy) in enumerate(projected):
            radius = 7 if i == 4 else 5
            self.canvas.coords(
                item_ids[4 + i],
                jx - radius, jy - radius, jx + radius, jy + radius,
            )
    
    def calculate_arm_positions(self):
        """Return the original five-point geometric arm schematic.

        The canvas and mouse IK intentionally share ``ArmIKSolver`` geometry.
        Fixed URDF/TF frame origins are not motor joints and must never be
        appended to this schematic.
        """

        base_rad, shoulder_rad, elbow_rad, forearm_rad = (
            servo_degrees_to_radians(self.joint_angles)[:4]
        )

        # Use the same four lengths as the geometric solver.  The historical
        # canvas used 0.074 m for the forearm while IK used 0.086 m, which left
        # the green gripper marker short of an otherwise correct target.
        L_base = self.L1
        L_upper = self.L2
        L_forearm = self.L3
        L_wrist = self.L4
        
        positions = []
        
        # Joint 1: Base motor (origin at ground level)
        positions.append((0, 0, 0))
        
        # Joint 2: Shoulder motor (top of base pedestal)
        positions.append((0, 0, L_base))
        
        # Joint 3: Elbow motor
        # Shoulder rotates the upper arm in vertical plane
        # At shoulder_rad=0 (configured home), arm points straight up
        elbow_x = L_upper * math.sin(shoulder_rad)
        elbow_z = L_base + L_upper * math.cos(shoulder_rad)
        positions.append((
            elbow_x * math.cos(base_rad),
            elbow_x * math.sin(base_rad),
            elbow_z
        ))
        
        # Joint 4: Forearm/Wrist motor
        # Cumulative angle for forearm
        arm_angle = shoulder_rad + elbow_rad
        wrist_x = elbow_x + L_forearm * math.sin(arm_angle)
        wrist_z = elbow_z + L_forearm * math.cos(arm_angle)
        positions.append((
            wrist_x * math.cos(base_rad),
            wrist_x * math.sin(base_rad),
            wrist_z
        ))
        
        # Joint 5: Gripper motor (wrist rotate / gripper base)
        # This is where the gripper attaches - add forearm pitch
        full_angle = arm_angle + forearm_rad
        gripper_x = wrist_x + L_wrist * math.sin(full_angle)
        gripper_z = wrist_z + L_wrist * math.cos(full_angle)
        positions.append((
            gripper_x * math.cos(base_rad),
            gripper_x * math.sin(base_rad),
            gripper_z
        ))
        
        return positions
    
    def draw_target(self):
        """Draw the draggable target point (orange ball - where user wants arm to go)"""
        tx, ty = self.project_3d_to_2d(self.target_x, self.target_y, self.target_z)
        radius = 12
        ground_x, ground_y = self.project_3d_to_2d(self.target_x, self.target_y, 0)
        item_ids = getattr(self, '_target_item_ids', [])

        if not self._items_exist(item_ids, 6):
            item_ids = [
                self.canvas.create_oval(
                    tx - radius, ty - radius, tx + radius, ty + radius,
                    fill='#ff8800', outline='#ffffff', width=2, tags='target'
                ),
                self.canvas.create_line(
                    tx - radius - 5, ty, tx + radius + 5, ty,
                    fill='#ffaa00', width=1, tags='target'
                ),
                self.canvas.create_line(
                    tx, ty - radius - 5, tx, ty + radius + 5,
                    fill='#ffaa00', width=1, tags='target'
                ),
                self.canvas.create_line(
                    tx, ty, ground_x, ground_y,
                    fill='#cc6600', width=1, dash=(3, 3), tags='target'
                ),
                self.canvas.create_oval(
                    ground_x - 4, ground_y - 4, ground_x + 4, ground_y + 4,
                    fill='#ff8800', outline='', tags='target'
                ),
                self.canvas.create_text(
                    tx, ty - radius - 12, text='Target', fill='#ff8800',
                    font=('Arial', 8, 'bold'), tags='target'
                ),
            ]
            self._target_item_ids = item_ids
            return

        self.canvas.coords(
            item_ids[0], tx - radius, ty - radius, tx + radius, ty + radius
        )
        self.canvas.coords(
            item_ids[1], tx - radius - 5, ty, tx + radius + 5, ty
        )
        self.canvas.coords(
            item_ids[2], tx, ty - radius - 5, tx, ty + radius + 5
        )
        self.canvas.coords(item_ids[3], tx, ty, ground_x, ground_y)
        self.canvas.coords(
            item_ids[4], ground_x - 4, ground_y - 4, ground_x + 4, ground_y + 4
        )
        self.canvas.coords(item_ids[5], tx, ty - radius - 12)
    
    def draw_gripper_marker(self):
        """Draw the gripper at the final point of the five-point schematic."""
        positions = self.calculate_arm_positions()
        if positions:
            gripper_pos = positions[-1]
            self.actual_gripper_x = gripper_pos[0]
            self.actual_gripper_y = gripper_pos[1]
            self.actual_gripper_z = gripper_pos[2]
        
        gx, gy = self.project_3d_to_2d(self.actual_gripper_x, self.actual_gripper_y, self.actual_gripper_z)
        
        radius = 8
        item_ids = getattr(self, '_gripper_item_ids', [])
        if not self._items_exist(item_ids, 2):
            item_ids = [
                self.canvas.create_oval(
                    gx - radius, gy - radius, gx + radius, gy + radius,
                    fill='#00ff00', outline='#ffffff', width=2, tags='gripper'
                ),
                self.canvas.create_text(
                    gx, gy + radius + 10, text='Gripper', fill='#00ff00',
                    font=('Arial', 8, 'bold'), tags='gripper'
                ),
            ]
            self._gripper_item_ids = item_ids
        else:
            self.canvas.coords(
                item_ids[0], gx - radius, gy - radius, gx + radius, gy + radius
            )
            self.canvas.coords(item_ids[1], gx, gy + radius + 10)
        
        # Update the position label to show both
        self.update_position_label()
    
    def sync_target_to_gripper(self):
        """
        Sync the target position to the current gripper position
        calculated from the arm's forward kinematics.
        """
        # Use calculate_arm_positions to get actual gripper position
        # This ensures consistency with how the arm is drawn
        positions = self.calculate_arm_positions()
        if len(positions) > 0:
            end_pos = positions[-1]
            self.target_x = end_pos[0]
            self.target_y = end_pos[1]
            self.target_z = end_pos[2]
            self.update_position_label()
    
    def trigger_move_to_target(self):
        """Trigger the callback to move arm to target position"""
        print(f"Tkinter trigger_move_to_target: X={self.target_x:.3f}, Y={self.target_y:.3f}, Z={self.target_z:.3f}")
        if self.on_target_move:
            self.on_target_move(self.target_x, self.target_y, self.target_z)
    
    def _fire_drag_ik(self):
        """Fire the IK callback after a short throttle delay."""
        self._drag_ik_after_id = None
        if self.auto_follow and self.on_target_move and self.dragging:
            self.on_target_move(self.target_x, self.target_y, self.target_z)

    def set_auto_follow(self, enabled):
        """Enable or disable automatic gripper-follows-target mode."""
        self.auto_follow = enabled
        if hasattr(self, 'instructions_label') and self.instructions_label:
            if enabled:
                self.instructions_label.config(
                    text="⚡ AUTO-FOLLOW ACTIVE — Drag target, arm follows in real-time",
                    fg='#ffaa00'
                )
            else:
                self.instructions_label.config(
                    text="Drag orange dot to set target | Press Start to enable auto-follow",
                    fg='#888888'
                )

    def set_move_callback(self, callback):
        """Set the callback for when user clicks Move to Target"""
        self.on_move_to_target = callback
    
    def set_view(self, azimuth, elevation):
        """Set the view angle"""
        self.view_azimuth = azimuth
        self.view_elevation = elevation
        self.request_draw(self.DRAW_FULL)
    
    def reset_target(self):
        """Reset target to current gripper position"""
        self.sync_target_to_gripper()
        self.request_draw(self.DRAW_TARGET)
    
    def update_position_label(self):
        """Update the position display label"""
        text = (
            f"Target: X={self.target_x*1000:.0f} "
            f"Y={self.target_y*1000:.0f} Z={self.target_z*1000:.0f} mm"
        )
        if text == getattr(self, '_position_label_text', None):
            return False
        self._position_label_text = text
        self.pos_label.config(text=text)
        return True
    
    def is_click_on_target(self, screen_x, screen_y):
        """Check if a click is on the target point"""
        tx, ty = self.project_3d_to_2d(self.target_x, self.target_y, self.target_z)
        distance = math.sqrt((screen_x - tx)**2 + (screen_y - ty)**2)
        return distance < 20  # 20 pixel tolerance
    
    def on_mouse_down(self, event):
        """Handle left mouse button down"""
        if self.is_click_on_target(event.x, event.y):
            self.dragging = True
            self.drag_start_x = event.x
            self.drag_start_y = event.y
    
    def on_mouse_drag(self, event):
        """Handle left mouse drag"""
        if self.dragging:
            # Calculate movement in screen space
            dx = event.x - self.drag_start_x
            dy = event.y - self.drag_start_y
            
            # Convert to 3D movement based on view angle
            az = math.radians(self.view_azimuth)
            el = math.radians(self.view_elevation)
            
            # Movement scale
            scale = 1.0 / self.zoom
            
            # The projection uses:
            # screen_x = center + x_rot * zoom, where x_rot = x*cos(az) - y*sin(az)
            # screen_y = center - z_final * zoom, where z_final = y*sin(el) + z*cos(el)
            #
            # So to invert: moving right on screen (positive dx) should increase x_rot
            # And moving up on screen (negative dy) should increase z_final
            
            # For horizontal screen movement (dx):
            # This maps to movement in the x_rot direction
            # x_rot = x*cos(az) - y*sin(az), so we need to move in world coords
            # To increase x_rot: increase x (proportional to cos(az)) or decrease y (proportional to sin(az))
            world_dx_from_horizontal = dx * scale * math.cos(az)
            world_dy_from_horizontal = -dx * scale * math.sin(az)
            
            # For vertical screen movement (dy):
            # screen_y decreases when z_final increases (note the minus sign in projection)
            # z_final = y_rot*sin(el) + z*cos(el)
            # y_rot = x*sin(az) + y*cos(az)
            # Moving up on screen (dy < 0) should increase z_final
            # So we use -dy to get the z increase
            world_dz_from_vertical = -dy * scale * math.cos(el)
            # Also affects y_rot component
            world_dx_from_vertical = -dy * scale * math.sin(el) * math.sin(az)
            world_dy_from_vertical = -dy * scale * math.sin(el) * math.cos(az)
            
            # Combine movements
            world_dx = world_dx_from_horizontal + world_dx_from_vertical
            world_dy = world_dy_from_horizontal + world_dy_from_vertical
            world_dz = world_dz_from_vertical
            
            # Update target position
            self.target_x += world_dx
            self.target_y += world_dy
            self.target_z += world_dz
            
            # Clamp to reasonable workspace
            self.target_x = max(-0.35, min(0.35, self.target_x))
            self.target_y = max(-0.35, min(0.35, self.target_y))
            self.target_z = max(0.0, min(0.4, self.target_z))
            
            # Update drag start for next frame
            self.drag_start_x = event.x
            self.drag_start_y = event.y
            
            # The position label is refreshed with the coalesced target frame.
            self.request_draw(self.DRAW_TARGET)
            
            # If auto-follow is active, trigger IK in real-time as user drags
            if self.auto_follow and self.on_target_move and self._drag_ik_after_id is None:
                self._drag_ik_after_id = self.canvas.after(
                    self.DRAG_IK_INTERVAL_MS,
                    self._fire_drag_ik,
                )
    
    def on_mouse_up(self, event):
        """Handle left mouse button release"""
        was_dragging = self.dragging
        self.dragging = False
        self._cancel_drag_callback()
        if was_dragging and self.auto_follow and self.on_target_move:
            # Fire IK one final time at the release position
            self.on_target_move(self.target_x, self.target_y, self.target_z)
    
    def on_right_mouse_down(self, event):
        """Handle right mouse button down (view rotation)"""
        self.rotating_view = True
        self.drag_start_x = event.x
        self.drag_start_y = event.y
    
    def on_right_mouse_drag(self, event):
        """Handle right mouse drag (view rotation)"""
        if self.rotating_view:
            dx = event.x - self.drag_start_x
            dy = event.y - self.drag_start_y
            
            # Update view angles
            self.view_azimuth += dx * 0.5
            self.view_elevation += dy * 0.5
            
            # Clamp elevation
            self.view_elevation = max(-89, min(89, self.view_elevation))
            
            # Normalize azimuth
            self.view_azimuth = self.view_azimuth % 360
            
            self.drag_start_x = event.x
            self.drag_start_y = event.y
            
            self.request_draw(self.DRAW_FULL)
    
    def on_right_mouse_up(self, event):
        """Handle right mouse button release"""
        self.rotating_view = False
    
    def on_mouse_wheel(self, event):
        """Handle mouse wheel (zoom)"""
        if event.delta > 0:
            self.zoom *= 1.1
        else:
            self.zoom /= 1.1
        
        self.zoom = max(200, min(2000, self.zoom))
        self.request_draw(self.DRAW_FULL)
    
    def on_mouse_wheel_up(self, event):
        """Handle mouse wheel up (Linux)"""
        self.zoom *= 1.1
        self.zoom = min(2000, self.zoom)
        self.request_draw(self.DRAW_FULL)
    
    def on_mouse_wheel_down(self, event):
        """Handle mouse wheel down (Linux)"""
        self.zoom /= 1.1
        self.zoom = max(200, self.zoom)
        self.request_draw(self.DRAW_FULL)
    
    def set_target(self, x, y, z):
        """Set target position programmatically"""
        target = (float(x), float(y), float(z))
        if target == (self.target_x, self.target_y, self.target_z):
            return False
        self.target_x, self.target_y, self.target_z = target
        self.update_position_label()
        self.request_draw(self.DRAW_TARGET)
        return True
    
    def set_joint_angles(self, angles):
        """Update joint angles for arm visualization"""
        updated = list(angles)
        if updated == self.joint_angles:
            return False
        self.joint_angles = updated
        self.request_draw(self.DRAW_ARM)
        return True

    def update_from_ik_solution(self, joint_angles_dict):
        """Update arm visualization from IK solution dictionary"""
        ik_to_idx = {
            'base': 0,
            'shoulder': 1,
            'elbow': 2,
            'forearm': 3,
            'wrist': 4,
            'gripper': 5
        }
        
        updated = list(self.joint_angles)
        for joint, idx in ik_to_idx.items():
            if joint in joint_angles_dict:
                updated[idx] = joint_angles_dict[joint]
        
        # After IK solution, target should already be at desired position
        # (the target was dragged there), so don't sync back
        return self.set_joint_angles(updated)
    
    def set_actual_gripper_position(self, x, y, z):
        """Set the actual gripper position from FK/TF (for accurate green marker)"""
        position = (float(x), float(y), float(z))
        if position == (
            self.actual_gripper_x,
            self.actual_gripper_y,
            self.actual_gripper_z,
        ):
            return False
        self.actual_gripper_x, self.actual_gripper_y, self.actual_gripper_z = position
        self.request_draw(self.DRAW_ARM)
        return True
