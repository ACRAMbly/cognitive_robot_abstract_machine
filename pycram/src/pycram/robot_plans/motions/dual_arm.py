from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
from giskardpy.motion_statechart.goals.templates import Parallel, Sequence
from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState

from .base import BaseMotion
from ...datastructures.enums import Arms
from ...datastructures.pose import PoseStamped
from ...view_manager import ViewManager
from semantic_digital_twin.world_description.world_entity import Body
from giskardpy.motion_statechart.goals.collision_avoidance import CollisionAvoidance
from giskardpy.model.collision_matrix_manager import CollisionRequest, CollisionAvoidanceTypes
from krrood.symbolic_math.symbolic_math import Scalar


@dataclass
class DualArmMotion(BaseMotion):
    """
    Moves both robot arms simultaneously to their respective target poses.
    Uses a Parallel node so the QP solver satisfies both CartesianPose goals at once.
    """

    left_pose: PoseStamped
    """Target 6D pose for the left arm TCP."""

    right_pose: PoseStamped
    """Target 6D pose for the right arm TCP."""

    reference_linear_velocity: float = 0.2
    """Maximum linear velocity for both arms (m/s)."""

    reference_angular_velocity: float = 0.2
    """Maximum angular velocity for both arms (rad/s)."""

    ignored_collision_bodies: Optional[List[Body]] = None
    """Optional list of bodies that the grippers are allowed to collide with (e.g., the handover object)."""

    def perform(self):
        return

    @property
    def _motion_chart(self):
        left_tip = ViewManager().get_end_effector_view(Arms.LEFT, self.robot_view).tool_frame
        right_tip = ViewManager().get_end_effector_view(Arms.RIGHT, self.robot_view).tool_frame

        root = (
            self.world.root
            if self.robot_view.full_body_controlled
            else self.robot_view.root
        )

        left_task = CartesianPose(
            root_link=root,
            tip_link=left_tip,
            goal_pose=self.left_pose.to_spatial_type(),
            name="LeftArm_Move",
            reference_linear_velocity=self.reference_linear_velocity,
            reference_angular_velocity=self.reference_angular_velocity,
        )
        right_task = CartesianPose(
            root_link=root,
            tip_link=right_tip,
            goal_pose=self.right_pose.to_spatial_type(),
            name="RightArm_Move",
            reference_linear_velocity=self.reference_linear_velocity,
            reference_angular_velocity=self.reference_angular_velocity,
        )
        
        nodes_list = [left_task, right_task]
        
        # Build CollisionAvoidance
        left_gripper_links = ViewManager().get_arm_view(Arms.LEFT, self.robot_view).gripper.gripper_links
        right_gripper_links = ViewManager().get_arm_view(Arms.RIGHT, self.robot_view).gripper.gripper_links
        all_grippers = left_gripper_links + right_gripper_links
        left_gripper_bodies = [self.world.get_body_by_name(l) for l in left_gripper_links if hasattr(self.world, 'get_body_by_name')]
        right_gripper_bodies = [self.world.get_body_by_name(l) for l in right_gripper_links if hasattr(self.world, 'get_body_by_name')]
        
        # Filter Nones if bodies don't exist
        left_gripper_bodies = [b for b in left_gripper_bodies if b is not None]
        right_gripper_bodies = [b for b in right_gripper_bodies if b is not None]
        all_gripper_bodies = left_gripper_bodies + right_gripper_bodies
        
        if all_gripper_bodies:
            safe_entries = []
            
            # Allow left to collide with right
            if left_gripper_bodies and right_gripper_bodies:
                safe_entries.append(CollisionRequest(
                    type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                    body_group1=left_gripper_bodies,
                    body_group2=right_gripper_bodies,
                    distance=0.05
                ))
            
            # Allow grippers to collide with ignored bodies (e.g. the object)
            if self.ignored_collision_bodies:
                safe_entries.append(CollisionRequest(
                    type_=CollisionAvoidanceTypes.ALLOW_COLLISION,
                    body_group1=all_gripper_bodies,
                    body_group2=self.ignored_collision_bodies,
                    distance=0.05
                ))
                
            if safe_entries:
                allow_col = CollisionAvoidance(collision_entries=safe_entries)
                allow_col.start_condition = Scalar.const_true()
                nodes_list.append(allow_col)

        return Parallel(nodes=nodes_list)


@dataclass
class HelicalMotion(BaseMotion):
    """
    Executes a helical (screw) motion around the local Z-axis of the start pose.
    Generates a sequence of CartesianPose waypoints, each rotated by `angle_per_segment`
    and translated by `pitch / segments_per_rotation` along the local Z-axis.

    Useful for screwing or fastening operations.
    """

    start_pose: PoseStamped
    """Starting 6D pose of the TCP (also the screw engagement pose)."""

    arm: Arms
    """Arm to use for the helical motion."""

    rotations: float = 3.0
    """Number of full rotations to perform."""

    pitch: float = 0.001
    """Distance advanced per full rotation (metres). Default: 1mm per turn."""

    reference_linear_velocity: float = 0.005
    """Linear speed along the axis (m/s)."""

    reference_angular_velocity: float = 1.0
    """Angular speed around the axis (rad/s)."""

    segments_per_rotation: int = 8
    """Number of waypoint segments per full rotation for accuracy."""

    def perform(self):
        return

    @property
    def _motion_chart(self):
        tip = ViewManager().get_end_effector_view(self.arm, self.robot_view).tool_frame
        root = (
            self.world.root
            if self.robot_view.full_body_controlled
            else self.robot_view.root
        )

        angle_per_seg = (2 * math.pi) / self.segments_per_rotation
        z_per_seg = self.pitch / self.segments_per_rotation
        total_segments = int(self.rotations * self.segments_per_rotation)

        # Build waypoints by accumulating rotation and translation in local frame
        start = self.start_pose
        current_q = np.array([
            start.orientation.x,
            start.orientation.y,
            start.orientation.z,
            start.orientation.w,
        ])
        current_pos = np.array([
            start.position.x,
            start.position.y,
            start.position.z,
        ])

        def _quat_multiply(q1, q2):
            """Hamilton product q1 * q2."""
            x1, y1, z1, w1 = q1
            x2, y2, z2, w2 = q2
            return np.array([
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            ])

        def _rot_matrix_from_quat(q):
            x, y, z, w = q
            return np.array([
                [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
                [2*(x*y + z*w),       1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
                [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x**2 + y**2)],
            ])

        def _quat_from_axis_angle(axis, angle):
            half = angle / 2.0
            s = math.sin(half)
            return np.array([axis[0]*s, axis[1]*s, axis[2]*s, math.cos(half)])

        tasks = []
        for i in range(total_segments):
            # Rotate around local Z by angle_per_seg
            local_z = _rot_matrix_from_quat(current_q)[:, 2]
            q_rot = _quat_from_axis_angle(local_z, angle_per_seg)
            new_q = _quat_multiply(current_q, q_rot)
            new_q /= np.linalg.norm(new_q)

            # Translate along local Z
            delta_pos = local_z * z_per_seg
            new_pos = current_pos + delta_pos

            # Build PoseStamped waypoint
            wp = PoseStamped()
            wp.header.frame_id = start.header.frame_id
            wp.pose.position.x = float(new_pos[0])
            wp.pose.position.y = float(new_pos[1])
            wp.pose.position.z = float(new_pos[2])
            wp.pose.orientation.x = float(new_q[0])
            wp.pose.orientation.y = float(new_q[1])
            wp.pose.orientation.z = float(new_q[2])
            wp.pose.orientation.w = float(new_q[3])

            tasks.append(CartesianPose(
                root_link=root,
                tip_link=tip,
                goal_pose=wp.to_spatial_type(),
                name=f"Screw_Seg_{i}",
                reference_linear_velocity=self.reference_linear_velocity,
                reference_angular_velocity=self.reference_angular_velocity,
            ))

            current_q = new_q
            current_pos = new_pos

        return Sequence(nodes=tasks)
