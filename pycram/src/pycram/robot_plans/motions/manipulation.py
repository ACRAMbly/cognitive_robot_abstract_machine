from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.world_entity import Body

from .base import BaseMotion
from ...datastructures.enums import Arms
from ...datastructures.pose import PoseStamped
from ...view_manager import ViewManager


@dataclass
class AlignMotion(BaseMotion):
    """
    Slow, precise 6-DOF Cartesian alignment motion used before insertion or snapping.
    Uses a reduced velocity and tighter convergence threshold to achieve sub-millimetre
    positioning accuracy.
    """

    target: PoseStamped
    """Target 6D pose to align the TCP to."""

    arm: Arms
    """Arm whose TCP should be aligned."""

    reference_linear_velocity: float = 0.02
    """Very slow linear speed for fine alignment (m/s)."""

    reference_angular_velocity: float = 0.05
    """Very slow angular speed for fine alignment (rad/s)."""

    threshold: float = 0.001
    """Convergence threshold in metres (1 mm default)."""

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
        return CartesianPose(
            root_link=root,
            tip_link=tip,
            goal_pose=self.target.to_spatial_type(),
            name="AlignTCP",
            reference_linear_velocity=self.reference_linear_velocity,
            reference_angular_velocity=self.reference_angular_velocity,
            threshold=self.threshold,
        )


@dataclass
class PressMotion(BaseMotion):
    """
    Cartesian motion that drives the TCP toward a target pose with a slow, controlled
    velocity — the basis for snap and press-fit operations.

    The slow speed allows the force controller / compliance to register contact
    before the positional goal is reached.
    """

    target: PoseStamped
    """Target 6D pose (typically the final snap/press pose)."""

    arm: Arms
    """Arm to use for the pressing motion."""

    reference_linear_velocity: float = 0.03
    """Slow linear velocity for press operations (m/s)."""

    reference_angular_velocity: float = 0.05
    """Angular velocity (rad/s)."""

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
        return CartesianPose(
            root_link=root,
            tip_link=tip,
            goal_pose=self.target.to_spatial_type(),
            name="PressTCP",
            reference_linear_velocity=self.reference_linear_velocity,
            reference_angular_velocity=self.reference_angular_velocity,
        )


@dataclass
class UpdateToolFrameMotion(BaseMotion):
    """
    Attaches a virtual tool body to the robot's tool frame via a FixedConnection,
    effectively shifting the active TCP to the tip of the tool.

    This motion does **not** produce a Giskard task — it modifies the world's
    kinematic structure directly, so `_motion_chart` returns an identity-like
    joint task (a no-op for the QP solver).
    """

    tool_body: Body
    """The Body in the world representing the tool to attach."""

    arm: Arms
    """Arm whose tool frame will be used as the attachment parent."""

    def perform(self):
        """Apply the world modification immediately when performed."""
        tip = ViewManager().get_end_effector_view(self.arm, self.robot_view).tool_frame
        current_tf = self.world.compute_forward_kinematics(tip, self.tool_body)
        with self.world.modify_world():
            self.world.remove_connection(self.tool_body.parent_connection)
            connection = FixedConnection(
                parent=tip,
                child=self.tool_body,
                parent_T_connection_expression=current_tf,
            )
            self.world.add_connection(connection)

    @property
    def _motion_chart(self):
        # No Giskard motion needed — world modification handled in perform()
        from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState
        return JointPositionList(
            goal_state=JointState.from_mapping({}),
            name="UpdateToolFrame_NoOp",
        )
