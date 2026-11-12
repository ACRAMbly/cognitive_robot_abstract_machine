"""
Joint-driven screwing actions for the Coraplex statechart API.

A :class:`~semantic_digital_twin.world_description.connections.ScrewConnection` couples
rotation and translation into a single degree of freedom, so the motion solver can drive
it directly with
:class:`~giskardpy.motion_statechart.goals.grasped_mechanism.ArticulateGraspedMechanism`
and the screw moves itself along its thread. That is what :class:`ScrewJointMotion` and
:class:`ScrewThreadAction` do for a screw body spawned with
:func:`~sub_parts.shared.utils.spawn_screw_body`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from coraplex.robot_plans.actions.base import GoalAction, SequentialAction
from coraplex.robot_plans.actions.core.pick_up import HasGraspChoice
from coraplex.robot_plans.mixins import (
    HasApproachesGraspPoses,
    MovesToolCenterPoint,
    OperatesGripper,
    PlaceTuningParameters,
    ReattachesBranches,
)
from cramph.composites import Parallel
from cramph.node import StatechartNode
from giskardpy.motion_statechart.goals.grasped_mechanism import (
    ArticulateGraspedMechanism,
)
from giskardpy.motion_statechart.weights import DefaultWeights
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
    Vector3,
)
from semantic_digital_twin.collision_checking.collision_matrix import CollisionRule
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionForBodies,
    AllowCollisionForEndEffector,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.world_description.connections import (
    ActiveConnection1DOF,
    ScrewConnection,
)
from semantic_digital_twin.world_description.world_entity import Body

from sub_parts.shared.utils import Bolt

# %% Spatial and thread geometry helpers


def offset_along_local_z(pose: Pose, distance: float) -> Pose:
    """
    Translate a pose along its own Z axis, keeping its orientation.

    :param pose: The pose to move from.
    :param distance: Metres to move along local +Z; negative backs off.
    :return: ``pose`` composed with that translation, in ``pose``'s reference frame.
    """
    delta = Pose.from_xyz_axis_angle(
        z=distance, axis=Vector3.Z(), angle=0.0, reference_frame=None
    )
    return pose.homogeneous_matrix @ delta


def _vector_components(vector: Vector3) -> tuple[float, float, float]:
    """
    :param vector: A constant spatial vector.
    :return: Its ``(x, y, z)`` components as plain floats.
    """
    return float(vector[0]), float(vector[1]), float(vector[2])


def rotation_from_local_z_to(axis: Vector3) -> Pose:
    """
    The rotation that maps a pose's local +Z axis onto ``axis``.

    :param axis: Unit vector to align the local +Z axis with.
    :return: A pure rotation, expressed in the frame the returned pose is composed into.
    :raises ValueError: If ``axis`` is the zero vector.
    """
    x, y, z = _vector_components(axis)
    length = math.sqrt(x * x + y * y + z * z)
    if length == 0.0:
        raise ValueError("Cannot align with the zero vector.")
    x, y, z = x / length, y / length, z / length

    # Rotating local +Z by the shortest arc onto ``axis``: the rotation axis is
    # ``Z x axis``, which is (-y, x, 0) and independent of the z component.
    cross_x, cross_y = -y, x
    cross_length = math.hypot(cross_x, cross_y)
    if cross_length < 1e-9:
        if z > 0.0:
            return Pose.from_xyz_axis_angle(
                axis=Vector3.Z(), angle=0.0, reference_frame=None
            )
        return Pose.from_xyz_axis_angle(
            axis=Vector3.X(), angle=math.pi, reference_frame=None
        )
    return Pose.from_xyz_axis_angle(
        axis=Vector3(cross_x / cross_length, cross_y / cross_length, 0.0),
        angle=math.atan2(cross_length, z),
        reference_frame=None,
    )


def screw_joint_goal_angle(
    connection: ActiveConnection1DOF,
    turns: float | None = None,
    travel_distance: float | None = None,
) -> float:
    """
    The joint angle that screws a body ``turns`` revolutions into its parent.

    Positive *turns* mean "drive inwards" towards the parent the screw is threaded into,
    which is a negative rotation about an axis that points from the parent towards the
    screw (see :func:`~sub_parts.shared.utils.spawn_screw_body`).

    :param connection: The screw joint to compute the goal for.
    :param turns: Revolutions to screw inwards.
    :param travel_distance: Signed translation along the joint axis, in metres. Wins over
        *turns* when both are given.
    :return: The signed joint angle in radians, clamped to the lower limit.
    :raises TypeError: If connection is not a ScrewConnection.
    :raises ValueError: If neither argument is given or *turns* is negative.
    """
    if not isinstance(connection, ScrewConnection):
        raise TypeError(
            f"{type(connection).__name__} couples no translation to its rotation; "
            "joint-driven screwing needs a ScrewConnection."
        )
    if travel_distance is None:
        if turns is None:
            raise ValueError("Need either turns or travel_distance.")
        if turns < 0.0:
            raise ValueError(f"turns must not be negative, got {turns}")
        travel_distance = -turns * connection.screw_pitch

    angle = float(connection.rotation_angle_for_travel_distance(travel_distance))
    lower_limit = connection.dof.limits.lower.position
    if lower_limit is not None:
        angle = max(float(lower_limit), angle)
    return angle


# %% Joint-driven screwing actions


@dataclass(eq=False, repr=False, kw_only=True)
class ScrewJointMotion(GoalAction):
    """
    Turns a threaded body by driving its own degree of freedom with
    :class:`~giskardpy.motion_statechart.goals.grasped_mechanism.ArticulateGraspedMechanism`.
    """

    screw_body: Body
    """
    The threaded body whose parent :class:`ActiveConnection1DOF` gets driven.
    """

    arm: Arm
    """
    The arm that holds the screw and follows its rotation.
    """

    turns: float = field(default=1.0, kw_only=True)
    """
    Revolutions to screw inwards. Ignored when :attr:`travel_distance` is set.
    """

    travel_distance: float | None = field(default=None, kw_only=True)
    """
    Signed translation along the joint axis in metres, positive meaning out of the
    thread. Overrides :attr:`turns` when given.
    """

    allow_gripper_collision: bool = field(default=True, kw_only=True)
    """
    Whether the end effector may touch the plate while following the screw.
    """

    @property
    def connection(self) -> ActiveConnection1DOF:
        """
        :return: The joint the screw body hangs from.
        :raises ValueError: If the body has no single-DoF parent connection.
        """
        connection = self.screw_body.get_first_parent_connection_of_type(
            ActiveConnection1DOF
        )
        if connection is None:
            raise ValueError(
                f"{self.screw_body.name} has no single-DoF parent connection to drive."
            )
        return connection

    @property
    def goal_joint_state(self) -> float:
        """
        :return: The joint angle that reaches the requested insertion.
        """
        return screw_joint_goal_angle(
            self.connection,
            turns=self.turns,
            travel_distance=self.travel_distance,
        )

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        if not self.allow_gripper_collision:
            return []
        rules: list[CollisionRule] = [
            AllowCollisionForEndEffector(end_effector=self.arm.end_effector)
        ]
        held = self.arm.end_effector.held_bodies
        if held:
            rules.append(AllowCollisionForBodies(allowed_collision_bodies=set(held)))
        return rules

    def create_goal(self) -> StatechartNode:
        return ArticulateGraspedMechanism(
            tip_link=self.arm.end_effector.tool_frame,
            environment_link=self.screw_body,
            goal_joint_state=self.goal_joint_state,
            mechanism_weight=DefaultWeights.WEIGHT_BELOW_COLLISION_AVOIDANCE,
        )


@dataclass(eq=False, repr=False, kw_only=True)
class ScrewThreadAction(MovesToolCenterPoint, OperatesGripper, SequentialAction):
    """
    Screws a threaded body in by turning its own joint, either grasping it by hand or
    driving it with a held tool such as a :class:`~sub_parts.shared.screwdriver.Screwdriver`.

    Every pose is read relative to the screw body when its step starts, so the screw may
    have been carried to its hole by earlier actions of the same plan.
    """

    arm: Arm
    """
    The arm that grasps the screw, or holds the tool, and turns it.
    """

    screw_body: Body
    """
    The threaded body to screw in, hanging from an :class:`ActiveConnection1DOF` such as
    a :class:`ScrewConnection`.
    """

    turns: float = field(default=1.0, kw_only=True)
    """
    Revolutions the screw stands inwards from its joint's zero position once this
    action ends.
    """

    tool_grasp: GraspCandidate | None = field(default=None, kw_only=True)
    """
    The grasp by which :attr:`arm` already holds a :class:`Tool` whose tip drives the
    screw; ``None`` grasps the screw by hand. A held tool keeps the gripper closed.
    """

    grasp_local_pose: Pose | None = field(default=None, kw_only=True)
    """
    Pose of the end effector, or of the tool's tip, relative to the screw body. Defaults
    to a pure rotation that lines its local +Z up with the insertion direction.
    """

    grasp_offset: float | None = field(default=None, kw_only=True)
    """
    Metres along the screw body's local +X axis (towards the head) where the gripper or
    tool tip meets the screw, used when :attr:`grasp_local_pose` is ``None``.
    """

    pre_approach_distance: float = field(default=0.05, kw_only=True)
    """
    Metres to back off along the thread axis before moving onto the screw.
    """

    retract_distance: float = field(default=0.05, kw_only=True)
    """
    Metres to back off along the thread axis after screwing.
    """

    release: bool = field(default=True, kw_only=True)
    """
    Whether to open the gripper before retracting; ignored while holding a tool.
    """

    max_linear_velocity: float | None = field(default=None, kw_only=True)
    """
    Linear speed cap in m/s for the point-to-point moves; ``None`` leaves it
    unconstrained.
    """

    allow_gripper_collision: bool = field(default=True, kw_only=True)
    """
    Whether the gripper, and anything it holds, may touch the screw and its counterpart.
    """

    @property
    def connection(self) -> ActiveConnection1DOF:
        """
        :return: The joint the screw body hangs from.
        :raises ValueError: If the body has no single-DoF parent connection.
        """
        connection = self.screw_body.get_first_parent_connection_of_type(
            ActiveConnection1DOF
        )
        if connection is None:
            raise ValueError(
                f"{self.screw_body.name} has no single-DoF parent connection to drive."
            )
        return connection

    @property
    def insertion_axis(self) -> Vector3:
        """
        :return: The direction the screw moves when it is driven in, i.e. the negated
            joint axis.
        """
        x, y, z = _vector_components(self.connection.axis)
        return Vector3(-x, -y, -z)

    @property
    def default_grasp_local_pose(self) -> Pose:
        """
        :return: The pose used when :attr:`grasp_local_pose` is ``None``: local +Z
            aligned with the insertion direction, optionally offset along local +X.
        """
        rotation = rotation_from_local_z_to(self.insertion_axis)
        offset = self.grasp_offset
        if offset is None:
            bolts = self.screw_body.get_semantic_annotations_by_type(Bolt)
            if bolts:
                offset = bolts[0].head_offset
        if offset is None:
            return rotation
        translation = Pose.from_xyz_axis_angle(x=offset)
        return (translation.homogeneous_matrix @ rotation.homogeneous_matrix).pose

    @property
    def engage_pose(self) -> Pose:
        """
        :return: Where the end effector or tool tip meets the screw, in the screw body's
            frame, so that it is resolved where the screw is when the step starts.
        """
        if self.grasp_local_pose is not None:
            return self.grasp_local_pose
        screw_frame = HomogeneousTransformationMatrix(reference_frame=self.screw_body)
        return (screw_frame @ self.default_grasp_local_pose.homogeneous_matrix).pose

    @property
    def tool_tip_T_tool_frame(self) -> Pose | None:
        """
        :return: The pose of the gripper's tool frame relative to the held tool's tip,
            ``None`` without a tool.
        """
        if self.tool_grasp is None:
            return None
        tool = self.tool_grasp.graspable
        tool_T_tool_frame = self.arm.end_effector.tool_frame_goal_for_grasp(
            self.tool_grasp.grasp_pose
        )
        return self.world.transform(tool_T_tool_frame, tool.get_tool_frame())

    def tool_frame_goal(self, tip_goal: Pose) -> Pose:
        """
        :param tip_goal: Where the gripper's tool frame, or the held tool's tip, should
            end up.
        :return: The matching goal for the gripper's tool frame.
        """
        tip_T_tool_frame = self.tool_tip_T_tool_frame
        if tip_T_tool_frame is None:
            return tip_goal
        return (tip_goal.homogeneous_matrix @ tip_T_tool_frame.homogeneous_matrix).pose

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        if not self.allow_gripper_collision:
            return []
        rules: list[CollisionRule] = [
            AllowCollisionForEndEffector(end_effector=self.arm.end_effector)
        ]
        held = self.arm.end_effector.held_bodies
        if held:
            rules.append(AllowCollisionForBodies(allowed_collision_bodies=set(held)))
        return rules

    def create_steps(self) -> list[StatechartNode]:
        end_effector = self.arm.end_effector
        holds_tool = self.tool_grasp is not None
        engage_pose = self.engage_pose
        steps: list[StatechartNode] = [
            self.tool_center_point_goal(
                self.tool_frame_goal(
                    offset_along_local_z(engage_pose, -self.pre_approach_distance)
                ),
                self.arm,
                max_linear_velocity=self.max_linear_velocity,
            ),
            self.tool_center_point_goal(
                self.tool_frame_goal(engage_pose),
                self.arm,
                max_linear_velocity=self.max_linear_velocity,
            ),
        ]
        if not holds_tool:
            steps.append(
                self.gripper_goal_type.moving_gripper_to(
                    end_effector.default_configuration(GripperState.CLOSE)
                )
            )
        steps.append(
            ScrewJointMotion(
                screw_body=self.screw_body,
                arm=self.arm,
                turns=self.turns,
                allow_gripper_collision=self.allow_gripper_collision,
            )
        )
        if self.release and not holds_tool:
            steps.append(
                self.gripper_goal_type.moving_gripper_to(
                    end_effector.default_configuration(GripperState.OPEN)
                )
            )
        steps.append(
            self.tool_center_point_goal(
                self.tool_frame_goal(
                    offset_along_local_z(engage_pose, -self.retract_distance)
                ),
                self.arm,
                max_linear_velocity=self.max_linear_velocity,
            )
        )
        return steps


# %% Inserting a held screw into its hole


@dataclass(eq=False, repr=False, kw_only=True)
class InsertScrewAction(
    MovesToolCenterPoint,
    OperatesGripper,
    ReattachesBranches,
    SequentialAction,
    HasGraspChoice,
    HasApproachesGraspPoses,
    PlaceTuningParameters,
):
    """
    Sets a held screw into its hole, threads it onto the counterpart, optionally turns
    it a little by hand so it stays seated, then lets go and withdraws.
    """

    counterpart: Body
    """
    The body the screw is threaded into, such as a plate or an airplane part.
    """

    target_location: Pose
    """
    The pose the screw has once it sits in its hole.
    """

    turn_by_hand: bool = True
    """
    Whether to turn the screw by :attr:`hand_turns` before letting go of it.
    """

    hand_turns: float = 0.1
    """
    Revolutions the screw stands inwards once turned by hand.
    """

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        return [AllowCollisionForEndEffector(end_effector=self.arm.end_effector)]

    def create_steps(self) -> list[StatechartNode]:
        end_effector = self.arm.end_effector
        screw = self.grasp.graspable.root
        at_hole = self.grasp_pose_sequence(
            self.grasp.moved_to(self.target_location), end_effector, self.grasp
        )
        steps: list[StatechartNode] = [
            self.tool_center_point_goal(
                at_hole.retreat,
                self.arm,
                max_linear_velocity=self.transport_linear_velocity,
            ),
            self.tool_center_point_goal(
                at_hole.grasp,
                self.arm,
                max_linear_velocity=self.placing_linear_velocity,
            ),
            *self.reattach(screw, self.counterpart),
        ]
        if self.turn_by_hand:
            steps.append(
                ScrewJointMotion(screw_body=screw, arm=self.arm, turns=self.hand_turns)
            )
        steps.extend(
            [
                Parallel(
                    nodes=[
                        self.gripper_goal_type.moving_gripper_to(
                            end_effector.default_configuration(
                                GripperState.OPEN,
                                finger_velocity=self.release_opening_velocity,
                            )
                        ),
                        self.tool_center_point_goal(at_hole.grasp, self.arm),
                    ]
                ),
                self.tool_center_point_goal(
                    at_hole.pre_grasp,
                    self.arm,
                    max_linear_velocity=self.retract_linear_velocity,
                ),
            ]
        )
        return steps
