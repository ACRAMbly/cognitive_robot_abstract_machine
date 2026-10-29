"""
``ScrewAction`` -- a composite Coraplex action that screws a held tool into a hole.

Verified API baseline
=====================
Every import and attribute below was checked by introspection against the Coraplex
sources on ``upstream/main`` (commit ``0463b6391d``), which is also what
``origin/coraplex-migration`` is aligned with. The names this module needs, and where
they really live in that version:

===========================================  ==========================================
what                                         where
===========================================  ==========================================
``ActionDescription`` (composite base)        ``coraplex.robot_plans.actions.base``
``sequential``                               ``coraplex.plans.factories``
``PlanNode``                                 ``coraplex.plans.plan_node``
``MoveToolCenterPointMotion``                ``coraplex.robot_plans.motions.gripper``
``MoveTCPWaypointsMotion``                   ``coraplex.robot_plans.motions.gripper``
``MoveGripperMotion``                        ``coraplex.robot_plans.motions.gripper``
arm, end effector                            ``semantic_digital_twin.robots.robot_parts``
``GripperState``                             ``semantic_digital_twin.datastructures.definitions``
``Pose``, ``Vector3``                        ``semantic_digital_twin.spatial_types.spatial_types``
``Body``                                     ``semantic_digital_twin.world_description.world_entity``
===========================================  ==========================================

Deliberately *not* referenced, because they no longer exist: ``Arms``,
``ApproachDirection``, ``VerticalAlignment`` and ``GraspDescription``. In the current
API an arm is a :class:`~semantic_digital_twin.robots.robot_parts.Arm` object that
carries its own ``end_effector``, so a grasp description is not needed to know which
gripper to open. There is no ``TYPE_CHECKING`` import of those names anywhere, not even
in annotations.

Two ways to model the thread
============================
Whether a screw can be turned by a joint depends on one thing only: does the world model
contain the thread? A :class:`~semantic_digital_twin.world_description.connections.
ScrewConnection` couples rotation and translation into a single degree of freedom, so a
solver can drive it directly (``Close``/``Open`` from
:mod:`giskardpy.motion_statechart.goals.open_close`) and the screw moves itself along its
thread. That is what :class:`ScrewJointMotion` and :class:`ScrewThreadAction` do, and it
needs the screw to be spawned with
:func:`~sub_parts.shared.utils.spawn_screw_body`. Without such a joint -- a plain
free-floating object, or hardware whose thread the model does not describe -- there is
nothing to turn and the thread has to be faked by moving the hand along a trajectory,
which is what :class:`ScrewAction` does.

============================  ===========================  ======================
                              :class:`ScrewThreadAction`   :class:`ScrewAction`
============================  ===========================  ======================
world model                   1-DoF ``ScrewConnection``    no joint for the thread
driven by                     the joint's ``Close``        a helix of TCP poses
insertion per revolution      ``screw_pitch`` of the       ``thread_pitch`` passed
                              joint                        to the action
limited by                    the joint's limits and the   the wrist roll range,
                              wrist roll range             hence re-grip segments
============================  ===========================  ======================

Why a discretised helix
=======================
Coraplex has no continuous "rotate the end effector by N revolutions" primitive. The
motion layer offers full-pose goals (``CartesianPose``) reachable through
:class:`~coraplex.robot_plans.motions.gripper.MoveToolCenterPointMotion`, an angular
*speed cap* (``CartesianRotationVelocityLimit``), and
:class:`~coraplex.robot_plans.motions.gripper.MoveTCPWaypointsMotion`, which sequences a
list of full pose goals.

There is no twist/screw motion class and no velocity-streaming primitive. The faithful
approximation is therefore to pre-compute the screw trajectory as a helix of discrete
poses and hand the whole list to ``MoveTCPWaypointsMotion``: waypoint *i* both rotates
about the hole's local Z axis and advances along it, so the interpolation between
consecutive waypoints traces a short arc that follows the thread. The rotation is
continuous *in effect* (interpolated between waypoints), not a single continuous
rotation command.

``steps_per_turn`` sets how finely that arc is sampled: 12 steps per revolution is a
30 degree arc per waypoint. Too few steps and the TCP cuts the corner between waypoints;
more steps track the helix better at the cost of a longer motion chart. Kinematic
simulation tolerates coarse sampling well, real hardware tolerates it badly.

Wrist roll limit and re-gripping
================================
A wrist can only roll so far before it hits its joint limit. Measured against Tracy's
URDF in kinematic simulation, one uninterrupted helix of 0.75 revolutions (270 degrees)
completes, while 1.0 revolution (360 degrees) stalls with ``MotionMadeNoProgress`` on the
wrist roll axis. Real threads need far more revolutions than that, so the action splits
``turns`` into segments of at most :attr:`ScrewAction.max_turn_per_grasp` revolutions.
Between two segments it opens the gripper, rotates the tool back by the angle it just
turned *without translating* (the screw is assumed to be held by the thread while the
hand re-positions), and closes the gripper again. That is a planning-level approximation
of re-gripping; a genuine continuous twist on real hardware needs the
``AlternativeMotion`` extension point named at the end of this docstring. Set
``regrasp=False`` to plan one uninterrupted helix instead, which is what the robot can
physically do without letting go of the tool.

Geometry convention
===================
``target_pose`` is the hole as the tool should meet it, so its **local +Z axis is the
insertion direction**. A hole in a table therefore has ``roll = pi`` (local +Z points
down into the table). Positive ``turns`` drives a *right-hand* thread inwards: the pose
rotates about local +Z by negative angles while translating along local +Z.

All poses handed to the motions are expressed in the world root frame; ``ScrewAction``
transforms its target there before planning, which is why the pose properties below
need the action to be part of a plan (they read ``self.world``).

Extension point: force-terminated screwing (deliberately NOT implemented here)
==============================================================================
"Turn until the screw is tight" needs a *terminating* monitor, which the plan factories
cannot express today: :func:`~coraplex.plans.factories.cancel_when` cancels the rest of
the plan with ``PlanCancelled``, and ``repeat`` treats a force spike as a *failure* to
retry rather than as success. The intended extension point is a custom
:class:`~coraplex.robot_plans.motions.base.BaseMotion` (say ``ScrewMotion``) whose
``_motion_chart`` returns a ``StoppedWhenTrue`` monitor wrapping the sequence of
``CartesianPose`` goals. For continuous twisting on real hardware that motion would
additionally need a ``coraplex.alternative_motion_mapping.AlternativeMotion`` subclass
emitting a velocity command instead of pose goals. Nothing here implements that.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from giskardpy.motion_statechart.data_types import DefaultWeights
from giskardpy.motion_statechart.goals.open_close import Close
from giskardpy.motion_statechart.goals.templates import Parallel
from giskardpy.motion_statechart.graph_node import Task
from coraplex.plans.factories import sequential
from coraplex.plans.plan_node import PlanNode
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.motions.base import BaseMotion
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveTCPWaypointsMotion,
    MoveToolCenterPointMotion,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.spatial_types.spatial_types import Pose, Vector3
from semantic_digital_twin.world_description.connections import (
    ActiveConnection1DOF,
    ScrewConnection,
)
from semantic_digital_twin.world_description.world_entity import Body

LOCAL_Z = Vector3(0.0, 0.0, 1.0)
"""
The insertion axis in the hole's own frame. Kept module-level so every helper rotates
about the same axis object.
"""


def screw_step_count(turns: float, steps_per_turn: int) -> int:
    """
    :param turns: Number of revolutions the hand should turn.
    :param steps_per_turn: Number of waypoints sampled per revolution.
    :return: How many waypoints the discretised helix has. At least one, so that a
        "screw" of zero turns still yields a well-formed (single-goal) motion.
    """
    return max(1, int(round(turns * steps_per_turn)))


def screw_depth(turns: float, thread_pitch: float) -> float:
    """
    :param turns: Number of revolutions the hand should turn.
    :param thread_pitch: Metres of insertion per full revolution.
    :return: The insertion depth the helix reaches, in metres.
    """
    return turns * thread_pitch


def offset_along_local_z(pose: Pose, distance: float) -> Pose:
    """
    Translate a pose along its own Z axis, keeping its orientation.

    :param pose: The pose to move from.
    :param distance: Metres to move along local +Z; negative backs off.
    :return: ``pose`` composed with that translation, in ``pose``'s reference frame.
    """
    delta = Pose.from_xyz_axis_angle(
        z=distance, axis=LOCAL_Z, angle=0.0, reference_frame=None
    )
    return pose.homogeneous_matrix @ delta


def build_screw_waypoints(
    base_pose: Pose,
    turns: float = 1.0,
    thread_pitch: float = 0.001,
    steps_per_turn: int = 12,
) -> list[Pose]:
    """
    Sample the helix a screw driver follows when turning ``turns`` revolutions.

    Waypoint *i* is ``base_pose`` composed with a rotation of ``-2*pi*i/steps_per_turn``
    about local Z and a translation of ``thread_pitch*i/steps_per_turn`` along local Z:
    one short arc of the thread per waypoint, clockwise about the insertion axis as seen
    from the tool, i.e. a right-hand thread driven inwards.

    Pure and world-independent: it only needs ``Pose`` algebra, so it can be exercised
    without ROS or a robot.

    :param base_pose: Pose of the hole, local +Z being the insertion axis.
    :param turns: Number of revolutions the hand should turn.
    :param thread_pitch: Metres of insertion per full revolution.
    :param steps_per_turn: Number of waypoints sampled per revolution.
    :return: ``screw_step_count(turns, steps_per_turn)`` poses, last one at full depth,
        all expressed in ``base_pose``'s reference frame.
    :raises ValueError: If an argument would produce a degenerate or backwards helix.
    """
    if steps_per_turn < 1:
        raise ValueError(f"steps_per_turn must be at least 1, got {steps_per_turn}")
    if turns < 0.0:
        raise ValueError(f"turns must not be negative, got {turns}")
    if thread_pitch < 0.0:
        raise ValueError(f"thread_pitch must not be negative, got {thread_pitch}")

    angle_per_step = -2.0 * math.pi / steps_per_turn
    advance_per_step = thread_pitch / steps_per_turn

    waypoints: list[Pose] = []
    for step in range(1, screw_step_count(turns, steps_per_turn) + 1):
        step_in_base = Pose.from_xyz_axis_angle(
            z=advance_per_step * step,
            axis=LOCAL_Z,
            angle=angle_per_step * step,
            reference_frame=None,
        )
        waypoints.append(base_pose.homogeneous_matrix @ step_in_base)
    return waypoints


def split_turns(turns: float, max_turn_per_grasp: float) -> list[float]:
    """
    Cut a total number of revolutions into segments a single wrist roll can achieve.

    :param turns: Total revolutions to screw.
    :param max_turn_per_grasp: Largest rotation allowed before the hand has to re-grip.
    :return: Segment lengths in revolutions, summing to ``turns``, each at most
        ``max_turn_per_grasp``. A zero-turn request yields ``[0.0]``.
    :raises ValueError: If ``turns`` is negative or ``max_turn_per_grasp`` is not positive.
    """
    if turns < 0.0:
        raise ValueError(f"turns must not be negative, got {turns}")
    if max_turn_per_grasp <= 0.0:
        raise ValueError(
            f"max_turn_per_grasp must be positive, got {max_turn_per_grasp}"
        )

    segments: list[float] = []
    remaining = turns
    while remaining > 1e-9:
        segment = min(max_turn_per_grasp, remaining)
        segments.append(segment)
        remaining -= segment
    return segments or [0.0]


def build_unwind_waypoints(
    last_waypoint: Pose,
    turns: float,
    steps_per_turn: int = 12,
) -> list[Pose]:
    """
    Rotate the tool back by ``turns`` revolutions without translating it.

    This is the "let the hand re-position" motion used between two screw segments: the
    gripper is open, the screw stays in the thread, and the wrist rolls back to the
    orientation the segment started from.

    :param last_waypoint: Pose the segment ended at, i.e. after turning ``turns``.
    :param turns: Revolutions to unwind; the last returned pose has the segment's start
        orientation at the same position.
    :param steps_per_turn: Number of waypoints sampled per revolution.
    :return: Waypoints from the first unwind step to the fully unwound pose. Empty if
        ``turns`` is zero.
    :raises ValueError: If ``turns`` is negative or ``steps_per_turn`` is below 1.
    """
    if steps_per_turn < 1:
        raise ValueError(f"steps_per_turn must be at least 1, got {steps_per_turn}")
    if turns < 0.0:
        raise ValueError(f"turns must not be negative, got {turns}")

    angle_per_step = 2.0 * math.pi / steps_per_turn
    return [
        last_waypoint.homogeneous_matrix
        @ Pose.from_xyz_axis_angle(
            z=0.0,
            axis=LOCAL_Z,
            angle=angle_per_step * step,
            reference_frame=None,
        )
        for step in range(1, screw_step_count(turns, steps_per_turn) + 1)
    ]


@dataclass
class ScrewAction(ActionDescription):
    """
    Screws a held tool into a hole by turning it in discrete steps of a helix.

    The plan is: move to a pre-approach pose backed off along the hole's axis, lower onto
    the hole, follow the discretised helix down the thread, optionally open the gripper,
    then retract. See the module docstring for why the helix is discretised and where a
    force-terminated / continuous-twist variant would attach.
    """

    arm: Arm
    """
    The arm that holds the tool and turns it.
    """

    target_pose: Optional[Pose] = field(default=None, kw_only=True)
    """
    Pose of the hole in the world frame; its local +Z axis is the insertion direction.
    Exactly one of ``target_pose`` and the :attr:`body`/:attr:`local_target_pose` pair
    must be given.
    """

    body: Optional[Body] = field(default=None, kw_only=True)
    """
    The body the hole belongs to, used together with :attr:`local_target_pose`.
    """

    local_target_pose: Optional[Pose] = field(default=None, kw_only=True)
    """
    Pose of the hole expressed in :attr:`body`'s frame.
    """

    turns: float = field(default=1.0, kw_only=True)
    """
    Number of revolutions the hand should turn.
    """

    thread_pitch: float = field(default=0.001, kw_only=True)
    """
    Metres of insertion per full revolution, so depth is ``turns * thread_pitch``.
    """

    steps_per_turn: int = field(default=12, kw_only=True)
    """
    Number of waypoints sampled per revolution of the helix.
    """

    pre_approach_distance: float = field(default=0.05, kw_only=True)
    """
    Metres to back off along the hole's axis before lowering onto it.
    """

    retract_distance: float = field(default=0.05, kw_only=True)
    """
    Metres to back off along the hole's axis after screwing.
    """

    release: bool = field(default=True, kw_only=True)
    """
    Whether to open the gripper at the end, before retracting.
    """

    max_turn_per_grasp: float = field(default=0.75, kw_only=True)
    """
    Revolutions one grasp may turn before the wrist has to be re-positioned. 0.75 (270
    degrees) is the largest uninterrupted helix that completes against Tracy's URDF in
    kinematic simulation; 1.0 revolution already stalls on the wrist roll limit.
    """

    regrasp: bool = field(default=True, kw_only=True)
    """
    Whether to split :attr:`turns` into :attr:`max_turn_per_grasp` sized segments and
    re-grip in between. ``False`` plans one uninterrupted helix, which is all the wrist
    can do without letting go of the tool.
    """

    max_linear_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Linear speed cap in m/s for the three point-to-point moves (approach, insertion,
    retract); ``None`` leaves it unconstrained. The turning motion is a waypoint
    sequence and has no single velocity argument.
    """

    allow_gripper_collision: Optional[bool] = field(default=None, kw_only=True)
    """
    Whether the gripper may collide with the world while approaching and retracting.
    The turning motion always allows it, since the tool is in the hole by then.
    """

    @property
    def turn_segments(self) -> list[float]:
        """
        :return: How the requested revolutions are distributed over separate grasps. One
            entry when :attr:`regrasp` is ``False``.
        """
        if not self.regrasp:
            if self.turns < 0.0:
                raise ValueError(f"turns must not be negative, got {self.turns}")
            return [self.turns]
        return split_turns(self.turns, self.max_turn_per_grasp)

    def segment_waypoints(self) -> list[tuple[float, list[Pose], Pose]]:
        """
        Build one helix per grasp, chained through the re-grip motions.

        :return: One ``(turns, waypoints, seed_pose_after)`` triple per segment.
            ``seed_pose_after`` is where the tool sits once it is ready for the next
            segment: the unwound pose when :attr:`regrasp` is on, the last waypoint
            otherwise.
        """
        segments: list[tuple[float, list[Pose], Pose]] = []
        seed_pose = self.resolved_target_pose
        all_turns = self.turn_segments

        for index, turns in enumerate(all_turns):
            waypoints = build_screw_waypoints(
                seed_pose,
                turns=turns,
                thread_pitch=self.thread_pitch,
                steps_per_turn=self.steps_per_turn,
            )
            if self.regrasp and index < len(all_turns) - 1:
                unwind = build_unwind_waypoints(
                    waypoints[-1], turns, self.steps_per_turn
                )
                seed_pose = unwind[-1] if unwind else waypoints[-1]
            else:
                seed_pose = waypoints[-1]
            segments.append((turns, waypoints, seed_pose))
        return segments

    @property
    def waypoints(self) -> list[Pose]:
        """
        :return: Every screwing waypoint of every segment in execution order, expressed
            in the world root frame. The re-grip motions between segments are not part
            of this list.
        """
        return [
            waypoint
            for _, segment, _ in self.segment_waypoints()
            for waypoint in segment
        ]

    @property
    def depth(self) -> float:
        """
        :return: Insertion depth the helix actually reaches, in metres.
        """
        return len(self.waypoints) * self.thread_pitch / self.steps_per_turn

    @property
    def resolved_target_pose(self) -> Pose:
        """
        :return: The hole's pose, resolved from either target variant and expressed in
            the world root frame, as the motions expect.

        .. note:: Reads :attr:`~coraplex.plans.designator.Designator.world`, so the
            action must have been placed in a plan (``sequential``/``add_subplan``)
            before this is evaluated.
        """
        if self.target_pose is not None:
            pose = self.target_pose
        elif self.body is not None and self.local_target_pose is not None:
            pose = self.body.global_pose.homogeneous_matrix @ self.local_target_pose
        else:
            raise ValueError(
                "ScrewAction needs either target_pose or "
                "body together with local_target_pose."
            )
        if pose.reference_frame is None:
            raise ValueError(
                "The screw target pose needs a reference frame, normally world.root."
            )
        return self.world.transform(pose, self.world.root)

    @property
    def _action_plan(self) -> PlanNode:
        target_pose = self.resolved_target_pose
        segments = self.segment_waypoints()
        pre_approach_pose = offset_along_local_z(
            target_pose, -self.pre_approach_distance
        )
        retract_pose = offset_along_local_z(
            segments[-1][1][-1], -self.retract_distance
        )

        children: list = [
            MoveToolCenterPointMotion(
                target=pre_approach_pose,
                arm=self.arm,
                allow_gripper_collision=self.allow_gripper_collision,
                max_linear_velocity=self.max_linear_velocity,
            ),
            MoveToolCenterPointMotion(
                target=target_pose,
                arm=self.arm,
                allow_gripper_collision=self.allow_gripper_collision,
                max_linear_velocity=self.max_linear_velocity,
            ),
        ]

        for index, (turns, waypoints, _) in enumerate(segments):
            children.append(
                MoveTCPWaypointsMotion(
                    waypoints=waypoints,
                    arm=self.arm,
                    allow_gripper_collision=True,
                )
            )
            if self.regrasp and index < len(segments) - 1:
                unwind = build_unwind_waypoints(
                    waypoints[-1], turns, self.steps_per_turn
                )
                children.append(
                    MoveGripperMotion(GripperState.OPEN, self.arm.end_effector)
                )
                children.append(
                    MoveTCPWaypointsMotion(
                        waypoints=unwind,
                        arm=self.arm,
                        allow_gripper_collision=True,
                    )
                )
                children.append(
                    MoveGripperMotion(GripperState.CLOSE, self.arm.end_effector)
                )

        if self.release:
            children.append(
                MoveGripperMotion(GripperState.OPEN, self.arm.end_effector)
            )
        children.append(
            MoveToolCenterPointMotion(
                target=retract_pose,
                arm=self.arm,
                allow_gripper_collision=self.allow_gripper_collision,
                max_linear_velocity=self.max_linear_velocity,
            )
        )
        return sequential(children=children)


# %% Joint-driven screwing
# Everything below drives a ``ScrewConnection`` that the world model already contains,
# instead of faking the thread with a helix of tool poses. The helpers are pure and
# world-independent except for the pose resolution, which -- like ``ScrewAction`` --
# reads ``self.world`` and therefore needs the designator to be part of a plan.


def _vector_components(vector: Vector3) -> tuple[float, float, float]:
    """
    :param vector: A constant spatial vector.
    :return: Its ``(x, y, z)`` components as plain floats.
    """
    return float(vector[0]), float(vector[1]), float(vector[2])


def rotation_from_local_z_to(axis: Vector3) -> Pose:
    """
    The rotation that maps a pose's local +Z axis onto ``axis``.

    Used to line the tool's approach axis up with the thread axis, which is what lets
    the wrist follow the screw's rotation: turning the screw then rolls the wrist rather
    than bending it.

    :param axis: Unit vector to align the local +Z axis with.
    :return: A pure rotation, expressed in the frame the returned pose is composed into.
    :raises ValueError: If ``axis`` is the zero vector or does not point along a single
        direction (i.e. its components are not constant floats).
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
        # ``axis`` is parallel to +Z or -Z, where the cross product degenerates.
        if z > 0.0:
            return Pose.from_xyz_axis_angle(
                axis=LOCAL_Z, angle=0.0, reference_frame=None
            )
        return Pose.from_xyz_axis_angle(
            axis=Vector3(1.0, 0.0, 0.0), angle=math.pi, reference_frame=None
        )
    return Pose.from_xyz_axis_angle(
        axis=Vector3(cross_x / cross_length, cross_y / cross_length, 0.0),
        angle=math.atan2(cross_length, z),
        reference_frame=None,
    )


def screw_joint_goal_angle(
    connection: ActiveConnection1DOF,
    turns: Optional[float] = None,
    travel_distance: Optional[float] = None,
) -> float:
    """
    The joint angle that screws a body ``turns`` revolutions into its parent.

    Positive *turns* mean "drive inwards", i.e. towards the parent the screw is threaded
    into, which is a **negative** rotation about an axis that points from the parent
    towards the screw (see
    :func:`~sub_parts.shared.utils.spawn_screw_body`). *travel_distance* is the signed
    translation along that axis instead, positive meaning away from the parent.

    The result is clamped to the joint's lower limit, which is exactly what
    :class:`~giskardpy.motion_statechart.goals.open_close.Close` does with its goal:
    driving a joint past its limit would otherwise be planned but never reached, and a
    screw without limits cannot be driven by ``Close`` at all.

    :param connection: The screw joint to compute the goal for.
    :param turns: Revolutions to screw inwards.
    :param travel_distance: Signed translation along the joint axis, in metres. Wins over
        *turns* when both are given.
    :return: The signed joint angle in radians, clamped to the lower limit.
    :raises ValueError: If neither argument is given, *turns* is negative, or the
        connection couples no translation to its rotation.
    """
    if not isinstance(connection, ScrewConnection):
        raise ValueError(
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


def screw_joint_delta(axis: Vector3, screw_pitch: float, angle: float) -> Pose:
    """
    The motion the child of a screw joint makes for a given joint angle.

    Mirrors
    :meth:`~semantic_digital_twin.world_description.connections.ScrewConnection.
    add_to_world`: a rotation of ``angle`` about ``axis`` (right-hand rule) combined with
    a translation of ``screw_pitch * angle / (2 * pi)`` along it.

    :param axis: The joint axis, pointing from the parent towards the screw.
    :param screw_pitch: Metres of travel per full revolution.
    :param angle: The joint angle in radians.
    :return: The child's transform relative to its zero-position pose.
    """
    x, y, z = _vector_components(axis)
    travel = screw_pitch * angle / (2.0 * math.pi)
    return Pose.from_xyz_axis_angle(
        x=x * travel,
        y=y * travel,
        z=z * travel,
        axis=axis,
        angle=angle,
        reference_frame=None,
    )


@dataclass
class ScrewJointMotion(BaseMotion):
    """
    Turns a threaded body by driving its own degree of freedom.

    The motion is a single ``Close`` goal: it drives the screw joint towards its lower
    limit while holding the end effector fixed relative to the body, so the arm follows
    the thread instead of pushing against it. Because the thread lives in the world
    model, the insertion is exact -- no helix, no waypoints, and no pitch bookkeeping
    beyond the joint's own ``screw_pitch``.

    The wrist still has to follow the rotation, so this motion is limited by the wrist
    roll range and not by the joint: one grasp cannot turn a real thread many times.
    """

    screw_body: Body
    """
    The threaded body. Its first parent ``ActiveConnection1DOF`` is the joint that gets
    driven, and its pose is what the end effector is held relative to.
    """

    arm: Arm
    """
    The arm that holds the screw and follows its rotation.
    """

    turns: float = field(default=1.0, kw_only=True)
    """
    Revolutions to screw inwards. Ignored when :attr:`travel_distance` is set.
    """

    travel_distance: Optional[float] = field(default=None, kw_only=True)
    """
    Signed translation along the joint axis in metres, positive meaning out of the
    thread. Overrides :attr:`turns` when given.
    """

    allow_gripper_collision: bool = field(default=True, kw_only=True)
    """
    Whether the end effector may touch the plate while following the screw. It usually
    has to: the fingers sit right next to the thread they turn.
    """

    def perform(self):
        return

    @property
    def connection(self) -> ActiveConnection1DOF:
        """
        :return: The joint the screw body hangs from.
        :raises ValueError: If the body has no single-DoF parent connection, i.e. it was
            not spawned with :func:`~sub_parts.shared.utils.spawn_screw_body`.
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

    @property
    def _motion_chart(self) -> Task:
        close = Close(
            tip_link=self.arm.end_effector.tool_frame,
            environment_link=self.screw_body,
            goal_joint_state=self.goal_joint_state,
            # Below collision avoidance: following a thread contorts the arm against
            # whatever is around it.
            mechanism_weight=DefaultWeights.WEIGHT_BELOW_COLLISION_AVOIDANCE,
        )
        if not self.allow_gripper_collision:
            return close
        return Parallel(
            nodes=[
                close,
                *self._only_allow_gripper_collision_rules(self.arm.end_effector),
            ],
            name="ScrewJoint",
        )


@dataclass
class ScrewThreadAction(ActionDescription):
    """
    Grasps a threaded body and screws it in by turning its own joint.

    The plan is: move to a pre-approach pose outside the thread, move onto the grasp
    pose, close the gripper, turn the joint with :class:`ScrewJointMotion`, optionally
    open the gripper, then retract. The grasp pose is resolved from the body's *current*
    pose, so the action follows the object if the scene is rearranged before planning;
    it is never a world constant.

    The pre-approach and the retract directions follow the same convention as
    :class:`ScrewAction`: the pose's local +Z axis is the insertion direction, so backing
    off is a negative offset along it. Here that axis is the thread axis pointing inwards,
    i.e. the negated joint axis (the joint axis points from the plate towards the screw).
    """

    arm: Arm
    """
    The arm that grasps the screw and turns it.
    """

    screw_body: Body
    """
    The threaded body to screw in. Must hang from an ``ActiveConnection1DOF`` such as a
    ``ScrewConnection``, normally spawned with
    :func:`~sub_parts.shared.utils.spawn_screw_body`.
    """

    turns: float = field(default=1.0, kw_only=True)
    """
    Revolutions to screw inwards. The joint's limits must allow them; the goal is
    clamped to the lower limit by the ``Close`` goal otherwise.
    """

    grasp_local_pose: Optional[Pose] = field(default=None, kw_only=True)
    """
    Pose of the end effector relative to the screw body. Defaults to a pure rotation
    that lines the tool's local +Z up with the insertion direction, at the body's origin.
    """

    grasp_offset: Optional[float] = field(default=None, kw_only=True)
    """
    Metres along the screw body's local +X axis (towards the head) where the gripper
    should grasp, used when :attr:`grasp_local_pose` is ``None``. ``None`` grasps at the
    body's origin.
    """

    pre_approach_distance: float = field(default=0.05, kw_only=True)
    """
    Metres to back off along the thread axis before moving onto the grasp pose.
    """

    retract_distance: float = field(default=0.05, kw_only=True)
    """
    Metres to back off along the thread axis after screwing.
    """

    release: bool = field(default=True, kw_only=True)
    """
    Whether to open the gripper before retracting. With ``False`` the retract drags the
    still-held screw back out of the thread.
    """

    max_linear_velocity: Optional[float] = field(default=None, kw_only=True)
    """
    Linear speed cap in m/s for the three point-to-point moves; ``None`` leaves it
    unconstrained.
    """

    allow_gripper_collision: bool = field(default=True, kw_only=True)
    """
    Whether the gripper may touch the plate. It usually must: the fingers are right next
    to the thread they turn, and giskard stalls on that collision otherwise.
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
    def screw_pitch(self) -> float:
        """
        :return: Metres of travel per full revolution, taken from the joint.
        """
        return self.connection.screw_pitch

    @property
    def thread_axis(self) -> Vector3:
        """
        :return: The joint axis, pointing from the plate towards the screw.
        """
        return self.connection.axis

    @property
    def insertion_axis(self) -> Vector3:
        """
        :return: The direction the screw moves when it is driven in, i.e. the negated
            :attr:`thread_axis`.
        """
        x, y, z = _vector_components(self.thread_axis)
        return Vector3(-x, -y, -z)

    @property
    def target_joint_state(self) -> float:
        """
        :return: The joint angle that reaches :attr:`turns` revolutions of insertion.
        """
        return screw_joint_goal_angle(self.connection, turns=self.turns)

    @property
    def default_grasp_local_pose(self) -> Pose:
        """
        :return: The grasp pose used when :attr:`grasp_local_pose` is ``None``: the tool
            aligned with the thread axis. When :attr:`grasp_offset` is given, the grasp
            sits that many metres along the screw body's +X axis (towards the head);
            otherwise it sits at the body's origin.
        """
        rotation = rotation_from_local_z_to(self.insertion_axis)
        if self.grasp_offset is None:
            return rotation
        translation = Pose.from_xyz_axis_angle(x=self.grasp_offset)
        return translation.homogeneous_matrix @ rotation

    @property
    def resolved_grasp_pose(self) -> Pose:
        """
        :return: The grasp pose in the world root frame, derived from the screw body's
            current pose.

        .. note:: Reads :attr:`~coraplex.plans.designator.Designator.world`, so the
            action must have been placed in a plan before this is evaluated.
        """
        local_pose = (
            self.grasp_local_pose
            if self.grasp_local_pose is not None
            else self.default_grasp_local_pose
        )
        pose = self.world.transform(
            self.screw_body.global_pose.homogeneous_matrix @ local_pose,
            self.world.root,
        )
        if pose.reference_frame is None:
            raise ValueError(
                "The screw body's global pose needs a reference frame, normally world.root."
            )
        return pose

    @property
    def inserted_grasp_pose(self) -> Pose:
        """
        :return: Where the tool sits once the joint reached :attr:`target_joint_state`.
            Computed analytically from the screw body's pose and the joint's own thread
            kinematics, so no world state has to be simulated to plan the retract.
        """
        local_pose = (
            self.grasp_local_pose
            if self.grasp_local_pose is not None
            else self.default_grasp_local_pose
        )
        delta = screw_joint_delta(
            self.thread_axis, self.screw_pitch, self.target_joint_state
        )
        # Compose through the homogeneous matrices: a ``Pose`` has no matrix product of
        # its own, only the transformation matrix has.
        return self.world.transform(
            self.screw_body.global_pose.homogeneous_matrix
            @ delta.homogeneous_matrix
            @ local_pose,
            self.world.root,
        )

    @property
    def pre_approach_pose(self) -> Pose:
        """
        :return: The pose the tool backs off to before moving onto the screw.
        """
        return offset_along_local_z(
            self.resolved_grasp_pose, -self.pre_approach_distance
        )

    @property
    def retract_pose(self) -> Pose:
        """
        :return: The pose the tool backs off to after screwing.
        """
        return offset_along_local_z(self.inserted_grasp_pose, -self.retract_distance)

    @property
    def _action_plan(self) -> PlanNode:
        children: list = [
            MoveToolCenterPointMotion(
                target=self.pre_approach_pose,
                arm=self.arm,
                allow_gripper_collision=self.allow_gripper_collision,
                max_linear_velocity=self.max_linear_velocity,
            ),
            MoveToolCenterPointMotion(
                target=self.resolved_grasp_pose,
                arm=self.arm,
                allow_gripper_collision=self.allow_gripper_collision,
                max_linear_velocity=self.max_linear_velocity,
            ),
            MoveGripperMotion(GripperState.CLOSE, self.arm.end_effector),
            ScrewJointMotion(
                screw_body=self.screw_body,
                arm=self.arm,
                turns=self.turns,
                allow_gripper_collision=self.allow_gripper_collision,
            ),
        ]
        if self.release:
            children.append(MoveGripperMotion(GripperState.OPEN, self.arm.end_effector))
        children.append(
            MoveToolCenterPointMotion(
                target=self.retract_pose,
                arm=self.arm,
                allow_gripper_collision=self.allow_gripper_collision,
                max_linear_velocity=self.max_linear_velocity,
            )
        )
        return sequential(children=children)
