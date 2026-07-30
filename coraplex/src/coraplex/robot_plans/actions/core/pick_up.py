from __future__ import annotations

import logging
from dataclasses import dataclass

from typing_extensions import Any, Dict, Optional

from coraplex.locations.pose_validator import AreReachableBy, IsObjectReachableBy
from coraplex.plans.attachment_nodes import AttachNode, DetachNode
from coraplex.plans.plan_node import PlanNode
from krrood.entity_query_language.core.variable import Variable
from krrood.entity_query_language.factories import (
    and_,
    or_,
    not_,
    variable_from,
    ConditionType,
)
from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import (
    Arms,
    MovementType,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.exceptions import GraspVerificationFailed
from coraplex.plans.factories import sequential, execute_single
from coraplex.querying.predicates import GripperIsFree
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from coraplex.view_manager import ViewManager
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.reasoning.predicates import allclose
from semantic_digital_twin.reasoning.robot_predicates import is_body_in_gripper
from semantic_digital_twin.robots.robot_part_mixins import HasMobileBase
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)

GRASP_DETECTION_THRESHOLD: float = 0.9
"""
Minimum fraction of sampled rays between the gripper's fingers that must hit an object
for it to be considered grasped/held (see
:func:`~semantic_digital_twin.reasoning.robot_predicates.is_body_in_gripper`).
"""


@dataclass
class ReachAction(ActionDescription):
    """
    Let the robot reach a specific pose.
    """

    target_pose: Pose
    """
    Pose that should be reached.
    """

    arm: Arms
    """
    The arm that should be used for pick up.
    """

    grasp_description: GraspDescription
    """
    The grasp description that should be used for picking up the object.
    """

    object_designator: Optional[Body] = None
    """
    Object designator_description describing the object that should be picked up.
    """

    reverse_reach_order: bool = False
    """
    Whether the grasp pose sequence should be approached in reverse order.
    """

    pre_approach_linear_velocity: Optional[float] = None
    """
    Linear reference velocity (in m/s) for the initial pre-pose approach. ``None`` keeps
    the default velocity.
    """

    final_approach_linear_velocity: Optional[float] = None
    """
    Linear reference velocity (in m/s) for the final approach onto the target pose.
    ``None`` keeps the default velocity.
    """

    open_gripper_at_pre_pose: bool = False
    """
    Whether to open the gripper once the pre-pose is reached, used by
    :class:`PickUpAction` to open before its slower final approach.
    """

    @property
    def _action_plan(self) -> PlanNode:
        target_pre_pose, target_pose, _ = self.grasp_description.pose_sequence(
            self.target_pose, self.object_designator, reverse=self.reverse_reach_order
        )
        children = [
            MoveToolCenterPointMotion(
                target_pre_pose,
                self.arm,
                allow_gripper_collision=False,
                reference_linear_velocity=self.pre_approach_linear_velocity,
            ),
        ]
        if self.open_gripper_at_pre_pose:
            children.append(
                MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm)
            )
        children.append(
            MoveToolCenterPointMotion(
                target_pose,
                self.arm,
                allow_gripper_collision=False,
                movement_type=MovementType.CARTESIAN,
                reference_linear_velocity=self.final_approach_linear_velocity,
            )
        )
        return sequential(children=children)

    def execute(self) -> Any:
        self.add_subplan(self.action_plan).perform()

    @staticmethod
    def pre_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The sequence in which the robot would reach the target pose needs to be
        achievable.
        """
        return and_(
            IsObjectReachableBy(
                context=Context(
                    robot=context.robot,
                    world=context.world,
                    alternative_motion_mappings=context.alternative_motion_mappings,
                ),
                arm=variables["arm"],
                object_designator=kwargs["object_designator"],
                grasp_description=kwargs["grasp_description"],
                target_pose=kwargs["target_pose"],
                reverse=kwargs["reverse_reach_order"],
            ),
        )

    @staticmethod
    def post_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The end effector needs to be close to the target pose.
        """
        end_effector = ViewManager.get_end_effector_view(kwargs["arm"], context.robot)
        return or_(
            is_body_in_gripper(variable_from(kwargs["object_designator"]), end_effector)
            > GRASP_DETECTION_THRESHOLD,
            allclose(
                variable_from(kwargs["object_designator"]).global_pose.to_position(),
                variable_from(end_effector.tool_frame).global_pose.to_position(),
                atol=3e-2,
            ),
        )


@dataclass
class PickUpAction(ActionDescription):
    """
    Let the robot pick up an object.
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be picked up.
    """

    arm: Arms
    """
    The arm that should be used for pick up.
    """

    grasp_description: GraspDescription
    """
    The GraspDescription that should be used for picking up the object.
    """

    grasp_opening: Optional[float] = None
    """
    Explicit finger opening (in meters) the grasp closes to, instead of the gripper's
    fully-closed CLOSE state. ``None`` keeps the default CLOSE behaviour.
    """

    pre_approach_linear_velocity: Optional[float] = None
    """
    Linear reference velocity (in m/s) for the initial reach towards the object's
    pre-grasp standoff. ``None`` keeps the default velocity.
    """

    grasp_linear_velocity: Optional[float] = None
    """
    Linear reference velocity (in m/s) for the final approach onto the grasp pose.
    ``None`` keeps the default velocity.
    """

    grasp_closing_velocity: Optional[float] = None
    """
    Finger joint velocity (in m/s) used while closing onto the object. ``None`` keeps
    the default velocity.
    """

    lift_linear_velocity: Optional[float] = None
    """
    Linear reference velocity (in m/s) for lifting the object clear of the table after
    grasping. ``None`` keeps the default velocity.
    """

    grasp_stall_minimum_time: Optional[float] = None
    """
    Minimum stall dwell time (in seconds, see
    :attr:`~coraplex.robot_plans.motions.gripper.MoveGripperMotion.stall_minimum_time`)
    for the CLOSE motion. ``None`` keeps the default.
    """

    object_friction: Optional[float] = None
    """
    Sliding friction coefficient to apply to the target object's geom before this pick,
    overriding the world's default. Not consumed by this action itself -- applying it is
    the caller's responsibility (see
    :meth:`~physics_simulators.mujoco_simulator.MujocoSimulator.set_geom_friction`);
    recorded here for persistence. ``None`` leaves the friction untouched.
    """

    max_grasp_attempts: Optional[int] = None
    """
    How many times to attempt reach and close before giving up, each verified via
    :func:`~semantic_digital_twin.reasoning.robot_predicates.is_body_in_gripper`. ``None``
    keeps the original single-attempt behaviour, raising
    :class:`~coraplex.exceptions.GraspVerificationFailed` immediately on failure.
    """

    def _grasp_attempt_plan(self) -> PlanNode:
        """
        :return: One reach-and-close attempt at grasping :attr:`object_designator`,
            without lifting it.
        """
        return sequential(
            children=[
                ReachAction(
                    target_pose=self.object_designator.global_pose,
                    object_designator=self.object_designator,
                    arm=self.arm,
                    grasp_description=self.grasp_description,
                    pre_approach_linear_velocity=self.pre_approach_linear_velocity,
                    final_approach_linear_velocity=self.grasp_linear_velocity,
                    open_gripper_at_pre_pose=True,
                ),
                MoveGripperMotion(
                    motion=GripperState.CLOSE,
                    gripper=self.arm,
                    target_opening=self.grasp_opening,
                    finger_velocity=self.grasp_closing_velocity,
                    stall_minimum_time=self.grasp_stall_minimum_time,
                    tolerate_stall=True,
                ),
                AttachNode(
                    body=self.object_designator,
                    new_parent=ViewManager.get_end_effector_view(
                        self.arm, self.robot
                    ).tool_frame,
                ),
            ],
        )

    def _lift_plan(self) -> PlanNode:
        """
        :return: The plan that lifts the already-grasped :attr:`object_designator` clear
            of the table.
        """
        _, _, lift_to_pose = self.grasp_description.grasp_pose_sequence(
            self.object_designator
        )
        return MoveToolCenterPointMotion(
            lift_to_pose,
            self.arm,
            allow_gripper_collision=True,
            movement_type=MovementType.TRANSLATION,
            reference_linear_velocity=self.lift_linear_velocity,
        )

    def _grasp_succeeded(self) -> bool:
        """
        :return: Whether :attr:`object_designator` is currently detected between the
            gripper's fingers (see :data:`GRASP_DETECTION_THRESHOLD`).
        """
        end_effector = ViewManager.get_end_effector_view(self.arm, self.robot)
        return (
            is_body_in_gripper(self.object_designator, end_effector)
            > GRASP_DETECTION_THRESHOLD
        )

    def execute(self) -> Any:
        """
        Attempt to grasp and lift :attr:`object_designator`, retrying up to
        :attr:`max_grasp_attempts` times.

        Each attempt is verified twice: right after closing, and again after lifting,
        since a grasp held only by contact/friction can still slip during the lift
        itself even though it looked successful right after closing. A failed attempt
        is detached (undoing :meth:`_grasp_attempt_plan`'s kinematic attachment) before
        the next reach.

        :raises GraspVerificationFailed: if no attempt succeeds.
        """
        attempt_count = (
            self.max_grasp_attempts if self.max_grasp_attempts is not None else 1
        )
        for attempt_index in range(attempt_count):
            self.add_subplan(self._grasp_attempt_plan()).perform()
            if self._grasp_succeeded():
                self.add_subplan(self._lift_plan()).perform()
                if self._grasp_succeeded():
                    return
            if attempt_index < attempt_count - 1:
                self.add_subplan(
                    execute_single(
                        DetachNode(
                            body=self.object_designator, new_parent=self.world.root
                        )
                    )
                ).perform()
                self.add_subplan(
                    execute_single(
                        MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm)
                    )
                ).perform()
        raise GraspVerificationFailed(
            object_designator=self.object_designator, attempt_count=attempt_count
        )

    @property
    def _action_plan(self) -> PlanNode:
        return sequential(
            children=[
                self._grasp_attempt_plan(),
                self._lift_plan(),
            ],
        )

    @staticmethod
    def pre_condition(
        variables: Dict, context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The gripper with which to grasp the object needs to be free and the object needs
        to be reachable.
        """
        end_effector = ViewManager.get_end_effector_view(
            variables["arm"], context.robot
        )
        return and_(
            GripperIsFree(end_effector),
            IsObjectReachableBy(
                context=Context(
                    robot=context.robot,
                    world=context.world,
                    alternative_motion_mappings=context.alternative_motion_mappings,
                ),
                arm=variables["arm"],
                object_designator=kwargs["object_designator"],
                grasp_description=kwargs["grasp_description"],
            ),
        )

    @staticmethod
    def post_condition(
        variables: Dict, context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The object needs to be in the gripper frame.
        """
        end_effector = ViewManager.get_end_effector_view(
            variables["arm"], context.robot
        )
        return or_(
            not_(GripperIsFree(end_effector)),
            is_body_in_gripper(variable_from(kwargs["object_designator"]), end_effector)
            > GRASP_DETECTION_THRESHOLD,
        )


@dataclass
class GraspingAction(ActionDescription):
    """
    Grasps an object described by the given Object Designator description.
    """

    object_designator: Body
    """
    Object Designator for the object that should be grasped.
    """

    arm: Arms
    """
    The arm that should be used to grasp.
    """

    grasp_description: GraspDescription
    """
    The grasp description that should be used to grasp the object.
    """

    @property
    def _action_plan(self) -> PlanNode:
        pre_pose, grasp_pose, _ = self.grasp_description.grasp_pose_sequence(
            self.object_designator
        )

        return sequential(
            [
                MoveToolCenterPointMotion(pre_pose, self.arm),
                MoveGripperMotion(GripperState.OPEN, self.arm),
                MoveToolCenterPointMotion(
                    grasp_pose, self.arm, allow_gripper_collision=True
                ),
                MoveGripperMotion(
                    GripperState.CLOSE, self.arm, allow_gripper_collision=True
                ),
            ]
        )
