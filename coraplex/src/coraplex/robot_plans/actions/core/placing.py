from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import Any, Dict, Optional

from coraplex.plans.attachment_nodes import DetachNode
from coraplex.plans.plan_node import PlanNode
from krrood.entity_query_language.core.variable import Variable
from krrood.entity_query_language.factories import (
    or_,
    not_,
    and_,
    variable_from,
    ConditionType,
)
from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import (
    Arms,
    ApproachDirection,
    VerticalAlignment,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.plans.factories import sequential, execute_single
from coraplex.querying.predicates import GripperIsFree
from coraplex.robot_plans.actions.base import ActionDescription
from coraplex.robot_plans.actions.core.pick_up import (
    GRASP_DETECTION_THRESHOLD,
    PickUpAction,
)
from coraplex.robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveToolCenterPointMotion,
)
from coraplex.view_manager import ViewManager
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.reasoning.predicates import allclose
from semantic_digital_twin.reasoning.robot_predicates import is_body_gripped
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.world_entity import Body

GRASP_RELEASE_THRESHOLD: float = 0.1
"""
Maximum fraction of sampled rays between the gripper's fingers that may still hit the
object for it to be considered released (see
:func:`~semantic_digital_twin.reasoning.robot_predicates.is_body_gripped`).
"""


@dataclass
class PlaceAction(ActionDescription):
    """
    Places an Object at a position using an arm.
    """

    object_designator: Body
    """
    Object designator_description describing the object that should be place
    """
    target_location: Pose
    """
    Pose in the world at which the object should be placed.
    """

    arm: Arms
    """
    Arm that is currently holding the object
    """

    placing_linear_velocity: Optional[float] = None
    """
    Linear reference velocity (in m/s) for the final descent onto the target location.
    ``None`` keeps the default velocity.
    """

    transport_linear_velocity: Optional[float] = None
    """
    Linear reference velocity (in m/s) for carrying the held object above the target
    location, before the final descent. ``None`` keeps the default velocity.
    """

    release_opening_velocity: Optional[float] = None
    """
    Finger joint velocity (in m/s) used while opening the gripper to release the
    object. ``None`` keeps the default velocity.
    """

    retract_linear_velocity: Optional[float] = None
    """
    Linear reference velocity (in m/s) for retracting the end effector away from the
    placed object. ``None`` keeps the default velocity.
    """

    max_release_attempts: Optional[int] = None
    """
    How many times to (re-)issue the OPEN motion, checking afterwards whether the
    object is released, before retracting regardless. ``None`` keeps the original
    single-attempt behaviour.
    """

    grasp_detection_threshold: float = GRASP_DETECTION_THRESHOLD
    """
    Minimum fraction of sampled rays between the gripper's fingers that must hit
    :attr:`object_designator` for it to still count as held (see
    :func:`~semantic_digital_twin.reasoning.robot_predicates.is_body_gripped`).
    """

    grasp_release_threshold: float = GRASP_RELEASE_THRESHOLD
    """
    Maximum fraction of sampled rays between the gripper's fingers that may still hit
    :attr:`object_designator` for it to count as released (see
    :func:`~semantic_digital_twin.reasoning.robot_predicates.is_body_gripped`).
    """

    def _previous_grasp(self) -> GraspDescription:
        """
        The grasp description used by the preceding :class:`PickUpAction` on this
        object, or a default front/no-alignment grasp if none is found.
        """
        end_effector = ViewManager.get_arm_view(self.arm, self.robot).end_effector
        previous_pick = self.plan_node.get_previous_node_by_designator_type(
            PickUpAction
        )
        if previous_pick:
            return previous_pick.designator.grasp_description
        return GraspDescription(
            ApproachDirection.FRONT, VerticalAlignment.NoAlignment, end_effector
        )

    def _pose_sequence(self) -> tuple[Pose, Pose, Pose]:
        """
        :return: The (transport, placing, retract) poses for this placement.
        """
        return self._previous_grasp().pose_sequence(
            self.target_location, self.object_designator, reverse=True
        )

    def _transport_and_descend_plan(
        self, transport_pose: Pose, placing_pose: Pose
    ) -> PlanNode:
        """
        :return: The plan that carries the held object above the target location and
            then descends onto it.
        """
        return sequential(
            [
                MoveToolCenterPointMotion(
                    transport_pose,
                    self.arm,
                    allow_gripper_collision=False,
                    reference_linear_velocity=self.transport_linear_velocity,
                ),
                MoveToolCenterPointMotion(
                    placing_pose,
                    self.arm,
                    allow_gripper_collision=False,
                    reference_linear_velocity=self.placing_linear_velocity,
                ),
            ],
        )

    def _retract_plan(self, retract_pose: Pose) -> PlanNode:
        """
        :return: The plan that re-parents the placed object back to the world and
            retracts the end effector away from it.
        """
        return sequential(
            [
                DetachNode(body=self.object_designator, new_parent=self.world.root),
                MoveToolCenterPointMotion(
                    retract_pose,
                    self.arm,
                    reference_linear_velocity=self.retract_linear_velocity,
                ),
            ],
        )

    def _object_released(self) -> bool:
        """
        :return: Whether :attr:`object_designator` is no longer detected between the
            gripper's fingers (see :attr:`grasp_release_threshold`).
        """
        end_effector = ViewManager.get_end_effector_view(self.arm, self.robot)
        return not is_body_gripped(
            self.object_designator, end_effector, threshold=self.grasp_release_threshold
        )

    def execute(self) -> Any:
        """
        Transport and descend to the target location, then release
        :attr:`object_designator`, retrying the OPEN motion up to
        :attr:`max_release_attempts` times before retracting.
        """
        transport_pose, placing_pose, retract_pose = self._pose_sequence()

        self.add_subplan(
            self._transport_and_descend_plan(transport_pose, placing_pose)
        ).perform()

        release_attempt_count = (
            self.max_release_attempts if self.max_release_attempts is not None else 1
        )
        for _ in range(release_attempt_count):
            self.add_subplan(
                execute_single(
                    MoveGripperMotion(
                        GripperState.OPEN,
                        self.arm,
                        finger_velocity=self.release_opening_velocity,
                    )
                )
            ).perform()
            if self._object_released():
                break

        self.add_subplan(self._retract_plan(retract_pose)).perform()

    @property
    def _action_plan(self) -> PlanNode:
        transport_pose, placing_pose, retract_pose = self._pose_sequence()

        return sequential(
            [
                self._transport_and_descend_plan(transport_pose, placing_pose),
                MoveGripperMotion(
                    GripperState.OPEN,
                    self.arm,
                    finger_velocity=self.release_opening_velocity,
                ),
                self._retract_plan(retract_pose),
            ],
            self.context,
        )

    @staticmethod
    def pre_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The object needs to be in the gripper frame.
        """
        end_effector = ViewManager.get_end_effector_view(
            variables["arm"], context.robot
        )
        return or_(
            not_(GripperIsFree(end_effector)),
            is_body_gripped(
                variable_from(kwargs["object_designator"]),
                end_effector,
                threshold=kwargs["grasp_detection_threshold"],
            ),
        )

    @staticmethod
    def post_condition(
        variables: Dict[str, Variable], context: Context, kwargs: Dict[str, Any]
    ) -> ConditionType:
        """
        The gripper must be free again and the object needs to be at the target
        location.
        """
        end_effector = ViewManager.get_end_effector_view(
            variables["arm"], context.robot
        )
        return and_(
            GripperIsFree(end_effector),
            not_(
                is_body_gripped(
                    variable_from(kwargs["object_designator"]),
                    end_effector,
                    threshold=kwargs["grasp_release_threshold"],
                )
            ),
            allclose(
                variable_from(kwargs["object_designator"]).global_pose,
                kwargs["target_location"],
                atol=0.03,
            ),
        )
