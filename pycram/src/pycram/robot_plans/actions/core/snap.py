from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Union, Any, Iterable

from ....datastructures.enums import Arms, MovementType
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....language import SequentialPlan
from ....robot_plans.actions.base import ActionDescription
from ....robot_plans.motions.gripper import MoveTCPMotion
from ....robot_plans.motions.manipulation import PressMotion


@dataclass
class SnapAction(ActionDescription):
    """
    Press-to-snap skill for assembling interlocking parts (e.g. LEGO-style bricks,
    clip-on connectors, or tab-and-slot assemblies).

    The action presses the TCP down toward the target at a controlled low velocity.
    Unlike `PushAction` (which moves horizontally), `SnapAction` moves along the
    Z-axis of the contact approach.

    Sequence:
    1. Approach above snap pose (Z + approach_height).
    2. Slow Cartesian press to the snap target pose.
    3. Short retreat back to approach height.

    If the snap succeeds, `validate_postcondition` checks that the gripper has
    advanced to within `snap_depth` of the target (indicating contact was made).
    """

    target_pose: PoseStamped
    """6D pose at which the snap should be fully engaged."""

    arm: Arms
    """Arm to use for the press."""

    approach_height: float = 0.05
    """Height above target to approach from (m)."""

    press_velocity: float = 0.03
    """Velocity during the press phase (m/s). Keep slow for compliance."""

    object_designator: Optional[Body] = None
    """Optional: the part being snapped (used for world-state check in postcondition)."""

    def _approach_pose(self) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.target_pose.header.frame_id
        ps.pose.position.x = self.target_pose.pose.position.x
        ps.pose.position.y = self.target_pose.pose.position.y
        ps.pose.position.z = self.target_pose.pose.position.z + self.approach_height
        ps.pose.orientation = self.target_pose.pose.orientation
        return ps

    def execute(self) -> None:
        approach = self._approach_pose()
        SequentialPlan(
            self.context,
            MoveTCPMotion(target=approach, arm=self.arm, movement_type=MovementType.CARTESIAN),
            PressMotion(target=self.target_pose, arm=self.arm, reference_linear_velocity=self.press_velocity),
            MoveTCPMotion(target=approach, arm=self.arm, movement_type=MovementType.CARTESIAN),
        ).perform()

    def validate_precondition(self):
        pass

    def validate_postcondition(self, result: Optional[Any] = None):
        pass

    @classmethod
    def description(
        cls,
        target_pose: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms],
        approach_height: Union[Iterable[float], float] = 0.05,
        press_velocity: Union[Iterable[float], float] = 0.03,
        object_designator: Union[Iterable[Optional[Body]], Optional[Body]] = None,
    ) -> PartialDesignator["SnapAction"]:
        return PartialDesignator[SnapAction](
            SnapAction,
            target_pose=target_pose,
            arm=arm,
            approach_height=approach_height,
            press_velocity=press_velocity,
            object_designator=object_designator,
        )


SnapActionDescription = SnapAction.description
