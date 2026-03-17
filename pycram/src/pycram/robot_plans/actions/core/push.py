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


@dataclass
class PushAction(ActionDescription):
    """
    Pushes or slides the TCP from a start pose to an end pose in a controlled,
    linear Cartesian motion.

    Typical use-cases:
    - Sliding a part into a rail or fixture.
    - Pushing an object across a surface.
    - Pressing a connector into place.

    Sequence:
    1. Approach pre-start (Z + approach_height) to clear obstacles.
    2. Move to start pose.
    3. Linear Cartesian push to end pose.
    4. Retreat to pre-end (Z + approach_height).
    """

    start_pose: PoseStamped
    """Pose at the beginning of the push stroke."""

    end_pose: PoseStamped
    """Pose at the end of the push stroke."""

    arm: Arms
    """Arm to use for the push."""

    approach_height: float = 0.10
    """Height above start/end poses at which to approach (m)."""

    def _offset_z(self, pose: PoseStamped, dz: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = pose.header.frame_id
        ps.pose.position.x = pose.pose.position.x
        ps.pose.position.y = pose.pose.position.y
        ps.pose.position.z = pose.pose.position.z + dz
        ps.pose.orientation = pose.pose.orientation
        return ps

    def execute(self) -> None:
        pre_start = self._offset_z(self.start_pose, self.approach_height)
        pre_end = self._offset_z(self.end_pose, self.approach_height)

        SequentialPlan(
            self.context,
            # Approach start from above
            MoveTCPMotion(target=pre_start, arm=self.arm, movement_type=MovementType.CARTESIAN),
            # Move to stroke start
            MoveTCPMotion(target=self.start_pose, arm=self.arm, movement_type=MovementType.CARTESIAN),
            # Push to end (linear Cartesian)
            MoveTCPMotion(target=self.end_pose, arm=self.arm, movement_type=MovementType.TRANSLATION),
            # Retreat from end
            MoveTCPMotion(target=pre_end, arm=self.arm, movement_type=MovementType.CARTESIAN),
        ).perform()

    def validate_precondition(self):
        pass

    def validate_postcondition(self, result: Optional[Any] = None):
        pass

    @classmethod
    def description(
        cls,
        start_pose: Union[Iterable[PoseStamped], PoseStamped],
        end_pose: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms],
        approach_height: Union[Iterable[float], float] = 0.10,
    ) -> PartialDesignator["PushAction"]:
        return PartialDesignator[PushAction](
            PushAction,
            start_pose=start_pose,
            end_pose=end_pose,
            arm=arm,
            approach_height=approach_height,
        )


PushActionDescription = PushAction.description
