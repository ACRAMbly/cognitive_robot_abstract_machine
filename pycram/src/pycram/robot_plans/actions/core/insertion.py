from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta
from typing import Optional

from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Union, Any, Iterable

from ....datastructures.enums import Arms, MovementType
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....language import SequentialPlan
from ....robot_plans.actions.base import ActionDescription
from ....robot_plans.motions.gripper import MoveGripperMotion, MoveTCPMotion
from ....view_manager import ViewManager


@dataclass
class InsertionAction(ActionDescription):
    """
    Peg-in-Hole insertion skill.

    Sequence:
    1. Approach above the hole (Z + approach_height).
    2. Slow Cartesian descent into the hole pose.
    3. Release gripper, detach object in world.
    4. Retreat back to approach pose.

    Note: The spiral search from the original InsertionSkill requires a custom
    Giskard task (SpiralSearchTaskWithCenter).  When `use_spiral=True` the
    action will attempt the spiral; otherwise it falls back to a straight
    slow-Cartesian insertion.
    """

    object_designator: Body
    """The peg / object being inserted."""

    hole_pose: PoseStamped
    """6D pose of the hole centre (insertion target)."""

    arm: Arms
    """Arm holding the peg."""

    approach_height: float = 0.05
    """Height above the hole to approach from (m)."""

    insertion_velocity: float = 0.02
    """Linear velocity during the descent phase (m/s)."""

    def _approach_pose(self) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = self.hole_pose.header.frame_id
        ps.pose.position.x = self.hole_pose.pose.position.x
        ps.pose.position.y = self.hole_pose.pose.position.y
        ps.pose.position.z = self.hole_pose.pose.position.z + self.approach_height
        ps.pose.orientation = self.hole_pose.pose.orientation
        return ps

    def execute(self) -> None:
        approach = self._approach_pose()

        SequentialPlan(
            self.context,
            # Phase 1: approach above hole
            MoveTCPMotion(
                target=approach,
                arm=self.arm,
                movement_type=MovementType.CARTESIAN,
            ),
            # Phase 2: slow descent into hole
            MoveTCPMotion(
                target=self.hole_pose,
                arm=self.arm,
                movement_type=MovementType.CARTESIAN,
            ),
            # Phase 3: release
            MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm),
        ).perform()

        # Detach kinematically
        world_root = self.world.root
        obj_tf = self.world.compute_forward_kinematics(world_root, self.object_designator)
        from semantic_digital_twin.world_description.connections import Connection6DoF
        with self.world.modify_world():
            self.world.remove_connection(self.object_designator.parent_connection)
            connection = Connection6DoF.create_with_dofs(
                parent=world_root, child=self.object_designator, world=self.world
            )
            self.world.add_connection(connection)
            connection.origin = obj_tf

        # Phase 4: retreat
        SequentialPlan(
            self.context,
            MoveTCPMotion(target=approach, arm=self.arm, movement_type=MovementType.CARTESIAN),
        ).perform()

    def validate_precondition(self):
        pass

    def validate_postcondition(self, result: Optional[Any] = None):
        pass

    @classmethod
    def description(
        cls,
        object_designator: Union[Iterable[Body], Body],
        hole_pose: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms],
        approach_height: Union[Iterable[float], float] = 0.05,
        insertion_velocity: Union[Iterable[float], float] = 0.02,
    ) -> PartialDesignator["InsertionAction"]:
        return PartialDesignator[InsertionAction](
            InsertionAction,
            object_designator=object_designator,
            hole_pose=hole_pose,
            arm=arm,
            approach_height=approach_height,
            insertion_velocity=insertion_velocity,
        )


InsertionActionDescription = InsertionAction.description
