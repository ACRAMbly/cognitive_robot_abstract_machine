from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Union, Any, Iterable

from ....datastructures.enums import Arms, MovementType
from ....datastructures.grasp import GraspDescription
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....language import SequentialPlan
from ....robot_plans.actions.base import ActionDescription
from ....robot_plans.motions.gripper import (
    MoveGripperMotion,
    MoveTCPMotion,
    ReachMotion,
)


@dataclass
class RegraspAction(ActionDescription):
    """
    Regrasp an object from a different angle or grasp pose.

    This composite action handles the common scenario where an object was picked
    up in a sub-optimal orientation and needs to be set down temporarily on a
    regrasping fixture (or the table) before being re-picked with a better grasp.

    Sequence:
    1. Move to the regrasp fixture pose (PlaceMotion equivalent — reach + open + detach).
    2. Retreat from the fixture.
    3. Re-pick with the new grasp description (reach + close + lift).

    For simplicity this action reuses `ReachMotion` + `MoveGripperMotion` directly,
    following the same pattern as `PickUpAction` and `PlaceAction`.
    """

    object_designator: Body
    """The object to be regrasped."""

    intermediate_place_pose: PoseStamped
    """Temporary rest pose where the object is set down for the regrasp."""

    new_grasp_description: GraspDescription
    """Grasp description for the second pick."""

    arm: Arms = Arms.RIGHT
    """Arm to use for the regrasp."""

    def execute(self) -> None:
        # Phase 1: Place at intermediate location (open and detach)
        SequentialPlan(
            self.context,
            ReachMotion(
                object_designator=self.object_designator,
                arm=self.arm,
                grasp_description=self.new_grasp_description,
                reverse_pose_sequence=True,  # Retreat-style motion to place pose
            ),
            MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm),
        ).perform()

        # Kinematically detach from arm
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

        # Phase 2: Re-approach with new grasp description to pick again
        tip_view = __import__(
            "pycram.view_manager", fromlist=["ViewManager"]
        ).ViewManager.get_end_effector_view(self.arm, self.robot_view)

        SequentialPlan(
            self.context,
            ReachMotion(
                object_designator=self.object_designator,
                arm=self.arm,
                grasp_description=self.new_grasp_description,
            ),
            MoveGripperMotion(motion=GripperState.CLOSE, gripper=self.arm),
        ).perform()

        # Kinematically attach to arm tip
        tip_frame = tip_view.tool_frame
        with self.world.modify_world():
            self.world.move_branch_with_fixed_connection(self.object_designator, tip_frame)

    def validate_precondition(self):
        pass

    def validate_postcondition(self, result: Optional[Any] = None):
        pass

    @classmethod
    def description(
        cls,
        object_designator: Union[Iterable[Body], Body],
        intermediate_place_pose: Union[Iterable[PoseStamped], PoseStamped],
        new_grasp_description: Union[Iterable[GraspDescription], GraspDescription],
        arm: Union[Iterable[Arms], Arms] = Arms.RIGHT,
    ) -> PartialDesignator["RegraspAction"]:
        return PartialDesignator[RegraspAction](
            RegraspAction,
            object_designator=object_designator,
            intermediate_place_pose=intermediate_place_pose,
            new_grasp_description=new_grasp_description,
            arm=arm,
        )


RegraspActionDescription = RegraspAction.description
