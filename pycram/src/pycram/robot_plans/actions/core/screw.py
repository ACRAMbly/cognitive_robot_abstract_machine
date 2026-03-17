from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Union, Any, Iterable

from ....datastructures.enums import Arms, MovementType
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....language import SequentialPlan
from ....robot_plans.actions.base import ActionDescription
from ....robot_plans.motions.dual_arm import HelicalMotion
from ....robot_plans.motions.gripper import MoveGripperMotion, MoveTCPMotion


@dataclass
class ScrewAction(ActionDescription):
    """
    Fastening skill that performs a helical (screw) motion around the tool Z-axis.

    Sequence:
    1. Align above the screw/bolt (Z + approach_height).
    2. Engage — move to contact at the target pose.
    3. Drive — execute helical motion for `rotations` turns.
    4. Release gripper, detach object in world.
    5. Retreat back to approach pose.
    """

    object_designator: Body
    """Designator for the screwdriver or fastener being driven."""

    target_pose: PoseStamped
    """6D pose of the screw head (engagement point)."""

    arm: Arms = Arms.RIGHT
    """Arm holding the screwdriver/fastener."""

    rotations: float = 3.0
    """Number of full rotations to drive."""

    pitch: float = 0.001
    """Axial advancement per revolution (m). Default: 1 mm per turn."""

    approach_height: float = 0.05
    """Height above target to approach from (m)."""

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

        # Phase 1 & 2: align above, then engage
        SequentialPlan(
            self.context,
            MoveTCPMotion(target=approach, arm=self.arm, movement_type=MovementType.CARTESIAN),
            MoveTCPMotion(target=self.target_pose, arm=self.arm, movement_type=MovementType.CARTESIAN),
        ).perform()

        # Phase 3: helical drive
        SequentialPlan(
            self.context,
            HelicalMotion(
                start_pose=self.target_pose,
                arm=self.arm,
                rotations=self.rotations,
                pitch=self.pitch,
            ),
        ).perform()

        # Phase 4: release
        SequentialPlan(
            self.context,
            MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm),
        ).perform()

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

        # Phase 5: retreat
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
        target_pose: Union[Iterable[PoseStamped], PoseStamped],
        arm: Union[Iterable[Arms], Arms] = Arms.RIGHT,
        rotations: Union[Iterable[float], float] = 3.0,
        pitch: Union[Iterable[float], float] = 0.001,
        approach_height: Union[Iterable[float], float] = 0.05,
    ) -> PartialDesignator["ScrewAction"]:
        return PartialDesignator[ScrewAction](
            ScrewAction,
            object_designator=object_designator,
            target_pose=target_pose,
            arm=arm,
            rotations=rotations,
            pitch=pitch,
            approach_height=approach_height,
        )


ScrewActionDescription = ScrewAction.description
