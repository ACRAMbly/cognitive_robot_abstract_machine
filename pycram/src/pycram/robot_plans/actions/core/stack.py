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
from ....robot_plans.motions.gripper import MoveGripperMotion, MoveTCPMotion


@dataclass
class StackAction(ActionDescription):
    """
    Places one object precisely on top of another by computing a target Z
    position from the base object's bounding-box height in the world model.

    Typical use-case: stacking toy blocks, placing lids on containers,
    assembling layered structures.

    Sequence:
    1. Compute stack target pose = base object AABB top + `stack_offset`.
    2. Approach above target (Z + approach_height).
    3. Move to target pose (slow Cartesian).
    4. Open gripper and detach object in world.
    5. Retreat back to approach height.
    """

    stack_object: Body
    """The object being placed (currently held by the arm)."""

    base_object: Body
    """The object to stack on top of."""

    arm: Arms
    """Arm holding the stack object."""

    approach_height: float = 0.10
    """Height above the computed stack pose to approach from (m)."""

    stack_offset: float = 0.002
    """Extra clearance above the base object AABB top (m). Default 2 mm."""

    place_velocity: float = 0.05
    """Velocity for the descent phase (m/s)."""

    grasp_description: Optional[GraspDescription] = None
    """Optional: used only to inherit orientation from original grasp."""

    def _compute_stack_pose(self) -> PoseStamped:
        """
        Derive the stacking pose from the base object's axis-aligned bounding
        box in world coordinates.
        """
        # Get the AABB of the base object in the world frame
        try:
            aabb = self.world.get_object_aabb(self.base_object)
            top_z = aabb.max_z + self.stack_offset
        except (AttributeError, Exception):
            # Fallback: use forward kinematics to estimate centre + rough half-height
            base_tf = self.world.compute_forward_kinematics(self.world.root, self.base_object)
            # Approx: assume 0.05m half-height
            top_z = float(base_tf.to_position().z) + 0.05 + self.stack_offset

        # Get XY from the base object's world position
        base_tf = self.world.compute_forward_kinematics(self.world.root, self.base_object)
        cx = float(base_tf.to_position().x)
        cy = float(base_tf.to_position().y)

        ps = PoseStamped()
        ps.header.frame_id = str(self.world.root.name)
        ps.pose.position.x = cx
        ps.pose.position.y = cy
        ps.pose.position.z = top_z
        ps.pose.orientation.w = 1.0
        return ps

    def _offset_z(self, pose: PoseStamped, dz: float) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = pose.header.frame_id
        ps.pose.position.x = pose.pose.position.x
        ps.pose.position.y = pose.pose.position.y
        ps.pose.position.z = pose.pose.position.z + dz
        ps.pose.orientation = pose.pose.orientation
        return ps

    def execute(self) -> None:
        target = self._compute_stack_pose()
        approach = self._offset_z(target, self.approach_height)

        SequentialPlan(
            self.context,
            # Approach from above
            MoveTCPMotion(target=approach, arm=self.arm, movement_type=MovementType.CARTESIAN),
            # Descend to stack pose
            MoveTCPMotion(target=target, arm=self.arm, movement_type=MovementType.CARTESIAN),
            # Release
            MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm),
        ).perform()

        # Kinematically detach from arm, re-attach to world (let base support it)
        world_root = self.world.root
        obj_tf = self.world.compute_forward_kinematics(world_root, self.stack_object)
        from semantic_digital_twin.world_description.connections import FixedConnection
        with self.world.modify_world():
            self.world.remove_connection(self.stack_object.parent_connection)
            connection = FixedConnection(
                parent=world_root,
                child=self.stack_object,
                parent_T_connection_expression=obj_tf,
            )
            self.world.add_connection(connection)

        # Retreat
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
        stack_object: Union[Iterable[Body], Body],
        base_object: Union[Iterable[Body], Body],
        arm: Union[Iterable[Arms], Arms],
        approach_height: Union[Iterable[float], float] = 0.10,
        stack_offset: Union[Iterable[float], float] = 0.002,
        place_velocity: Union[Iterable[float], float] = 0.05,
    ) -> PartialDesignator["StackAction"]:
        return PartialDesignator[StackAction](
            StackAction,
            stack_object=stack_object,
            base_object=base_object,
            arm=arm,
            approach_height=approach_height,
            stack_offset=stack_offset,
            place_velocity=place_velocity,
        )


StackActionDescription = StackAction.description
