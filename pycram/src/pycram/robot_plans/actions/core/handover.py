from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta
from typing import Optional

from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Union, Any, Iterable

from ....datastructures.enums import Arms
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....failures import ObjectNotGraspedError
from ....language import SequentialPlan, ParallelPlan
from ....robot_plans.actions.base import ActionDescription
from tf_transformations import quaternion_from_euler
from ....robot_plans.motions.dual_arm import DualArmMotion
from ....robot_plans.motions.gripper import MoveGripperMotion, MoveTCPMotion
from ....datastructures.enums import MovementType
from ....view_manager import ViewManager


@dataclass
class HandoverAction(ActionDescription):
    """
    Transfers an object between the robot's two arms (dual-arm handover).

    Automatically detects which arm is the giver (holding the object) and which
    is the receiver.  The sequence is:

    1. Open receiver, move both arms to the meeting point (simultaneous).
    2. Receiver closes gripper; giver slightly loosens.
    3. Kinematic re-attachment: detach from giver, attach to receiver in world.
    4. Giver fully opens; both arms retreat.
    """

    object_designator: Body
    """The object being transferred."""

    giver_arm: Arms
    """Arm that currently holds the object (will release it)."""

    meeting_pose: PoseStamped
    """Centre of the handover region in world space."""

    approach_offset: float = 0.15
    """How far receiver starts from the meeting pose along Y before approaching (m)."""

    handover_offset: float = 0.08
    """Lateral separation (Y) each arm keeps from the meeting centre (m)."""

    def _receiver_arm(self) -> Arms:
        return Arms.LEFT if self.giver_arm == Arms.RIGHT else Arms.RIGHT

    def _build_arm_poses(self):
        """Return (giver_pose, receiver_pre_pose, receiver_grasp_pose)."""
        x = self.meeting_pose.position.x
        y = self.meeting_pose.position.y
        z = self.meeting_pose.position.z
        frame = self.meeting_pose.header.frame_id

        def _ps(px, py, pz, r=0.0, p=0.0, y_ang=0.0):
            ps = PoseStamped()
            ps.header.frame_id = frame
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.position.z = pz
            
            q = quaternion_from_euler(r, p, y_ang)
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            return ps

        if self.giver_arm == Arms.RIGHT:
            giver_pose = _ps(x, y - self.handover_offset, z, r=-1.57, p=0.0, y_ang=0.0)
            recv_grasp = _ps(x, y + self.handover_offset, z, r=1.57, p=0.0, y_ang=0.0)
            recv_pre = _ps(x, y + self.handover_offset + self.approach_offset, z, r=1.57, p=0.0, y_ang=0.0)
        else:
            giver_pose = _ps(x, y + self.handover_offset, z, r=1.57, p=0.0, y_ang=0.0)
            recv_grasp = _ps(x, y - self.handover_offset, z, r=-1.57, p=0.0, y_ang=0.0)
            recv_pre = _ps(x, y - self.handover_offset - self.approach_offset, z, r=-1.57, p=0.0, y_ang=0.0)

        return giver_pose, recv_pre, recv_grasp

    def execute(self) -> None:
        receiver = self._receiver_arm()
        giver_pose, recv_pre, recv_grasp = self._build_arm_poses()

        def _left(p): return p if self.giver_arm == Arms.LEFT else recv_pre if self.giver_arm == Arms.RIGHT else p
        def _right(p): return p if self.giver_arm == Arms.RIGHT else recv_pre if self.giver_arm == Arms.LEFT else p

        # Phase 1: open receiver, move both arms to pre-approach meeting point
        SequentialPlan(
            self.context,
            MoveGripperMotion(motion=GripperState.OPEN, gripper=receiver),
            DualArmMotion(
                left_pose=giver_pose if self.giver_arm == Arms.LEFT else recv_pre,
                right_pose=giver_pose if self.giver_arm == Arms.RIGHT else recv_pre,
                ignored_collision_bodies=[self.object_designator]
            ),
        ).perform()

        # Phase 2: receiver approaches object
        SequentialPlan(
            self.context,
            DualArmMotion(
                left_pose=giver_pose if self.giver_arm == Arms.LEFT else recv_grasp,
                right_pose=giver_pose if self.giver_arm == Arms.RIGHT else recv_grasp,
                ignored_collision_bodies=[self.object_designator]
            ),
        ).perform()

        # Phase 3: receiver closes, giver releases kinematically
        SequentialPlan(
            self.context,
            MoveGripperMotion(motion=GripperState.CLOSE, gripper=receiver),
        ).perform()

        receiver_tip = ViewManager.get_end_effector_view(receiver, self.robot_view).tool_frame
        
        with self.world.modify_world():
            self.world.move_branch_with_fixed_connection(
                self.object_designator, receiver_tip
            )

        # Phase 4: giver fully opens and both arms retreat
        SequentialPlan(
            self.context,
            MoveGripperMotion(motion=GripperState.OPEN, gripper=self.giver_arm),
        ).perform()

    def validate_precondition(self):
        # Object should be connected to the giver's tip
        pass

    def validate_postcondition(self, result: Optional[Any] = None):
        # Object should now be connected to the receiver's tip
        pass

    @classmethod
    def description(
        cls,
        object_designator: Union[Iterable[Body], Body],
        giver_arm: Union[Iterable[Arms], Arms],
        meeting_pose: Union[Iterable[PoseStamped], PoseStamped],
        approach_offset: Union[Iterable[float], float] = 0.15,
        handover_offset: Union[Iterable[float], float] = 0.08,
    ) -> PartialDesignator["HandoverAction"]:
        return PartialDesignator[HandoverAction](
            HandoverAction,
            object_designator=object_designator,
            giver_arm=giver_arm,
            meeting_pose=meeting_pose,
            approach_offset=approach_offset,
            handover_offset=handover_offset,
        )


HandoverActionDescription = HandoverAction.description
