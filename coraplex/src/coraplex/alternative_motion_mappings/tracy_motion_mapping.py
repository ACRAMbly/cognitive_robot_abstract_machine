from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from control_msgs.action import ParallelGripperCommand

from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.tracy import Tracy

from coraplex.datastructures.enums import Arms, ExecutionType
from coraplex.robot_plans.motions.base import AlternativeMotion
from coraplex.robot_plans.motions.gripper import MoveGripperMotion

from giskardpy.motion_statechart.ros2_nodes.ros_tasks import ActionServerTask


LEFT_GRIPPER_ACTION_TOPIC = "/left_gripper/robotiq_gripper_controller/gripper_cmd"
RIGHT_GRIPPER_ACTION_TOPIC = "/right_gripper/robotiq_gripper_controller/gripper_cmd"

GRIPPER_OPEN_POSITION = 0.0
GRIPPER_CLOSE_TO_CUBE_POSITION = 0.35
GRIPPER_EFFORT = 10.0


@dataclass(eq=False, repr=False)
class ParallelGripperCommandActionServerTask(ActionServerTask):
    """
    Giskard task that sends the same goal as:

    ros2 action send_goal /left_gripper/robotiq_gripper_controller/gripper_cmd \
      control_msgs/action/ParallelGripperCommand \
      "{command: {position: [0.35], effort: [10.0]}}"
    """

    position: float
    effort: float = GRIPPER_EFFORT

    def __eq__(self, other: object) -> bool:
        # Prevent dataclass/Giskard equality from touching internal unbuilt fields.
        return self is other

    def __hash__(self) -> int:
        return id(self)

    def build_msg(self, context):
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [float(self.position)]
        goal.command.effort = [float(self.effort)]
        self._msg = goal


class TracyRealMoveGripperMotion(MoveGripperMotion, AlternativeMotion[Tracy]):
    """
    Real Tracy gripper mapping.

    In current execution path, Coraplex is calling motion_chart/_motion_chart,
    so the real gripper command must be represented as a Giskard ActionServerTask.
    """

    execution_type: ClassVar[ExecutionType] = ExecutionType.SIMULATED

    _POSITION_MAP = {
        GripperState.OPEN: GRIPPER_OPEN_POSITION,
        GripperState.CLOSE: GRIPPER_CLOSE_TO_CUBE_POSITION,
    }

    def perform(self) -> None:
        pass

    @property
    def _motion_chart(self) -> ParallelGripperCommandActionServerTask:
        print("[TEST] TracyRealMoveGripperMotion._motion_chart called")

        side = "right" if self.gripper == Arms.RIGHT else "left"

        topic = (
            RIGHT_GRIPPER_ACTION_TOPIC
            if self.gripper == Arms.RIGHT
            else LEFT_GRIPPER_ACTION_TOPIC
        )

        position = self._POSITION_MAP[self.motion]

        print(
            "[TEST] Creating ParallelGripperCommandActionServerTask:",
            "side=", side,
            "topic=", topic,
            "position=", position,
            "effort=", GRIPPER_EFFORT,
        )

        return ParallelGripperCommandActionServerTask(
            action_topic=topic,
            message_type=ParallelGripperCommand,
            position=position,
            effort=GRIPPER_EFFORT,
            name=f"Tracy_{side}_parallel_gripper",
        )