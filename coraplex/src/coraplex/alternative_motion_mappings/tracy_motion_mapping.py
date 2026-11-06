from __future__ import annotations

from dataclasses import dataclass

from control_msgs.action import ParallelGripperCommand


from giskardpy.motion_statechart.ros2_nodes.ros_tasks import ActionServerTask
from giskardpy.motion_statechart.graph_node import NodeArtifacts
from krrood.symbolic_math.symbolic_math import Scalar

LEFT_GRIPPER_ACTION_TOPIC = "/left_gripper/robotiq_gripper_controller/gripper_cmd"
RIGHT_GRIPPER_ACTION_TOPIC = "/right_gripper/robotiq_gripper_controller/gripper_cmd"

GRIPPER_EFFORT = 0.0

@dataclass(eq=False, repr=False)
class ParallelGripperCommandActionServerTask(ActionServerTask):
    position: float= 0.35
    effort: float = 0.0


    def __eq__(self, other: object) -> bool:
        return self is other

    def __hash__(self) -> int:
        return id(self)

    def build_action_goal(self, context):
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [float(self.position)]
        goal.command.effort = [float(self.effort)]
        self._msg = goal

    def goal_response_callback(self, future):
        result = super().goal_response_callback(future)
        return result

    def result_callback(self, future):
        result = super().result_callback(future)
        return result