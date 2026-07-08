"""
tracy_motion_mapping.py
========================
Coraplex alternative motion mapping for Tracy grippers in REAL execution mode.

Tracy's current gripper action interface uses:
    control_msgs/action/ParallelGripperCommand
with list fields:
    goal.command.position = [position]
    goal.command.effort   = [effort]

Verified command-line shape:
    ros2 action send_goal /left_gripper/robotiq_gripper_controller/gripper_cmd \
      control_msgs/action/ParallelGripperCommand \
      "{command: {position: [0.8], effort: [10.0]}}"
"""


from __future__ import annotations

import logging
from typing import Any

import rclpy
from control_msgs.action import ParallelGripperCommand
from rclpy.action import ActionClient
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.tracy import Tracy

from coraplex.datastructures.enums import Arms, ExecutionType
from coraplex.robot_plans.motions.base import AlternativeMotion
from coraplex.robot_plans.motions.gripper import MoveGripperMotion

logger = logging.getLogger(__name__)


LEFT_GRIPPER_ACTION_TOPIC = "/left_gripper/robotiq_gripper_controller/gripper_cmd"
RIGHT_GRIPPER_ACTION_TOPIC = "/right_gripper/robotiq_gripper_controller/gripper_cmd"

# These are the values you verified with ros2 action send_goal.
GRIPPER_OPEN_POSITION = 0.0
GRIPPER_CLOSE_TO_CUBE_POSITION = 0.8
GRIPPER_EFFORT = 10.0

_client_cache: dict[tuple[int, str], ActionClient] = {}

def _get_client(node: Any, topic: str) -> ActionClient:
    """Return one cached ActionClient per node/topic."""
    key = (id(node), topic)
    if key not in _client_cache:
        _client_cache[key] = ActionClient(node, ParallelGripperCommand, topic)
    return _client_cache[key]

class TracyRealMoveGripperMotion(MoveGripperMotion, AlternativeMotion[Tracy]):
    """
    Real-robot override of MoveGripperMotion for Tracy.

    MoveGripperMotion.perform() calls this alternative's perform() directly.

    Therefore the ROS2 ParallelGripperCommand action is sent here, not through _motion_chart_.
    """

    execution_type = ExecutionType.REAL

    _POSITION_MAP = {
        GripperState.OPEN: GRIPPER_OPEN_POSITION,
        GripperState.CLOSE: GRIPPER_CLOSE_TO_CUBE_POSITION,
    }

    def perform(self) -> None:
        side = "right" if self.gripper == Arms.RIGHT else "left"
        topic = RIGHT_GRIPPER_ACTION_TOPIC if self.gripper == Arms.RIGHT else LEFT_GRIPPER_ACTION_TOPIC
        position = float(self._POSITION_MAP[self.motion])
        effort = float(GRIPPER_EFFORT)

        node = self.plan.context.ros_node
        client = _get_client(node, topic)

        logger.info(
            "[TracyGripper] Sending %s gripper command: position=%s effort=%s topic=%s",
            side.upper(),
            position,
            effort,
            topic,
        )

        goal = ParallelGripperCommand.Goal()
        goal.command.position = [position]
        goal.command.effort = [effort]

        if not client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f"Gripper action server not available: {topic}")

        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, goal_future, timeout_sec=5.0)

        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"Gripper goal rejected: {topic}")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)

        if result_future.result() is None:
            raise RuntimeError(f"Gripper result timeout: {topic}")