"""
tracy_motion_mapping.py
=======================

Coraplex alternative motion mapping for Tracy's real Robotiq parallel grippers.

This overrides:

    MoveGripperMotion -> TracyRealMoveGripperMotion

for:

    robot = Tracy
    execution_type = REAL
For closing, `stalled=True` is accepted as contact/grasp.

Usage (in any demo / script):
    import coraplex.alternative_motion_mappings.tracy_motion_mapping  # noqa: F401
    That single import registers the alternative.  Nothing else is needed.
    When executing inside `with real_robot:`, every MoveGripperMotion issued
    for a Tracy robot will automatically use TracyRealMoveGripperMotion.
"""

from __future__ import annotations

import logging
import time

from rclpy.action import ActionClient
from control_msgs.action import ParallelGripperCommand

from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.tracy import Tracy

from coraplex.datastructures.enums import ExecutionType, Arms
from coraplex.robot_plans.motions.base import AlternativeMotion
from coraplex.robot_plans.motions.gripper import MoveGripperMotion

from coraplex.view_manager import ViewManager
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList


logger = logging.getLogger(__name__)


class _RealParallelGripperClient:
    """
    Thin wrapper around Tracy's real left/right Robotiq gripper action servers.

    Uses the shared ROS node from the Coraplex Context.
    It assumes your demo already has a background rclpy spinner running.
    """

    TOPIC_TEMPLATE = "/{side}_gripper/robotiq_gripper_controller/gripper_cmd"

    # Tested Tracy values
    OPEN_POSITION: float = 0.0
    CLOSE_POSITION: float = 0.8
    DEFAULT_EFFORT: float = 10.0

    WAIT_FOR_SERVER_TIMEOUT: float = 3.0
    GOAL_HANDLE_TIMEOUT: float = 5.0
    RESULT_TIMEOUT: float = 10.0
    DEFAULT_WAIT_AFTER_RESULT: float = 1.0

    def __init__(self, node):
        self._node = node

        # Create both clients immediately.
        # This avoids races with an already-running executor/spinner.
        self._clients: dict[str, ActionClient] = {
            side: ActionClient(
                node,
                ParallelGripperCommand,
                self.TOPIC_TEMPLATE.format(side=side),
            )
            for side in ("left", "right")
        }

        logger.info("[TracyGripper] Created left/right ParallelGripperCommand clients")

    def _get_client(self, side: str) -> ActionClient:
        if side not in self._clients:
            raise ValueError(f"Unknown gripper side: {side!r}. Expected 'left' or 'right'.")
        return self._clients[side]

    def command(
        self,
        side: str,
        position: float,
        effort: float = DEFAULT_EFFORT,
        *,
        closing: bool = False,
        wait_time: float = DEFAULT_WAIT_AFTER_RESULT,
    ) -> None:
        """
        Send one ParallelGripperCommand goal.

        :param side: 'left' or 'right'
        :param position: command position, e.g. 0.0 open, 0.8 close
        :param effort: command effort, e.g. 10.0
        :param closing: True when this is a close/grasp command
        :param wait_time: extra sleep after action result
        """

        client = self._get_client(side)

        if not client.wait_for_server(timeout_sec=self.WAIT_FOR_SERVER_TIMEOUT):
            logger.error(
                "[TracyGripper] Action server not available: %s",
                self.TOPIC_TEMPLATE.format(side=side),
            )
            return

        # ParallelGripperCommand
        # CLI command for close: ros2 action send_goal /left_gripper/robotiq_gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand "{command: {position: [0.8], effort: [10.0]}}"
        # CLI command for open: ros2 action send_goal /left_gripper/robotiq_gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand "{command: {position: [0.0], effort: [10.0]}}"
        goal = ParallelGripperCommand.Goal()
        goal.command.position = [float(position)]
        goal.command.effort = [float(effort)]

        logger.info(
            "[TracyGripper] Sending %s gripper command: position=[%.3f], effort=[%.1f]",
            side.upper(),
            position,
            effort,
        )

        goal_future = client.send_goal_async(goal)

        deadline = time.monotonic() + self.GOAL_HANDLE_TIMEOUT
        while not goal_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)

        if not goal_future.done():
            logger.error("[TracyGripper] Timeout waiting for %s goal handle", side)
            return

        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            logger.error("[TracyGripper] Goal rejected for %s gripper", side)
            return

        result_future = goal_handle.get_result_async()

        deadline = time.monotonic() + self.RESULT_TIMEOUT
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)

        if not result_future.done():
            logger.warning("[TracyGripper] Result timeout for %s gripper; continuing", side)
            time.sleep(wait_time)
            return

        result_response = result_future.result()
        result = result_response.result

        logger.info(
            "[TracyGripper] %s result: position=%s, stalled=%s, reached_goal=%s",
            side.upper(),
            list(result.state.position),
            result.stalled,
            result.reached_goal,
        )

        if closing:
            # For grasping an object, stalled=True often means contact.
            if result.reached_goal or result.stalled:
                logger.info("[TracyGripper] %s close/grasp accepted", side.upper())
            else:
                logger.warning(
                    "[TracyGripper] %s close finished without reaching goal or contact",
                    side.upper(),
                )
        else:
            # For opening, we normally want the open target to be reached.
            if result.reached_goal and not result.stalled:
                logger.info("[TracyGripper] %s open reached goal", side.upper())
            else:
                logger.warning(
                    "[TracyGripper] %s open did not fully reach goal "
                    "(stalled=%s, reached_goal=%s)",
                    side.upper(),
                    result.stalled,
                    result.reached_goal,
                )

        time.sleep(wait_time)


_client_cache: dict[int, _RealParallelGripperClient] = {}


def _get_real_gripper_client(node) -> _RealParallelGripperClient:
    """
    one gripper client wrapper per ROS node.
    """
    key = id(node)

    if key not in _client_cache:
        _client_cache[key] = _RealParallelGripperClient(node)

    return _client_cache[key]


class TracyRealMoveGripperMotion(MoveGripperMotion, AlternativeMotion[Tracy]):
    """
    Real Tracy override for MoveGripperMotion.

    This directly commands the physical gripper in perform(), then returns a no-op
    Giskard JointPositionList in _motion_chart so the rest of the Coraplex/Giskard
    pipeline remains satisfied.
    """

    execution_type = ExecutionType.REAL

    def perform(self) -> None:
        side = "right" if self.gripper == Arms.RIGHT else "left"

        if self.motion == GripperState.OPEN:
            position = _RealParallelGripperClient.OPEN_POSITION
            closing = False

        elif self.motion == GripperState.CLOSE:
            position = _RealParallelGripperClient.CLOSE_POSITION
            closing = True

        else:
            raise ValueError(f"Unsupported Tracy gripper motion: {self.motion!r}")

        node = self.plan.context.ros_node
        if node is None:
            logger.error(
                "[TracyGripper] Context has no ros_node. "
                "Make sure your Coraplex Context was created with ros_node=node."
            )
            return

        _get_real_gripper_client(node).command(
            side=side,
            position=position,
            effort=_RealParallelGripperClient.DEFAULT_EFFORT,
            closing=closing,
        )

    @property
    def _motion_chart(self) -> JointPositionList:
        """
        No-op Giskard task.

        The real gripper was already commanded in perform().
        Giskard still expects a valid task object, so we provide the gripper's
        nominal joint goal with weight=0.0 and a huge threshold.
        """

        arm = ViewManager().get_end_effector_view(self.gripper, self.robot_view)
        goal_state = arm.get_joint_state_by_type(self.motion)

        return JointPositionList(
            goal_state=goal_state,
            name="RealParallelGripper_NoOp",
            weight=0.0,
            threshold=100.0,
        )