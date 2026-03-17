"""
tracy_motion_mapping.py
========================
Alternative motion implementations for the Tracy robot running in REAL execution mode.

Currently overrides:
    MoveGripperMotion → TracyRealMoveGripperMotion
        Calls the physical Robotiq 85 gripper action servers
        (/left_gripper/robotiq_gripper_controller/gripper_cmd  and
         /right_gripper/robotiq_gripper_controller/gripper_cmd)
        instead of sending a Giskard JointPositionList task (which only works
        in simulation / when joints are part of ros2_control).

Usage (in any demo / script):
    import pycram.alternative_motion_mappings.tracy_motion_mapping  # noqa: F401
    # That single import registers the alternative.  Nothing else is needed.
    # When executing inside `with real_robot:`, every MoveGripperMotion issued
    # for a Tracy robot will automatically use TracyRealMoveGripperMotion.
"""

from __future__ import annotations

import sys
import os
import time
import logging

from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.tracy import Tracy

from ..datastructures.enums import ExecutionType, Arms
from ..robot_plans.motions.base import AlternativeMotion
from ..robot_plans.motions.gripper import MoveGripperMotion
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Lightweight gripper client (no node ownership — uses the plan's shared node)
# ---------------------------------------------------------------------------

class _RealGripperClient:
    """
    Thin wrapper around two ROS 2 action clients (left + right Robotiq 85).

    Uses the ROS node that is already running in the demo (passed via Context.ros_node),
    so it never spins a competing executor. The demo spinner thread handles callbacks.
    """

    # Action topic template — matches the real robot launch files
    _TOPIC = "/{side}_gripper/robotiq_gripper_controller/gripper_cmd"

    # Gripper Limits
    GRIPPER_OPEN: float = 0.0
    GRIPPER_CLOSE: float = 0.35
    GRIPPER_EFFORT_DEFAULT: float = 50.0
    GRIPPER_EFFORT_SOFT: float = 2.0

    # How long to sleep after sending the command to let the physical gripper move
    DEFAULT_WAIT = 1.0  # seconds

    def __init__(self, node):
        self._node = node
        # Eagerly create BOTH clients at startup so they are fully constructed
        # (including _lock) before rclpy.spin starts. Lazy creation races with
        # the spin thread which calls get_num_entities() on partially-built clients.
        self._clients: dict[str, ActionClient] = {
            side: ActionClient(node, GripperCommand, self._TOPIC.format(side=side))
            for side in ("left", "right")
        }
        logger.info("[TracyGripper] Pre-created action clients for left and right grippers")

    def _get_client(self, side: str) -> ActionClient:
        """Return the cached ActionClient for the given side."""
        return self._clients[side]

    def command(self, side: str, position: float, effort: float = GRIPPER_EFFORT_DEFAULT,
                wait_time: float = DEFAULT_WAIT) -> None:
        """
        Send a GripperCommand goal and block until the gripper physically finishes.

        :param side:      'left' or 'right'
        :param position:  Target position in metres (0.0 = open, 0.35 = closed)
        :param effort:    Max effort / force [N] (default 50 N)
        :param wait_time: Extra safety sleep after the action completes [s]
        """
        client = self._get_client(side)

        if not client.wait_for_server(timeout_sec=3.0):
            logger.error(
                f"[TracyGripper] Action server for '{side}' gripper not available! "
                "Skipping gripper command."
            )
            return

        goal = GripperCommand.Goal()
        goal.command.position   = float(position)
        goal.command.max_effort = float(effort)

        logger.info(f"[TracyGripper] Sending {side.upper()} gripper → {position:.3f} m")

        # send_goal — the demo's background rclpy.spin() thread will process callbacks
        future = client.send_goal_async(goal)

        # Busy-wait for the goal handle (callbacks run in the spinner thread)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)

        if not future.done():
            logger.error("[TracyGripper] Timeout waiting for goal handle!")
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            logger.error(f"[TracyGripper] Goal rejected for {side} gripper.")
            return

        # Wait for the result
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + 10.0
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)

        if result_future.done():
            logger.info(f"[TracyGripper] {side.upper()} gripper reached target.")
        else:
            logger.warning(f"[TracyGripper] {side.upper()} gripper result timed out — continuing.")

        time.sleep(wait_time)


# One shared client per ROS node (lazy, created on first use)
_client_cache: dict[int, _RealGripperClient] = {}


def _get_real_gripper_client(node) -> _RealGripperClient:
    key = id(node)
    if key not in _client_cache:
        _client_cache[key] = _RealGripperClient(node)
    return _client_cache[key]


# ---------------------------------------------------------------------------
# Alternative motion: replaces MoveGripperMotion for Tracy + REAL execution
# ---------------------------------------------------------------------------

class TracyRealMoveGripperMotion(MoveGripperMotion, AlternativeMotion[Tracy]):
    """
    Real-robot override of MoveGripperMotion for the Tracy platform.

    When ``MotionExecutor.execution_type == ExecutionType.REAL`` (i.e. inside a
    ``with real_robot:`` block) and the active robot is a Tracy instance, PyCRAM
    will automatically dispatch every MoveGripperMotion through this class instead
    of building a Giskard JointPositionList task.

    The physical Robotiq 85 grippers are commanded via their ROS 2 action servers.
    The Giskard motion chart receives a no-op joint task so the rest of the
    pipeline stays unchanged (collision avoidance, FK, etc. are unaffected).
    """

    execution_type = ExecutionType.REAL

    # Gripper joint positions for OPEN / CLOSE (Giskard world model stays in sync)
    _POSITION_MAP = {
        GripperState.OPEN:  _RealGripperClient.GRIPPER_OPEN,
        GripperState.CLOSE: _RealGripperClient.GRIPPER_CLOSE,
    }

    def perform(self) -> None:
        """Command the physical gripper and return — no Giskard task is sent."""
        side = "right" if self.gripper == Arms.RIGHT else "left"
        position = self._POSITION_MAP[self.motion]

        node = self.plan.context.ros_node
        if node is None:
            logger.error(
                "[TracyGripper] Context has no ros_node — cannot command real gripper. "
                "Make sure you pass `ros_node=node` to Context()."
            )
            return

        _get_real_gripper_client(node).command(side, position)

    @property
    def _motion_chart(self) -> JointPositionList:
        """
        Returns a no-op Giskard joint task.

        The real gripper is already commanded synchronously in perform().
        Giskard still needs a valid task object, but JointPositionList fails if empty.
        So we pass the actual gripper joint goals, but set weight=0.0 and a huge threshold,
        causing Giskard to instantly succeed without ever trying to move them.
        """
        from pycram.view_manager import ViewManager

        arm = ViewManager().get_end_effector_view(self.gripper, self.robot_view)
        goal_state = arm.get_joint_state_by_type(self.motion)

        return JointPositionList(
            goal_state=goal_state,
            name="RealGripper_NoOp",
            weight=0.0,
            threshold=100.0,
        )
