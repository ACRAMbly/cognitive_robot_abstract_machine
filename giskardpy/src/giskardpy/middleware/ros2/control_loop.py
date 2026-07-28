from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from giskardpy.executor import Executor
from giskardpy.middleware.ros2.action_server import ActionServerHandler
from giskardpy.middleware.ros2.command_publishing import CommandPublisher
from giskardpy.middleware.ros2.exceptions import ExecutionCanceledException
from giskardpy.middleware.ros2.feedback_publisher import ActionFeedbackPublisher
from giskardpy.middleware.ros2.input_synchronization import WorldStateInputs
from giskardpy.middleware.ros2.qp_data_publisher import QPDataPublisher
from semantic_digital_twin.world import World


@dataclass
class ControlLoop:
    """
    Runs a compiled motion statechart until it ends.

    Every cycle reads the inputs of the robot, ticks the controller and sends the
    resulting velocities back to the robot.
    """

    executor: Executor
    """
    Computes the next command from the motion statechart.
    """

    action_server: ActionServerHandler
    """
    The action server the running goal belongs to; polled for cancel requests.
    """

    feedback_publisher: ActionFeedbackPublisher
    """
    Reports the state of the motion statechart to the action client.
    """

    inputs: WorldStateInputs
    """
    Writes the state of the robot into the world at the start of every cycle.
    """

    command_publishers: List[CommandPublisher] = field(default_factory=list)
    """
    Sends the computed velocities to the robot at the end of every cycle.
    """

    qp_data_publisher: Optional[QPDataPublisher] = None
    """
    Streams the internals of the quadratic program for debugging, if configured.
    """

    @property
    def world(self) -> World:
        return self.executor.context.world

    def run(self) -> None:
        """
        Run cycles until the motion statechart reaches an end motion.

        :raises ExecutionCanceledException: If the goal was canceled.
        """
        while True:
            self.run_cycle()
            if self.executor.motion_statechart.is_end_motion():
                return
            self.executor.pacer.sleep()

    def run_cycle(self) -> None:
        """
        Synchronize the inputs, compute the next command and publish it.

        :raises ExecutionCanceledException: If the goal was canceled.
        """
        self.inputs.synchronize()
        self.raise_if_canceled()
        self.executor.tick()
        self.publish_commands()
        self.publish_qp_data()
        self.feedback_publisher.publish_if_changed()

    def publish_qp_data(self) -> None:
        """
        Stream the internals of the quadratic program, if a publisher is configured.
        """
        if self.qp_data_publisher is None:
            return
        self.qp_data_publisher.publish(self.executor.qp_controller)

    def raise_if_canceled(self) -> None:
        """
        :raises ExecutionCanceledException: If the client canceled the goal or a new
            goal superseded it.
        """
        if not self.action_server.is_cancel_requested():
            return
        self.action_server.loginfo("canceled")
        raise ExecutionCanceledException(
            action_server_name=self.action_server.action_name,
            goal_id=self.action_server.goal_id,
        )

    def publish_commands(self) -> None:
        """
        Send the velocities of the current cycle to the robot.
        """
        for command_publisher in self.command_publishers:
            command_publisher.publish()

    def stop(self) -> None:
        """
        Bring the robot to a halt and clear the commanded velocities.
        """
        for command_publisher in self.command_publishers:
            command_publisher.stop()
        self.executor.set_velocity_acceleration_jerk_to_zero()
        self.world.notify_state_change()
