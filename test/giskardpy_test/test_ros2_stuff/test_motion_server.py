import json
from dataclasses import dataclass, field
from typing import Any, List, Optional

import pytest

from giskardpy.executor import Executor, SimulationPacer
from giskardpy.middleware.ros2.command_publishing import CommandPublisher
from giskardpy.middleware.ros2.control_loop import ControlLoop
from giskardpy.middleware.ros2.feedback_publisher import ActionFeedbackPublisher
from giskardpy.middleware.ros2.input_synchronization import (
    InputSynchronizer,
    WorldStateInputs,
)
from giskardpy.middleware.ros2.motion_server import MotionServer
from giskardpy.middleware.ros2.post_goal_plotters import PostGoalPlotter
from giskardpy.motion_statechart.context import MotionStatechartContext
from giskardpy.motion_statechart.graph_node import EndMotion
from giskardpy.motion_statechart.monitors.payload_monitors import (
    CountSimulationTimeSeconds,
)
from giskardpy.motion_statechart.motion_statechart import MotionStatechart
from giskardpy.qp.qp_controller_config import QPControllerConfig
from semantic_digital_twin.world import World

# %% mimics of the ros facing collaborators


@dataclass
class GoalQueueMimic:
    """
    Stands in for the action server: hands out one goal and records the outcome.
    """

    goal_json: Optional[str] = None
    """
    The goal that is waiting to be accepted, or ``None`` if there is none.
    """

    action_name: str = "mimic"
    """
    Name reported in cancel exceptions.
    """

    goal_id: int = -1
    """
    Number of goals accepted so far.
    """

    cancel_requested: bool = False
    """
    Whether the current goal should be canceled.
    """

    goal_msg: Optional[Any] = field(init=False, default=None)
    """
    Request of the accepted goal.
    """

    result_msg: Optional[Any] = field(init=False, default=None)
    """
    Result built for the accepted goal.
    """

    outcome: Optional[str] = field(init=False, default=None)
    """
    Whether the goal was marked as succeeded, aborted or canceled.
    """

    sent_results: List[Any] = field(init=False, default_factory=list)
    """
    Every result that was handed back to a client.
    """

    feedback_messages: List[Any] = field(init=False, default_factory=list)
    """
    Every feedback message that was published.
    """

    def has_goal(self) -> bool:
        return self.goal_json is not None

    def accept_goal(self) -> None:
        self.goal_msg = GoalMessageMimic(goal=self.goal_json)
        self.goal_json = None
        self.goal_id += 1

    def is_cancel_requested(self) -> bool:
        return self.cancel_requested

    def loginfo(self, message: str) -> None:
        pass

    def send_feedback(self, message: Any) -> None:
        self.feedback_messages.append(message)

    def set_canceled(self) -> None:
        self.outcome = "canceled"

    def set_aborted(self) -> None:
        self.outcome = "aborted"

    def set_succeeded(self) -> None:
        self.outcome = "succeeded"

    def send_result(self) -> None:
        self.sent_results.append(self.result_msg)


@dataclass
class GoalMessageMimic:
    """
    Stands in for the goal request of the action.
    """

    goal: str
    """
    The motion statechart as json.
    """


@dataclass
class WorldSyncMimic:
    """
    Stands in for the world synchronizer of the idle loop.
    """

    applied_message_batches: int = 0
    """
    How often buffered messages of other processes were applied.
    """

    published_states: int = 0
    """
    How often the world state was published.
    """

    def apply_missed_messages(self) -> None:
        self.applied_message_batches += 1

    def on_state_change(self, **kwargs) -> None:
        self.published_states += 1


@dataclass
class RecordingInputSynchronizer(InputSynchronizer):
    """
    Records in which order inputs are read relative to the control cycles.
    """

    executor: Executor = None
    """
    The executor whose control cycles are recorded on every apply.
    """

    applied_at_control_cycles: List[float] = field(default_factory=list)
    """
    The control cycle count at the time of every apply.
    """

    def apply(self) -> None:
        self.applied_at_control_cycles.append(self.executor.control_cycles)


@dataclass
class RecordingCommandPublisher(CommandPublisher):
    """
    Records how often commands were published and when the robot was stopped.
    """

    published_velocities: List[float] = field(default_factory=list)
    """
    The commanded velocity of every publish.
    """

    stop_count: int = 0
    """
    How often the robot was told to stop.
    """

    world: World = None
    """
    The world the commanded velocities are read from.
    """

    def publish(self) -> None:
        self.published_velocities.append(float(self.world.state.velocities.sum()))

    def stop(self) -> None:
        self.stop_count += 1


@dataclass
class FailingInputSynchronizer(InputSynchronizer):
    """
    Fails while reading its input, standing in for a broken robot interface.
    """

    def apply(self) -> None:
        raise BrokenInputError()


class BrokenInputError(Exception):
    """
    Raised by :class:`FailingInputSynchronizer`.
    """


@dataclass
class RecordingPlotter(PostGoalPlotter):
    """
    Records for which goals debug plots were requested.
    """

    plotted_goal_ids: List[int] = field(default_factory=list)
    """
    The goal ids that were plotted.
    """

    def plot(self, goal_id: int) -> None:
        self.plotted_goal_ids.append(goal_id)


# %% fixtures


def create_executor() -> Executor:
    """
    Build an executor that simulates as fast as possible in an empty world.
    """
    return Executor(
        context=MotionStatechartContext(
            world=World(),
            qp_controller_config=QPControllerConfig.create_with_simulation_defaults(),
        ),
        pacer=SimulationPacer(real_time_factor=None),
    )


def create_goal_json(seconds: float = 0.5) -> str:
    """
    Build the json of a motion statechart that ends after the given simulated time.
    """
    motion_statechart = MotionStatechart()
    motion_statechart.add_node(counter := CountSimulationTimeSeconds(seconds=seconds))
    motion_statechart.add_node(EndMotion.when_true(counter))
    return json.dumps(motion_statechart.to_json())


@dataclass
class MotionServerFixture:
    """
    A motion server wired to mimics, so its lifecycle can be driven from a test.
    """

    executor: Executor
    action_server: GoalQueueMimic
    world_synchronizer: WorldSyncMimic
    motion_server: MotionServer
    control_loop: ControlLoop
    command_publisher: RecordingCommandPublisher
    idle_input: RecordingInputSynchronizer
    control_input: RecordingInputSynchronizer
    plotter: RecordingPlotter


@pytest.fixture()
def motion_server(init_rospy) -> MotionServerFixture:
    executor = create_executor()
    world = executor.context.world
    action_server = GoalQueueMimic()
    world_synchronizer = WorldSyncMimic()
    feedback_publisher = ActionFeedbackPublisher(
        executor=executor, action_server=action_server
    )
    command_publisher = RecordingCommandPublisher(world=world)
    control_input = RecordingInputSynchronizer(world=world, executor=executor)
    control_loop = ControlLoop(
        executor=executor,
        action_server=action_server,
        feedback_publisher=feedback_publisher,
        inputs=WorldStateInputs(world=world, synchronizers=[control_input]),
        command_publishers=[command_publisher],
    )
    idle_input = RecordingInputSynchronizer(world=world, executor=executor)
    plotter = RecordingPlotter(executor=executor)
    server = MotionServer(
        executor=executor,
        action_server=action_server,
        control_loop=control_loop,
        world_synchronizer=world_synchronizer,
        feedback_publisher=feedback_publisher,
        inputs=WorldStateInputs(world=world, synchronizers=[idle_input]),
        post_goal_plotters=[plotter],
    )
    return MotionServerFixture(
        executor=executor,
        action_server=action_server,
        world_synchronizer=world_synchronizer,
        motion_server=server,
        control_loop=control_loop,
        command_publisher=command_publisher,
        idle_input=idle_input,
        control_input=control_input,
        plotter=plotter,
    )


# %% goal lifecycle


class TestGoalResult:
    """
    A goal is always answered, with an outcome that reflects what happened.
    """

    def test_finished_motion_succeeds(self, motion_server: MotionServerFixture):
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        assert motion_server.action_server.outcome == "succeeded"
        assert len(motion_server.action_server.sent_results) == 1

    def test_canceled_goal_is_reported_as_canceled(
        self, motion_server: MotionServerFixture
    ):
        motion_server.action_server.goal_json = create_goal_json(seconds=100.0)
        motion_server.action_server.cancel_requested = True

        motion_server.motion_server.run_idle_cycle()

        assert motion_server.action_server.outcome == "canceled"

    def test_broken_input_aborts_the_goal(self, motion_server: MotionServerFixture):
        motion_server.control_loop.inputs.synchronizers = [
            FailingInputSynchronizer(world=motion_server.executor.context.world)
        ]
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        assert motion_server.action_server.outcome == "aborted"
        assert len(motion_server.action_server.sent_results) == 1

    def test_result_contains_the_final_states(self, motion_server: MotionServerFixture):
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        result = json.loads(motion_server.action_server.sent_results[0].result)
        assert "life_cycle_state" in result
        assert "observation_state" in result


class TestCleanupAfterGoal:
    """
    Whatever happens to a goal, the robot is stopped and the client is answered.
    """

    def test_robot_is_stopped_after_a_successful_goal(
        self, motion_server: MotionServerFixture
    ):
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        assert motion_server.command_publisher.stop_count == 1

    def test_robot_is_stopped_after_a_failed_goal(
        self, motion_server: MotionServerFixture
    ):
        motion_server.control_loop.inputs.synchronizers = [
            FailingInputSynchronizer(world=motion_server.executor.context.world)
        ]
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        assert motion_server.command_publisher.stop_count == 1

    def test_debug_plots_are_written_for_the_goal(
        self, motion_server: MotionServerFixture
    ):
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        assert motion_server.plotter.plotted_goal_ids == [
            motion_server.action_server.goal_id
        ]

    def test_feedback_is_published_at_the_end_of_the_goal(
        self, motion_server: MotionServerFixture
    ):
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        assert len(motion_server.action_server.feedback_messages) > 0


# %% control loop


class TestControlCycleOrder:
    """
    Every control cycle reads the robot before it computes the next command.
    """

    def test_inputs_are_read_before_the_controller_ticks(
        self, motion_server: MotionServerFixture
    ):
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        applied = motion_server.control_input.applied_at_control_cycles
        assert applied == sorted(applied)
        assert applied[0] == 0
        assert applied[-1] == motion_server.executor.control_cycles - 1

    def test_commands_are_published_once_per_cycle(
        self, motion_server: MotionServerFixture
    ):
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        assert len(motion_server.command_publisher.published_velocities) == len(
            motion_server.control_input.applied_at_control_cycles
        )


class TestStopClearsCommands:
    """
    Stopping the control loop leaves no commanded motion behind.
    """

    def test_derivatives_are_zeroed(self, motion_server: MotionServerFixture):
        world = motion_server.executor.context.world
        world.state.velocities[:] = 1
        world.state.accelerations[:] = 1
        world.state.jerks[:] = 1

        motion_server.control_loop.stop()

        assert not world.state.velocities.any()
        assert not world.state.accelerations.any()
        assert not world.state.jerks.any()

    def test_publishers_are_stopped(self, motion_server: MotionServerFixture):
        motion_server.control_loop.stop()

        assert motion_server.command_publisher.stop_count == 1


# %% idle loop


class TestIdleLoop:
    """
    While waiting for a goal, Giskard keeps its world in sync with the outside.
    """

    def test_world_updates_of_other_processes_are_applied(
        self, motion_server: MotionServerFixture
    ):
        motion_server.motion_server.run_idle_cycle()

        assert motion_server.world_synchronizer.applied_message_batches == 1

    def test_world_state_is_published(self, motion_server: MotionServerFixture):
        motion_server.motion_server.run_idle_cycle()

        assert motion_server.world_synchronizer.published_states == 1

    def test_world_state_is_not_published_when_disabled(
        self, motion_server: MotionServerFixture
    ):
        motion_server.motion_server.publish_world_state = False

        motion_server.motion_server.run_idle_cycle()

        assert motion_server.world_synchronizer.published_states == 0

    def test_inputs_are_read(self, motion_server: MotionServerFixture):
        motion_server.motion_server.run_idle_cycle()

        assert len(motion_server.idle_input.applied_at_control_cycles) == 1

    def test_cycle_count_increases(self, motion_server: MotionServerFixture):
        motion_server.motion_server.run_idle_cycle()
        motion_server.motion_server.run_idle_cycle()

        assert motion_server.motion_server.cycle_count == 2

    def test_nothing_happens_while_the_world_is_being_modified(
        self, motion_server: MotionServerFixture
    ):
        with motion_server.executor.context.world.modify_world():
            motion_server.motion_server.run_idle_cycle()

        assert motion_server.motion_server.cycle_count == 0
        assert motion_server.world_synchronizer.applied_message_batches == 0
