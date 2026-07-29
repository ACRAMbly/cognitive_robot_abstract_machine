import json
from dataclasses import dataclass, field
from typing import Any, List, Optional

import pytest

from giskardpy.executor import Executor, SimulationPacer
from giskardpy.middleware.ros2.command_publishing import CommandPublisher
from giskardpy.middleware.ros2.control_loop import ControlLoop
from giskardpy.middleware.ros2.exceptions import WorldModelModifiedDuringMotionError
from giskardpy.middleware.ros2.feedback_publisher import ActionFeedbackPublisher
from giskardpy.middleware.ros2.heartbeat import Heartbeat
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
from semantic_digital_twin.callbacks.callback import StateChangeCallback
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
class WorldUpdatesMimic:
    """
    Stands in for the incoming world updates, counting how they are drained.
    """

    applied_batches: int = 0
    """
    How often everything that was received was applied.
    """

    applied_state_update_batches: int = 0
    """
    How often the state up to the next model change was applied.
    """

    acknowledged_batches: int = 0
    """
    How often the receipt of the pending updates was acknowledged.
    """

    pending_model_change: bool = False
    """
    Whether a model change is waiting to be applied.
    """

    def apply_all(self) -> None:
        self.applied_batches += 1
        self.pending_model_change = False

    def apply_state_updates(self) -> None:
        self.applied_state_update_batches += 1

    def acknowledge_receipt(self) -> None:
        self.acknowledged_batches += 1

    @property
    def has_pending_model_change(self) -> bool:
        return self.pending_model_change


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
class HeartbeatWatcher(InputSynchronizer):
    """
    Watches the heartbeat from inside the control loop and cancels the goal once it
    ticked often enough, standing in for a client that cancels a never-ending motion.
    """

    heartbeat: Heartbeat = None
    """
    The heartbeat that is watched.
    """

    action_server: Optional[GoalQueueMimic] = None
    """
    The action server the cancel request is written to.
    """

    ticks_until_cancel: int = 5
    """
    How many ticks to observe before requesting the cancel.
    """

    observed_ticks: int = 0
    """
    How many ticks were observed while the goal was running.
    """

    def apply(self) -> None:
        self.observed_ticks = self.heartbeat.count
        if self.observed_ticks >= self.ticks_until_cancel:
            self.action_server.cancel_requested = True


@dataclass(eq=False)
class StateChangeRecorder(StateChangeCallback):
    """
    Records every announced state change, standing in for the publishers that observe
    the world through its callbacks.
    """

    announced_changes: int = 0
    """
    How often a state change was announced to the observers of the world.
    """

    def on_state_change(self, **kwargs) -> None:
        self.announced_changes += 1


@dataclass
class PendingModelChangeInjector(InputSynchronizer):
    """
    Announces a pending model change after a few control cycles, standing in for another
    process that changes the world model mid-motion.
    """

    world_updates: Optional[WorldUpdatesMimic] = None
    """
    The incoming updates the model change is announced on.
    """

    cycles_until_model_change: int = 5
    """
    How many control cycles to run before the model change is announced.
    """

    observed_cycles: int = 0
    """
    How many control cycles were observed so far.
    """

    def apply(self) -> None:
        self.observed_cycles += 1
        if self.observed_cycles >= self.cycles_until_model_change:
            self.world_updates.pending_model_change = True


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
    world_updates: WorldUpdatesMimic
    motion_server: MotionServer
    control_loop: ControlLoop
    command_publisher: RecordingCommandPublisher
    idle_input: RecordingInputSynchronizer
    control_input: RecordingInputSynchronizer
    plotter: RecordingPlotter
    heartbeat: Heartbeat


@pytest.fixture()
def motion_server(init_rospy) -> MotionServerFixture:
    executor = create_executor()
    world = executor.context.world
    action_server = GoalQueueMimic()
    world_updates = WorldUpdatesMimic()
    feedback_publisher = ActionFeedbackPublisher(
        executor=executor, action_server=action_server
    )
    command_publisher = RecordingCommandPublisher(world=world)
    control_input = RecordingInputSynchronizer(world=world, executor=executor)
    heartbeat = Heartbeat()
    control_loop = ControlLoop(
        executor=executor,
        action_server=action_server,
        feedback_publisher=feedback_publisher,
        inputs=WorldStateInputs(world=world, synchronizers=[control_input]),
        heartbeat=heartbeat,
        world_updates=world_updates,
        command_publishers=[command_publisher],
    )
    idle_input = RecordingInputSynchronizer(world=world, executor=executor)
    plotter = RecordingPlotter(executor=executor)
    server = MotionServer(
        executor=executor,
        action_server=action_server,
        control_loop=control_loop,
        world_updates=world_updates,
        feedback_publisher=feedback_publisher,
        inputs=WorldStateInputs(world=world, synchronizers=[idle_input]),
        heartbeat=heartbeat,
        post_goal_plotters=[plotter],
    )
    return MotionServerFixture(
        executor=executor,
        action_server=action_server,
        world_updates=world_updates,
        motion_server=server,
        control_loop=control_loop,
        command_publisher=command_publisher,
        idle_input=idle_input,
        control_input=control_input,
        plotter=plotter,
        heartbeat=heartbeat,
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

    def test_the_reason_of_a_failure_is_reported(
        self, motion_server: MotionServerFixture
    ):
        """
        A motion terminated by a world model modification can be sent again, so the
        client has to be able to tell it apart from a real failure.
        """
        inject_modification = PendingModelChangeInjector(
            world=motion_server.executor.context.world,
            world_updates=motion_server.world_updates,
            cycles_until_model_change=5,
        )
        motion_server.control_loop.inputs.synchronizers.append(inject_modification)
        motion_server.action_server.goal_json = create_goal_json(seconds=1000.0)

        motion_server.motion_server.run_idle_cycle()

        result = json.loads(motion_server.action_server.sent_results[0].result)
        assert result["error"] == WorldModelModifiedDuringMotionError.__name__

    def test_a_successful_goal_reports_no_failure(
        self, motion_server: MotionServerFixture
    ):
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        result = json.loads(motion_server.action_server.sent_results[0].result)
        assert "error" not in result

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

        assert motion_server.world_updates.applied_batches == 1

    def test_the_state_change_is_announced_to_the_observers_of_the_world(
        self, motion_server: MotionServerFixture
    ):
        """
        The idle loop hands the published state to the world callbacks instead of
        calling a publisher itself, so every observer of the world sees it.
        """
        recorder = StateChangeRecorder(_world=motion_server.executor.context.world)

        motion_server.motion_server.run_idle_cycle()

        assert recorder.announced_changes == 1

    def test_inputs_are_read(self, motion_server: MotionServerFixture):
        motion_server.motion_server.run_idle_cycle()

        assert len(motion_server.idle_input.applied_at_control_cycles) == 1

    def test_heartbeat_ticks_once_per_idle_cycle(
        self, motion_server: MotionServerFixture
    ):
        motion_server.motion_server.run_idle_cycle()
        motion_server.motion_server.run_idle_cycle()

        assert motion_server.heartbeat.count == 2

    def test_nothing_happens_while_the_world_is_being_modified(
        self, motion_server: MotionServerFixture
    ):
        with motion_server.executor.context.world.modify_world():
            motion_server.motion_server.run_idle_cycle()

        assert motion_server.heartbeat.count == 0
        assert motion_server.world_updates.applied_batches == 0


class TestHeartbeatDuringGoals:
    """
    The heartbeat keeps ticking while a goal is running, so an observer waiting for the
    server to make progress is never blocked by a motion that only ends on cancel.
    """

    def test_heartbeat_ticks_during_a_goal(self, motion_server: MotionServerFixture):
        motion_server.action_server.goal_json = create_goal_json()
        heartbeat_before_goal = motion_server.heartbeat.count

        motion_server.motion_server.run_idle_cycle()

        control_cycles = len(motion_server.control_input.applied_at_control_cycles)
        assert control_cycles > 1
        assert (
            motion_server.heartbeat.count == heartbeat_before_goal + 1 + control_cycles
        )

    def test_heartbeat_ticks_while_a_goal_never_ends_on_its_own(
        self, motion_server: MotionServerFixture
    ):
        """
        A goal without an end motion only stops on cancel; an observer must still see
        progress while it runs.
        """
        cancel_after = HeartbeatWatcher(
            world=motion_server.executor.context.world,
            heartbeat=motion_server.heartbeat,
            action_server=motion_server.action_server,
            ticks_until_cancel=5,
        )
        motion_server.control_loop.inputs.synchronizers.append(cancel_after)
        motion_server.action_server.goal_json = create_goal_json(seconds=1000.0)

        motion_server.motion_server.run_idle_cycle()

        assert cancel_after.observed_ticks >= 5
        assert motion_server.action_server.outcome == "canceled"


class TestWorldUpdatesDuringGoals:
    """
    State updates of other processes are applied while a goal runs, but a model
    modification invalidates the compiled motion statechart, so it terminates the motion
    instead of being applied under it.

    Receipt is acknowledged either way, so a process that modifies the world
    synchronously is never blocked until the motion is over.
    """

    def test_receipt_is_acknowledged_once_per_control_cycle(
        self, motion_server: MotionServerFixture
    ):
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        control_cycles = len(motion_server.control_input.applied_at_control_cycles)
        assert control_cycles > 1
        assert motion_server.world_updates.acknowledged_batches == control_cycles

    def test_state_updates_are_applied_once_per_control_cycle(
        self, motion_server: MotionServerFixture
    ):
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        control_cycles = len(motion_server.control_input.applied_at_control_cycles)
        assert control_cycles > 1
        assert (
            motion_server.world_updates.applied_state_update_batches == control_cycles
        )

    def test_the_whole_buffer_is_not_applied_while_a_goal_runs(
        self, motion_server: MotionServerFixture
    ):
        motion_server.action_server.goal_json = create_goal_json()

        motion_server.motion_server.run_idle_cycle()

        assert motion_server.world_updates.applied_batches == 1

    def test_a_pending_model_change_terminates_the_goal(
        self, motion_server: MotionServerFixture
    ):
        inject_modification = PendingModelChangeInjector(
            world=motion_server.executor.context.world,
            world_updates=motion_server.world_updates,
            cycles_until_model_change=5,
        )
        motion_server.control_loop.inputs.synchronizers.append(inject_modification)
        motion_server.action_server.goal_json = create_goal_json(seconds=1000.0)

        motion_server.motion_server.run_idle_cycle()

        assert motion_server.action_server.outcome == "aborted"
        control_cycles = len(motion_server.control_input.applied_at_control_cycles)
        assert control_cycles < 10, "the goal ran on instead of terminating promptly"

    def test_the_buffered_modification_is_applied_by_the_next_idle_cycle(
        self, motion_server: MotionServerFixture
    ):
        inject_modification = PendingModelChangeInjector(
            world=motion_server.executor.context.world,
            world_updates=motion_server.world_updates,
            cycles_until_model_change=5,
        )
        motion_server.control_loop.inputs.synchronizers.append(inject_modification)
        motion_server.action_server.goal_json = create_goal_json(seconds=1000.0)
        motion_server.motion_server.run_idle_cycle()
        assert motion_server.world_updates.has_pending_model_change

        motion_server.motion_server.run_idle_cycle()

        assert not motion_server.world_updates.has_pending_model_change
