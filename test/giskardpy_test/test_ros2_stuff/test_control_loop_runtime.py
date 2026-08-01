import pytest

from giskardpy.middleware.ros2.control_loop_profiler import (
    CONTROL_CYCLE_PHASES,
    CallTreeProfile,
    ControlLoopProfiler,
)
from giskardpy.middleware.ros2.utils.control_loop_benchmark import (
    CartesianGoalScenario,
    LongSequenceScenario,
    ScenarioRunner,
)

# %% profiler mechanics


class TestProfilerInstallation:
    """
    The profiler may only change the control loop while it is measuring it.
    """

    def test_profiled_methods_are_restored(self):
        profiler = ControlLoopProfiler(scenario_name="nothing", control_dt=0.05)
        before = {
            definition: definition.owner.__dict__[definition.method_name]
            for definition in CONTROL_CYCLE_PHASES
        }

        with profiler:
            for definition in CONTROL_CYCLE_PHASES:
                assert (
                    definition.owner.__dict__[definition.method_name]
                    is not before[definition]
                )

        for definition in CONTROL_CYCLE_PHASES:
            assert (
                definition.owner.__dict__[definition.method_name] is before[definition]
            )


# %% measured runtime


@pytest.fixture()
def cartesian_goal_profile(init_rospy) -> CallTreeProfile:
    return ScenarioRunner(debug_mode=False).run(CartesianGoalScenario())


@pytest.fixture()
def long_sequence_profile(init_rospy) -> CallTreeProfile:
    return ScenarioRunner(debug_mode=False).run(LongSequenceScenario())


@pytest.mark.slow
class TestPhaseAccounting:
    """
    A phase table that loses time cannot be used to decide what to optimize.
    """

    def test_measured_phases_account_for_the_whole_cycle(
        self, cartesian_goal_profile: CallTreeProfile
    ):
        control_cycle = cartesian_goal_profile.control_cycle
        children = cartesian_goal_profile.children_of(control_cycle.path)
        accounted = control_cycle.exclusive_total + sum(
            child.inclusive_total for child in children
        )

        assert accounted == pytest.approx(control_cycle.inclusive_total, rel=1e-9)


@pytest.mark.slow
class TestControlCycleBudget:
    """
    A cycle that overruns its slot makes the robot run slower than the controller was
    discretized for, and the pacer does not catch that up.
    """

    def test_cycle_stays_within_the_control_budget(
        self, long_sequence_profile: CallTreeProfile
    ):
        assert long_sequence_profile.control_cycle.inclusive_percentile(95) < (
            long_sequence_profile.control_dt
        )

    def test_motion_is_long_enough_to_judge_the_distribution(
        self, long_sequence_profile: CallTreeProfile
    ):
        assert long_sequence_profile.control_cycles >= 100
