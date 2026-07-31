from dataclasses import dataclass, field
from typing import List

from giskardpy.middleware.ros2.action_server import GoalOutcome

# %% mimics


@dataclass
class GoalStateRecorder:
    """
    Stands in for a goal handle and records which state transition was requested.
    """

    transitions: List[str] = field(default_factory=list)
    """
    The name of every transition that was requested, in order.
    """

    def succeed(self) -> None:
        self.transitions.append("succeed")

    def abort(self) -> None:
        self.transitions.append("abort")

    def canceled(self) -> None:
        self.transitions.append("canceled")


# %% reporting an outcome


def test_succeeded_is_reported_as_success():
    goal_handle = GoalStateRecorder()

    GoalOutcome.SUCCEEDED.report_to(goal_handle)

    assert goal_handle.transitions == ["succeed"]


def test_aborted_is_reported_as_abort():
    goal_handle = GoalStateRecorder()

    GoalOutcome.ABORTED.report_to(goal_handle)

    assert goal_handle.transitions == ["abort"]


def test_canceled_is_reported_as_cancellation():
    goal_handle = GoalStateRecorder()

    GoalOutcome.CANCELED.report_to(goal_handle)

    assert goal_handle.transitions == ["canceled"]


def test_every_outcome_reports_exactly_one_transition():
    for outcome in GoalOutcome:
        goal_handle = GoalStateRecorder()

        outcome.report_to(goal_handle)

        assert len(goal_handle.transitions) == 1
