"""Tests for reliable colored-block pose queries."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field

import pytest
from py_trees.blackboard import Blackboard
from py_trees.common import Status

from robokudo.behaviours.action_server_checks import (
    ActionServerPresentAndDone,
)
from robokudo.descriptors.analysis_engines import (
    robokudo_cram_integration as integration,
)
from robokudo.identifier import BBIdentifier


@dataclass(frozen=True)
class DetectedPosition:
    """Position returned by a perception result."""

    x: float
    """Position along the x-axis."""

    y: float
    """Position along the y-axis."""

    z: float
    """Position along the z-axis."""


@dataclass(frozen=True)
class DetectedPose:
    """Pose containing a detected position."""

    position: DetectedPosition
    """Position component of the pose."""


@dataclass(frozen=True)
class DetectedStampedPose:
    """Stamped-pose shape returned in an object designator."""

    pose: DetectedPose
    """Detected pose."""


@dataclass(frozen=True)
class DetectedObject:
    """Object designator shape used by the integration parser."""

    color: list[str]
    """Detected semantic colors."""

    pose: list[DetectedStampedPose]
    """Detected stamped poses."""


@dataclass(frozen=True)
class PerceptionResult:
    """Perception result containing detected objects."""

    res: list[DetectedObject]
    """Detected object designators."""


@dataclass(frozen=True)
class CompletedResult:
    """Completed action result wrapper."""

    result: PerceptionResult
    """Returned perception result."""


@dataclass(frozen=True)
class CompletedFuture:
    """Immediately completed future used by synchronous tests."""

    value: object
    """Value returned when the future is inspected."""

    def result(self) -> object:
        """Return the completed value."""
        return self.value


@dataclass(frozen=True)
class AttemptResponse:
    """Goal response for one perception attempt."""

    perception_result: PerceptionResult
    """Perception result returned for the attempt."""

    accepted: bool
    """Whether the query goal was accepted."""

    def get_result_async(self) -> CompletedFuture:
        """Return the completed perception result."""
        return CompletedFuture(CompletedResult(self.perception_result))


@dataclass
class SequencedAttemptResponder:
    """Return one configured perception result for each submitted attempt."""

    perception_results: list[PerceptionResult]
    """Perception results in submission order."""

    server_available: bool = True
    """Whether the responder reports an available action server."""

    goals_accepted: bool = True
    """Whether submitted goals are accepted."""

    submitted_goals: list[object] = field(default_factory=list)
    """Goals submitted by the integration."""

    def wait_for_server(self, timeout_sec: float) -> bool:
        """Report the configured server availability."""
        return self.server_available

    def send_goal_async(self, goal: object) -> CompletedFuture:
        """Return the result assigned to the current attempt."""
        result_index = len(self.submitted_goals)
        self.submitted_goals.append(goal)
        return CompletedFuture(
            AttemptResponse(
                self.perception_results[result_index],
                self.goals_accepted,
            )
        )


@dataclass(frozen=True)
class QueryLifecycleActionServer:
    """Expose the action-server state relevant to pipeline initialization."""

    new_query: object | None
    """Query that has not yet been consumed by the query annotator."""

    active: bool
    """Whether the action server currently owns a goal."""

    def is_active(self) -> bool:
        """Return whether the action server currently owns a goal."""
        return self.active


@pytest.fixture
def query_server_blackboard() -> Iterator[Blackboard]:
    """Provide isolated query-server blackboard state."""
    blackboard = Blackboard()
    blackboard.set(BBIdentifier.QUERY_SERVER_IN_PIPELINE, True)
    yield blackboard
    blackboard.set(BBIdentifier.QUERY_SERVER, None)
    blackboard.set(BBIdentifier.QUERY_SERVER_IN_PIPELINE, False)


def detected_object(
    color: str,
    x_position: float,
    y_position: float,
    z_position: float = 0.0,
) -> DetectedObject:
    """Create a detected object with one pose."""
    position = DetectedPosition(x_position, y_position, z_position)
    return DetectedObject(
        [color],
        [DetectedStampedPose(DetectedPose(position))],
    )


def perception_result(*detected_objects: DetectedObject) -> PerceptionResult:
    """Create a perception result from detected objects."""
    return PerceptionResult(list(detected_objects))


def install_responder(
    monkeypatch: pytest.MonkeyPatch,
    responder: SequencedAttemptResponder,
) -> None:
    """Route integration action requests to a configured responder."""

    def create_responder(
        node: object,
        action_type: object,
        action_name: str,
    ) -> SequencedAttemptResponder:
        """Return the configured responder."""
        return responder

    def complete_future(node: object, future: CompletedFuture) -> None:
        """Treat the configured future as already complete."""

    monkeypatch.setattr(integration, 'ActionClient', create_responder)
    monkeypatch.setattr(
        integration.rclpy,
        'spin_until_future_complete',
        complete_future,
    )


def test_new_query_can_reach_query_annotator_during_initialization(
    query_server_blackboard: Blackboard,
) -> None:
    """Allow an unconsumed goal through the completed-goal guard."""
    query_server_blackboard.set(
        BBIdentifier.QUERY_SERVER,
        QueryLifecycleActionServer(new_query=object(), active=True),
    )

    status = ActionServerPresentAndDone().update()

    assert status is Status.SUCCESS


def test_cyan_block_label_is_normalized_to_blue() -> None:
    """Normalize the live camera's cyan block label to the blue target."""
    positions = integration.ColoredBlockPoseParser().parse(
        perception_result(
            detected_object('cyan', 0.4, -0.2),
        )
    )

    assert positions == {'blue': (0.62, -0.2, 0.955)}


def test_empty_result_is_retried_with_a_fresh_query(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Retry with a fresh query when a perception result is empty."""
    responder = SequencedAttemptResponder(
        [
            perception_result(),
            perception_result(
                detected_object('red', 0.1, 0.2),
                detected_object('blue', 0.3, 0.4),
                detected_object('yellow', 0.5, 0.6),
            ),
        ]
    )
    install_responder(monkeypatch, responder)

    positions = integration.query_colored_block_poses_from_robokudo(object())

    assert set(positions) == {'red', 'blue', 'yellow'}
    assert len(responder.submitted_goals) == 2


def test_partial_results_are_aggregated_across_attempts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Merge colors found by separate fresh-frame query attempts."""
    responder = SequencedAttemptResponder(
        [
            perception_result(detected_object('red', 0.1, 0.2)),
            perception_result(detected_object('yellow', 0.3, 0.4)),
            perception_result(detected_object('blue', 0.5, 0.6)),
        ]
    )
    install_responder(monkeypatch, responder)

    positions = integration.query_colored_block_poses_from_robokudo(object())

    assert positions == {
        'red': (0.32, 0.2, 0.955),
        'yellow': (0.52, 0.4, 0.955),
        'blue': (0.72, 0.6, 0.955),
    }
    assert len(responder.submitted_goals) == 3


def test_all_colors_stop_further_attempts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Stop querying as soon as every target color has a pose."""
    responder = SequencedAttemptResponder(
        [
            perception_result(
                detected_object('red', 0.1, 0.2),
                detected_object('blue', 0.3, 0.4),
                detected_object('yellow', 0.5, 0.6),
            ),
            perception_result(),
        ]
    )
    install_responder(monkeypatch, responder)

    integration.query_colored_block_poses_from_robokudo(object())

    assert len(responder.submitted_goals) == 1


def test_missing_colors_exhaust_five_attempts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Stop after five attempts when target colors remain missing."""
    responder = SequencedAttemptResponder(
        [perception_result() for _ in range(5)]
    )
    install_responder(monkeypatch, responder)

    positions = integration.query_colored_block_poses_from_robokudo(object())

    assert positions == {}
    assert len(responder.submitted_goals) == 5
    assert all(goal.obj.type == 'block' for goal in responder.submitted_goals)


def test_only_target_colors_with_poses_are_transformed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Transform only target-colored object designators that have poses."""
    object_without_pose = DetectedObject(['red'], [])
    responder = SequencedAttemptResponder(
        [
            perception_result(
                object_without_pose,
                detected_object('green', 0.1, 0.2, 0.3),
                detected_object('blue', 0.4, 0.5, 0.6),
                detected_object('red', 0.7, 0.8, 0.9),
                detected_object('yellow', 1.0, 1.1, 1.2),
            )
        ]
    )
    install_responder(monkeypatch, responder)

    positions = integration.query_colored_block_poses_from_robokudo(object())

    assert positions == {
        'blue': (0.62, 0.5, 0.955),
        'red': (0.9199999999999999, 0.8, 0.955),
        'yellow': (1.22, 1.1, 0.955),
    }


def test_unavailable_server_preserves_error_behavior(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Raise the existing error when the action server is unavailable."""
    responder = SequencedAttemptResponder([], server_available=False)
    install_responder(monkeypatch, responder)

    with pytest.raises(
        RuntimeError,
        match='RoboKudo query action server is not available',
    ):
        integration.query_colored_block_poses_from_robokudo(object())

    assert responder.submitted_goals == []


def test_rejected_query_preserves_error_behavior(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Raise the existing error when RoboKudo rejects a query."""
    responder = SequencedAttemptResponder(
        [perception_result()],
        goals_accepted=False,
    )
    install_responder(monkeypatch, responder)

    with pytest.raises(
        RuntimeError, match='RoboKudo rejected the block query'
    ):
        integration.query_colored_block_poses_from_robokudo(object())

    assert len(responder.submitted_goals) == 1
