"""Tests for action-future waiting in the CRAM integration."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import pytest

from robokudo.descriptors.analysis_engines import (
    robokudo_cram_integration as integration,
)
from robokudo_msgs.action import Query


@dataclass(frozen=True)
class CompletedFuture:
    """Action future that invokes callbacks immediately."""

    value: object
    """Value returned by the completed future."""

    def add_done_callback(
        self,
        callback: Callable[[CompletedFuture], None],
    ) -> None:
        """Invoke a completion callback for the finished future."""
        callback(self)

    def result(self) -> object:
        """Return the completed value."""
        return self.value


@dataclass(frozen=True)
class CompletedActionResult:
    """Completed action result returned by RoboKudo."""

    result: Query.Result
    """RoboKudo perception query result."""


@dataclass(frozen=True)
class AcceptedGoal:
    """Accepted RoboKudo goal with an immediately available result."""

    query_result: Query.Result
    """Result associated with the accepted goal."""

    accepted: bool = True
    """Whether RoboKudo accepted the goal."""

    def get_result_async(self) -> CompletedFuture:
        """Return the completed RoboKudo result future."""
        return CompletedFuture(CompletedActionResult(self.query_result))


@dataclass(frozen=True)
class CallbackActionClient:
    """Action client whose futures complete through callbacks."""

    query_result: Query.Result
    """Result returned for the submitted goal."""

    def send_goal_async(self, goal: Query.Goal) -> CompletedFuture:
        """Return an accepted goal through a completed future."""
        return CompletedFuture(AcceptedGoal(self.query_result))


def test_request_waits_for_callbacks_without_spinning_node(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Do not spin a node that may already belong to an executor."""

    def reject_nested_spin(node: object, future: object) -> None:
        """Fail when the integration attempts to start another spin."""
        raise AssertionError('The integration attempted a nested ROS spin.')

    monkeypatch.setattr(
        integration.rclpy,
        'spin_until_future_complete',
        reject_nested_spin,
    )
    expected_result = Query.Result()
    query = integration.ColoredBlockPoseQuery(
        object(),
        CallbackActionClient(expected_result),
    )

    actual_result = query._request_fresh_frame()

    assert actual_result is expected_result
