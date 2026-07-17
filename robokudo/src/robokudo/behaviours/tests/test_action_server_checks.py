"""Tests for query action-server lifecycle checks."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass

from py_trees.blackboard import Blackboard
from py_trees.common import Status

import pytest

from robokudo.behaviours.action_server_checks import ActionServerPresentAndDone
from robokudo.identifier import BBIdentifier


@dataclass(frozen=True)
class PendingQueryLifecycle:
    """Represent an accepted query that the pipeline has not processed."""

    def is_active(self) -> bool:
        """Return whether an accepted query owns the action server."""
        return True

    def has_pending_query(self) -> bool:
        """Return whether the pipeline still needs to receive the query."""
        return True


@dataclass(frozen=True)
class ProcessingQueryLifecycle:
    """Represent a query that the pipeline is already processing."""

    def is_active(self) -> bool:
        """Return whether a query owns the action server."""
        return True

    def has_pending_query(self) -> bool:
        """Return whether the pipeline still needs to receive the query."""
        return False


@pytest.fixture
def query_server_blackboard() -> Iterator[Blackboard]:
    """Provide an isolated blackboard configured for a query pipeline."""
    Blackboard.clear()
    blackboard = Blackboard()
    blackboard.set(BBIdentifier.QUERY_SERVER_IN_PIPELINE, True)
    yield blackboard
    Blackboard.clear()


def test_pending_query_does_not_block_pipeline_initialization(
    query_server_blackboard: Blackboard,
) -> None:
    """Allow the pipeline to reach the annotator for a pending query."""
    query_server_blackboard.set(
        BBIdentifier.QUERY_SERVER,
        PendingQueryLifecycle(),
    )

    status = ActionServerPresentAndDone().update()

    assert status is Status.SUCCESS


def test_processing_query_blocks_next_pipeline_iteration(
    query_server_blackboard: Blackboard,
) -> None:
    """Wait for a processed query result before starting another iteration."""
    query_server_blackboard.set(
        BBIdentifier.QUERY_SERVER,
        ProcessingQueryLifecycle(),
    )

    status = ActionServerPresentAndDone().update()

    assert status is Status.RUNNING
