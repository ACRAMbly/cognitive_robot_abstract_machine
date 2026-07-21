"""
Tests for the typed per-pass state collaborators on EvaluationContext.
"""

import uuid

from ordered_set import OrderedSet

from krrood.entity_query_language.evaluation_context import (
    ActiveConditionsRoot,
    EvaluatedExpressionIds,
)


class _NodeStub:
    """
    Minimal stand-in for a SymbolicExpression: only ``_id_`` and ``_root_`` are needed by
    these collaborators. ``_root_`` defaults to itself, matching an unattached node's own
    structural root.
    """

    def __init__(self):
        self._id_ = uuid.uuid4()
        self._root_ = self


def test_active_conditions_root_claims_first_node_and_ignores_later_claims():
    tracking = ActiveConditionsRoot()
    originating = _NodeStub()
    first = _NodeStub()
    second = _NodeStub()

    tracking.claim(first, originating)
    tracking.claim(second, originating)

    assert tracking.is_active_root(first)
    assert not tracking.is_active_root(second)


def test_active_conditions_root_resolves_by_claim_not_by_construction_order():
    """
    The whole point of this class: whichever node claims the pass first is the active
    root, regardless of any other node's structural/construction history.
    """
    tracking = ActiveConditionsRoot()
    originating = _NodeStub()
    node = _NodeStub()

    assert not tracking.is_active_root(node), "unclaimed pass must not match any node"
    tracking.claim(node, originating)
    assert tracking.is_active_root(node)


def test_active_conditions_root_has_condition_when_claimed_root_differs_from_originator():
    tracking = ActiveConditionsRoot()
    originating = _NodeStub()
    filter_condition = _NodeStub()

    tracking.claim(filter_condition, originating)

    assert tracking.has_condition()


def test_active_conditions_root_has_no_condition_when_claimed_root_is_the_originators_root():
    """
    A Filter-less evaluation's own _conditions_root_ falls back to its plain _root_, so
    claiming with that same value must record "no real condition" for this pass.
    """
    tracking = ActiveConditionsRoot()
    originating = _NodeStub()

    tracking.claim(originating._root_, originating)

    assert not tracking.has_condition()


def test_active_conditions_root_has_condition_defaults_false_before_any_claim():
    tracking = ActiveConditionsRoot()

    assert not tracking.has_condition()


def test_evaluated_expression_ids_records_and_iterates():
    tracked = EvaluatedExpressionIds()
    first, second = uuid.uuid4(), uuid.uuid4()

    tracked.record(first)
    tracked.record(second)

    assert set(tracked) == {first, second}


def test_evaluated_expression_ids_merges_ids_from_another_set():
    tracked = EvaluatedExpressionIds()
    tracked.record(uuid.uuid4())
    other_ids = OrderedSet([uuid.uuid4(), uuid.uuid4()])

    tracked.merge(other_ids)

    assert other_ids.issubset(set(tracked))


def test_evaluated_expression_ids_snapshot_reflects_recorded_ids():
    tracked = EvaluatedExpressionIds()
    first = uuid.uuid4()
    tracked.record(first)

    snapshot = tracked.snapshot()

    assert set(snapshot) == {first}


def test_evaluated_expression_ids_snapshot_is_reused_while_set_is_unchanged():
    """
    The id set only grows, so its length is a valid version key: two snapshots taken
    without an intervening record() share the same cached object.
    """
    tracked = EvaluatedExpressionIds()
    tracked.record(uuid.uuid4())

    first_snapshot = tracked.snapshot()
    second_snapshot = tracked.snapshot()

    assert first_snapshot is second_snapshot


def test_evaluated_expression_ids_snapshot_refreshes_after_growth():
    tracked = EvaluatedExpressionIds()
    tracked.record(uuid.uuid4())
    stale_snapshot = tracked.snapshot()

    tracked.record(uuid.uuid4())
    fresh_snapshot = tracked.snapshot()

    assert fresh_snapshot is not stale_snapshot
    assert len(fresh_snapshot) == 2
