"""
Queries about a robot's past execution behaviour, expressed in the KRROOD Entity Query Language.

The module builds a representative example plan — a sequential pick-and-place task wrapped
in a TryInOrderNode with a MonitorNode — then executes every query against it.
Each query is prefixed with a comment that states the plain-English question it answers.
"""

from __future__ import annotations

import datetime
from dataclasses import dataclass, field
from typing import Optional

import krrood.entity_query_language.factories as eql
from coraplex.datastructures.enums import TaskStatus
from coraplex.language import MonitorNode, SequentialNode, TryInOrderNode
from coraplex.plans.plan import Plan
from coraplex.plans.plan_node import ActionNode, PlanNode


# ---------------------------------------------------------------------------
# Minimal stub objects so the plan can be built without a live robot / world
# ---------------------------------------------------------------------------

@dataclass(eq=False)
class _StubNode(PlanNode):
    """A concrete PlanNode that records a fake execution trace when performed."""

    label: str = ""
    _forced_status: TaskStatus = TaskStatus.SUCCEEDED
    _duration_seconds: float = 1.0
    _failure: Optional[Exception] = field(default=None, repr=False)

    def _perform(self):
        pass  # execution is pre-filled below


def _stamp(node: PlanNode, status: TaskStatus, start_offset: float, duration: float,
           reason=None) -> None:
    """Back-fill timing and status fields to simulate a completed execution."""
    t0 = datetime.datetime(2026, 6, 18, 10, 0, 0) + datetime.timedelta(seconds=start_offset)
    node.start_time = t0
    node.end_time = t0 + datetime.timedelta(seconds=duration)
    node.status = status
    node.reason = reason


# ---------------------------------------------------------------------------
# Build a representative plan
#
# Task: navigate → pick up object → place object
# Wrapped in a TryInOrderNode so a fallback is available.
# A MonitorNode watches for collisions and interrupts if triggered.
#
# Simulated outcome:
#   - First navigate attempt FAILED (NavigationGoalNotReachedError)
#   - TryInOrderNode fell through to a second navigate attempt (SUCCEEDED)
#   - PickUp SUCCEEDED
#   - Place SUCCEEDED
#   - MonitorNode fired once (INTERRUPTED) before the second navigate
# ---------------------------------------------------------------------------

from coraplex.plans.failures import NavigationGoalNotReachedError

plan = Plan()

# --- Root: TryInOrderNode -------------------------------------------------
root = TryInOrderNode()
plan.add_node(root)
_stamp(root, TaskStatus.SUCCEEDED, start_offset=0.0, duration=12.5)

# --- MonitorNode (collision watch) ----------------------------------------
monitor = MonitorNode(condition=lambda: False, behavior=None)  # type: ignore[arg-type]
plan.add_edge(root, monitor)
_stamp(monitor, TaskStatus.INTERRUPTED, start_offset=0.0, duration=2.1)

# --- First navigate attempt (FAILED) --------------------------------------
nav_fail = _StubNode(label="NavigateAction[attempt=1]")
plan.add_edge(root, nav_fail)
_stamp(nav_fail, TaskStatus.FAILED, start_offset=0.2, duration=3.0,
       reason=NavigationGoalNotReachedError("goal unreachable"))

# --- Second navigate attempt (SUCCEEDED) ----------------------------------
nav_ok = _StubNode(label="NavigateAction[attempt=2]")
plan.add_edge(root, nav_ok)
_stamp(nav_ok, TaskStatus.SUCCEEDED, start_offset=3.5, duration=4.0)

# --- Sequential: pick then place ------------------------------------------
seq = SequentialNode()
plan.add_edge(root, seq)
_stamp(seq, TaskStatus.SUCCEEDED, start_offset=7.5, duration=5.0)

pick = _StubNode(label="PickUpAction")
plan.add_edge(seq, pick)
_stamp(pick, TaskStatus.SUCCEEDED, start_offset=7.5, duration=2.5)

place = _StubNode(label="PlaceAction")
plan.add_edge(seq, place)
_stamp(place, TaskStatus.SUCCEEDED, start_offset=10.0, duration=2.5)


# ---------------------------------------------------------------------------
# Helper: a variable ranging over every node in the plan
# ---------------------------------------------------------------------------

def _all_nodes():
    return eql.variable(PlanNode, domain=plan.plan_graph.nodes())


# ===========================================================================
# QUERIES
# ===========================================================================

# ---------------------------------------------------------------------------
# "What did you just do?" — leaf nodes that finished successfully
# ---------------------------------------------------------------------------
def what_did_you_do():
    n = _all_nodes()
    return eql.an(eql.entity(n).where(n.is_leaf, n.status == TaskStatus.SUCCEEDED))


# ---------------------------------------------------------------------------
# "Walk me through what you did in order."
# ---------------------------------------------------------------------------
def walk_through_in_order():
    n = _all_nodes()
    return (
        eql.an(eql.entity(n).where(n.status == TaskStatus.SUCCEEDED))
        .ordered_by(n.start_time)
    )


# ---------------------------------------------------------------------------
# "How long did the whole task take?"
# ---------------------------------------------------------------------------
def total_task_duration():
    n = _all_nodes()
    return eql.the(eql.entity(n).where(n.parent == None))  # noqa: E711 — EQL uses ==


# ---------------------------------------------------------------------------
# "How long did each step take?"
# ---------------------------------------------------------------------------
def duration_per_step():
    n = _all_nodes()
    duration = n.end_time - n.start_time
    return (
        eql.set_of(n, duration)
        .where(n.end_time != None)  # noqa: E711
        .ordered_by(n.start_time)
    )


# ---------------------------------------------------------------------------
# "Did anything go wrong?"
# ---------------------------------------------------------------------------
def did_anything_go_wrong():
    n = _all_nodes()
    return eql.an(eql.entity(n).where(n.status == TaskStatus.FAILED))


# ---------------------------------------------------------------------------
# "Why did you fail at that step?"
# ---------------------------------------------------------------------------
def why_did_you_fail():
    n = _all_nodes()
    return eql.an(eql.entity(n.reason).where(n.status == TaskStatus.FAILED))


# ---------------------------------------------------------------------------
# "How many times did you retry before giving up?"
# ---------------------------------------------------------------------------
def how_many_retries():
    n = _all_nodes()
    return eql.count(n).where(n.status == TaskStatus.FAILED)


# ---------------------------------------------------------------------------
# "Which fallback did you end up using?"
# ---------------------------------------------------------------------------
def which_fallback_was_used():
    n = _all_nodes()
    return eql.an(
        eql.entity(n).where(
            n.status == TaskStatus.SUCCEEDED,
            n.parent.status == TaskStatus.SUCCEEDED,
            # The node's left siblings all failed — it is therefore a fallback that ran
            eql.for_all(
                eql.variable(PlanNode, domain=n.left_siblings),
                lambda s: s.status == TaskStatus.FAILED,
            ),
        )
    )


# ---------------------------------------------------------------------------
# "Were you ever interrupted? What caused it?"
# ---------------------------------------------------------------------------
def were_you_interrupted():
    n = _all_nodes()
    return eql.an(eql.entity(n).where(n.status == TaskStatus.INTERRUPTED))


# ---------------------------------------------------------------------------
# "Was there a point where you were paused?"
# ---------------------------------------------------------------------------
def were_you_ever_paused():
    n = _all_nodes()
    return eql.an(eql.entity(n).where(n.status == TaskStatus.PAUSE))


# ---------------------------------------------------------------------------
# "Which step took the longest?"
# ---------------------------------------------------------------------------
def which_step_took_longest():
    n = _all_nodes()
    return eql.max(
        n,
        key=lambda node: (node.end_time - node.start_time).total_seconds()
        if node.end_time is not None else 0.0,
    )


# ---------------------------------------------------------------------------
# "Were all subtasks successful, or did some fail?"
# — returns a breakdown of node counts per status
# ---------------------------------------------------------------------------
def status_breakdown():
    n = _all_nodes()
    return (
        eql.set_of(n.status, c := eql.count(n))
        .grouped_by(n.status)
        .ordered_by(c, descending=True)
    )


# ---------------------------------------------------------------------------
# "Did any monitored condition trigger during execution?"
# ---------------------------------------------------------------------------
def did_monitor_trigger():
    m = eql.variable(MonitorNode, domain=plan.plan_graph.nodes())
    return eql.an(eql.entity(m).where(m.status == TaskStatus.INTERRUPTED))


# ---------------------------------------------------------------------------
# "What world modifications did you make?"
# — ActionNodes carry execution_data with added_world_modifications
# ---------------------------------------------------------------------------
def what_world_modifications_were_made():
    n = eql.variable(ActionNode, domain=plan.plan_graph.nodes())
    return eql.an(
        eql.entity(n.execution_data.added_world_modifications)
        .where(
            n.status == TaskStatus.SUCCEEDED,
            n.execution_data != None,  # noqa: E711
        )
    )


# ---------------------------------------------------------------------------
# Demo: evaluate every query and print results
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    def _sep(title: str):
        print(f"\n{'=' * 60}")
        print(f"  {title}")
        print('=' * 60)

    _sep("What did you just do?")
    for node in what_did_you_do().evaluate():
        print(f"  {node.label!r}  [{node.status.name}]")

    _sep("Walk me through what you did in order")
    for node in walk_through_in_order().evaluate():
        print(f"  {node.start_time:%H:%M:%S}  {node.label!r}")

    _sep("How long did the whole task take?")
    root_node = total_task_duration().evaluate()
    print(f"  {(root_node.end_time - root_node.start_time).total_seconds():.1f}s")

    _sep("How long did each step take?")
    for result in duration_per_step().evaluate():
        node = result[eql.variable(PlanNode)]
        print(f"  {node.label!r}: {result}")

    _sep("Did anything go wrong?")
    failed = list(did_anything_go_wrong().evaluate())
    if failed:
        for node in failed:
            print(f"  {node.label!r} — reason: {node.reason}")
    else:
        print("  Nothing failed.")

    _sep("Why did you fail at that step?")
    for reason in why_did_you_fail().evaluate():
        print(f"  {reason}")

    _sep("How many times did you retry?")
    print(f"  {how_many_retries().evaluate()}")

    _sep("Which fallback did you end up using?")
    for node in which_fallback_was_used().evaluate():
        print(f"  {node.label!r}")

    _sep("Were you ever interrupted?")
    for node in were_you_interrupted().evaluate():
        print(f"  {type(node).__name__} was interrupted")

    _sep("Were you ever paused?")
    paused = list(were_you_ever_paused().evaluate())
    print(f"  {'Yes' if paused else 'No'}")

    _sep("Which step took the longest?")
    longest = which_step_took_longest().evaluate()
    print(f"  {longest.label!r}: "
          f"{(longest.end_time - longest.start_time).total_seconds():.1f}s")

    _sep("Status breakdown")
    for result in status_breakdown().evaluate():
        print(f"  {result}")

    _sep("Did any monitor trigger?")
    monitors = list(did_monitor_trigger().evaluate())
    print(f"  {'Yes — ' + str(len(monitors)) + ' monitor(s) fired' if monitors else 'No'}")

    _sep("What world modifications were made?")
    for mod in what_world_modifications_were_made().evaluate():
        print(f"  {mod}")
