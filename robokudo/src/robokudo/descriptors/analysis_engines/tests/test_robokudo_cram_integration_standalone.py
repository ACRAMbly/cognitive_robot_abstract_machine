"""Tests for standalone colored-block pose queries."""

from __future__ import annotations

import importlib.util
import sys
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType

import pytest


@dataclass
class StandaloneGoalObject:
    """Represent the object constraints of a standalone query."""

    type: str = ""
    """Requested object type."""

    color: list[str] = field(default_factory=list)
    """Requested object colors."""


class StandaloneQuery:
    """Mimic the query action messages used during standalone execution."""

    @dataclass
    class Goal:
        """Represent one standalone action goal."""

        obj: StandaloneGoalObject = field(default_factory=StandaloneGoalObject)
        """Object constraints carried by the goal."""

    @dataclass
    class Result:
        """Represent detected objects returned by RoboKudo."""

        res: list[StandaloneDetectedObject] = field(default_factory=list)
        """Detected object designators."""


@dataclass(frozen=True)
class StandalonePosition:
    """Represent a detected Cartesian position."""

    x: float
    """Position on the x-axis."""

    y: float
    """Position on the y-axis."""


@dataclass(frozen=True)
class StandalonePose:
    """Represent a pose containing a position."""

    position: StandalonePosition
    """Detected Cartesian position."""


@dataclass(frozen=True)
class StandaloneStampedPose:
    """Represent the stamped pose returned by RoboKudo."""

    pose: StandalonePose
    """Detected pose."""


@dataclass(frozen=True)
class StandaloneDimensions:
    """Represent bounding-box side lengths in metres."""

    x: float = 0.05
    """Length along the x-axis."""

    y: float = 0.05
    """Length along the y-axis."""

    z: float = 0.05
    """Length along the z-axis."""


@dataclass(frozen=True)
class StandaloneShapeSize:
    """Represent the bounding-box dimensions of a detected block."""

    dimensions: StandaloneDimensions = field(default_factory=StandaloneDimensions)
    """Three side lengths of the block."""


@dataclass(frozen=True)
class StandaloneDetectedObject:
    """Represent a safely sized object with a detected pose."""

    pose: list[StandaloneStampedPose]
    """Detected poses."""

    shape_size: list[StandaloneShapeSize] = field(
        default_factory=lambda: [StandaloneShapeSize()]
    )
    """Detected bounding-box dimensions."""


@dataclass(frozen=True)
class StandaloneActionResult:
    """Represent the action wrapper around a query result."""

    result: StandaloneQuery.Result
    """RoboKudo query result."""


@dataclass
class SpinCompletedFuture:
    """Represent a future that completes only while its node is spun."""

    value: object
    """Value made available after completion."""

    completed: bool = False
    """Whether executor progress completed the future."""

    def add_done_callback(
        self, callback: Callable[[SpinCompletedFuture], None]
    ) -> None:
        """Reject callback-only waiting before executor progress occurs."""
        raise AssertionError("Standalone futures require the node to be spun.")

    def complete(self) -> None:
        """Complete the future through simulated executor progress."""
        self.completed = True

    def result(self) -> object:
        """Return the result after executor progress completes the future."""
        assert self.completed
        return self.value


@dataclass(frozen=True)
class StandaloneGoalHandle:
    """Represent an accepted query goal."""

    query_result: StandaloneQuery.Result
    """Result returned for the accepted goal."""

    accepted: bool = True
    """Whether RoboKudo accepted the goal."""

    def get_result_async(self) -> SpinCompletedFuture:
        """Return a future completed by spinning the standalone node."""
        return SpinCompletedFuture(StandaloneActionResult(self.query_result))


@dataclass
class StandaloneNode:
    """Represent a node that is not attached to an executor."""

    executor: object | None = None
    """Last executor associated with the standalone node."""


@dataclass
class StandaloneActionClient:
    """Return one safely sized pose for every requested block color."""

    node: StandaloneNode
    """Node used by the action client."""

    action_type: object
    """Action message type used by the client."""

    action_name: str
    """ROS action name used by the client."""

    def wait_for_server(self, timeout_sec: float) -> bool:
        """Report that the simulated RoboKudo server is available."""
        return True

    def send_goal_async(self, goal: StandaloneQuery.Goal) -> SpinCompletedFuture:
        """Return a pose future for the goal's requested color."""
        positions_by_color = {
            "blue": StandalonePosition(0.1, 0.2),
            "red": StandalonePosition(0.3, 0.4),
            "yellow": StandalonePosition(0.5, 0.6),
        }
        position = positions_by_color[goal.obj.color[0]]
        detected_object = StandaloneDetectedObject(
            pose=[StandaloneStampedPose(StandalonePose(position))]
        )
        query_result = StandaloneQuery.Result(res=[detected_object])
        return SpinCompletedFuture(StandaloneGoalHandle(query_result))


@pytest.fixture
def standalone_integration_module(
    monkeypatch: pytest.MonkeyPatch,
) -> Iterator[tuple[ModuleType, list[SpinCompletedFuture]]]:
    """Load the integration with ROS futures requiring executor progress."""
    spun_futures: list[SpinCompletedFuture] = []
    rclpy_module = ModuleType("rclpy")
    rclpy_action_module = ModuleType("rclpy.action")
    message_action_module = ModuleType("robokudo_msgs.action")
    message_package_module = ModuleType("robokudo_msgs")

    def spin_until_future_complete(
        node: StandaloneNode, future: SpinCompletedFuture
    ) -> None:
        """Simulate executor progress for one standalone future."""
        if node.executor is None:
            node.executor = object()
        future.complete()
        spun_futures.append(future)

    rclpy_module.spin_until_future_complete = spin_until_future_complete
    rclpy_action_module.ActionClient = StandaloneActionClient
    message_action_module.Query = StandaloneQuery

    monkeypatch.setitem(sys.modules, "rclpy", rclpy_module)
    monkeypatch.setitem(sys.modules, "rclpy.action", rclpy_action_module)
    monkeypatch.setitem(sys.modules, "robokudo_msgs", message_package_module)
    monkeypatch.setitem(sys.modules, "robokudo_msgs.action", message_action_module)

    integration_path = (
        Path(__file__).parents[1] / "robokudo_cram_integration.py"
    )
    module_name = "standalone_robokudo_cram_integration_under_test"
    specification = importlib.util.spec_from_file_location(module_name, integration_path)
    assert specification is not None
    assert specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    monkeypatch.setitem(sys.modules, module_name, module)
    specification.loader.exec_module(module)
    yield module, spun_futures


def test_standalone_query_spins_futures_and_returns_all_three_poses(
    standalone_integration_module: tuple[ModuleType, list[SpinCompletedFuture]],
) -> None:
    """Spin action futures and return blue, red, and yellow block poses."""
    integration_module, spun_futures = standalone_integration_module

    positions = integration_module.query_colored_block_poses_from_robokudo(
        StandaloneNode()
    )

    assert positions == {
        "blue": (0.1, 0.2, 0.95),
        "red": (0.3, 0.4, 0.95),
        "yellow": (0.5, 0.6, 0.95),
    }
    assert len(spun_futures) == 6
