"""Tests for query-driven colored cube detection."""

from __future__ import annotations

import ast
import importlib.util
import sys
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from pathlib import Path
from types import ModuleType

import pytest


@dataclass
class GoalObject:
    """Represent the object constraints sent in a query goal."""

    type: str = ""
    """Requested object type."""

    color: list[str] = field(default_factory=list)
    """Requested object colors."""


class QueryMessage:
    """Mimic the query action types used by the integration."""

    @dataclass
    class Goal:
        """Represent one query goal."""

        obj: GoalObject = field(default_factory=GoalObject)
        """Object constraints for the query."""

    @dataclass
    class Result:
        """Represent a query result."""

        res: list[ResultObject] = field(default_factory=list)
        """Detected object designators."""


@dataclass(frozen=True)
class Position:
    """Represent a detected Cartesian position."""

    x: float
    """Position on the x-axis."""

    y: float
    """Position on the y-axis."""

    z: float = 0.0
    """Position on the z-axis."""


@dataclass(frozen=True)
class Pose:
    """Represent a pose containing a position."""

    position: Position
    """Detected position."""


@dataclass(frozen=True)
class StampedPose:
    """Represent the stamped pose structure returned by RoboKudo."""

    pose: Pose
    """Detected pose."""


@dataclass(frozen=True)
class Dimensions:
    """Represent three bounding-box side lengths in metres."""

    x: float
    """Length along the x-axis."""

    y: float
    """Length along the y-axis."""

    z: float
    """Length along the z-axis."""


@dataclass(frozen=True)
class ShapeSize:
    """Represent the bounding-box dimensions in a result."""

    dimensions: Dimensions
    """Three side lengths of the detected shape."""


@dataclass(frozen=True)
class ResultObject:
    """Represent the result fields consumed by the parser."""

    pose: list[StampedPose] = field(default_factory=list)
    """Detected poses."""

    shape_size: list[ShapeSize] = field(default_factory=list)
    """Detected shape dimensions."""

    color: list[str] = field(default_factory=list)
    """Optional detector-provided color labels."""


@dataclass(frozen=True)
class ActionResult:
    """Represent the result wrapper returned by an action goal."""

    result: QueryMessage.Result
    """RoboKudo query result."""


@dataclass(frozen=True)
class ImmediateFuture:
    """Complete callbacks immediately with a stored value."""

    value: object
    """Value produced by the future."""

    def add_done_callback(self, callback: Callable[[ImmediateFuture], None]) -> None:
        """Invoke a completion callback immediately."""
        callback(self)

    def result(self) -> object:
        """Return the stored result value."""
        return self.value


@dataclass(frozen=True)
class ConfigurableGoalHandle:
    """Represent an accepted or rejected action goal."""

    accepted: bool
    """Whether the action server accepted the goal."""

    query_result: QueryMessage.Result
    """Result returned for an accepted goal."""

    def get_result_async(self) -> ImmediateFuture:
        """Return the wrapped query result."""
        return ImmediateFuture(ActionResult(self.query_result))


@dataclass
class RecordingActionClient:
    """Record query goals and return configured results by requested color."""

    results_by_color: dict[str, list[QueryMessage.Result]]
    """Queued results for each requested color."""

    server_available: bool = True
    """Whether the action server is available."""

    goals_accepted: bool = True
    """Whether sent goals are accepted."""

    sent_goals: list[tuple[str, tuple[str, ...]]] = field(default_factory=list)
    """Object type and colors recorded for every goal."""

    def wait_for_server(self, timeout_sec: float) -> bool:
        """Return the configured server availability."""
        return self.server_available

    def send_goal_async(self, goal: QueryMessage.Goal) -> ImmediateFuture:
        """Record a goal and return its configured result."""
        requested_colors = tuple(goal.obj.color)
        self.sent_goals.append((goal.obj.type, requested_colors))
        requested_color = requested_colors[0]
        queued_results = self.results_by_color.get(requested_color, [])
        query_result = queued_results.pop(0) if queued_results else empty_result()
        return ImmediateFuture(
            ConfigurableGoalHandle(self.goals_accepted, query_result)
        )


@pytest.fixture
def integration_module(monkeypatch: pytest.MonkeyPatch) -> Iterator[ModuleType]:
    """Load the integration against lightweight ROS action mimics."""
    rclpy_module = ModuleType("rclpy")
    rclpy_action_module = ModuleType("rclpy.action")
    rclpy_node_module = ModuleType("rclpy.node")
    rclpy_task_module = ModuleType("rclpy.task")
    message_action_module = ModuleType("robokudo_msgs.action")
    message_package_module = ModuleType("robokudo_msgs")

    rclpy_action_module.ActionClient = object
    rclpy_node_module.Node = object
    rclpy_task_module.Future = object
    message_action_module.Query = QueryMessage

    monkeypatch.setitem(sys.modules, "rclpy", rclpy_module)
    monkeypatch.setitem(sys.modules, "rclpy.action", rclpy_action_module)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node_module)
    monkeypatch.setitem(sys.modules, "rclpy.task", rclpy_task_module)
    monkeypatch.setitem(sys.modules, "robokudo_msgs", message_package_module)
    monkeypatch.setitem(sys.modules, "robokudo_msgs.action", message_action_module)

    repository_root = Path(__file__).parents[2]
    integration_path = repository_root / (
        "robokudo/src/robokudo/descriptors/analysis_engines/"
        "robokudo_cram_integration.py"
    )
    module_name = "robokudo_cram_integration_under_test"
    specification = importlib.util.spec_from_file_location(
        module_name, integration_path
    )
    assert specification is not None
    assert specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    monkeypatch.setitem(sys.modules, module_name, module)
    specification.loader.exec_module(module)
    yield module


def detected_result(
    dimensions: tuple[float, float, float] = (0.05, 0.05, 0.05),
    position: tuple[float, float] = (0.4, -0.2),
) -> QueryMessage.Result:
    """Create a result containing one posed, sized candidate."""
    result_object = ResultObject(
        pose=[StampedPose(Pose(Position(position[0], position[1])))],
        shape_size=[ShapeSize(Dimensions(*dimensions))],
    )
    return QueryMessage.Result(res=[result_object])


def empty_result() -> QueryMessage.Result:
    """Create a result without candidates."""
    return QueryMessage.Result()


def test_query_builds_single_color_goals_in_deterministic_order_and_stops(
    integration_module: ModuleType,
) -> None:
    """Request blue, red, and yellow once when every cube is accepted."""
    action_client = RecordingActionClient(
        {
            "blue": [detected_result(position=(0.1, 0.2))],
            "red": [detected_result(position=(0.3, 0.4))],
            "yellow": [detected_result(position=(0.5, 0.6))],
        }
    )

    positions = integration_module.ColoredBlockPoseQuery(
        object(), action_client
    ).execute()

    assert action_client.sent_goals == [
        ("block", ("blue",)),
        ("block", ("red",)),
        ("block", ("yellow",)),
    ]
    assert positions == {
        "blue": (0.1, 0.2, 0.95),
        "red": (0.3, 0.4, 0.95),
        "yellow": (0.5, 0.6, 0.95),
    }


@pytest.mark.parametrize(
    "rejected_result",
    [
        QueryMessage.Result(
            res=[
                ResultObject(
                    pose=[StampedPose(Pose(Position(0.3, 0.4)))],
                )
            ]
        ),
        QueryMessage.Result(
            res=[
                ResultObject(
                    shape_size=[ShapeSize(Dimensions(0.05, 0.05, 0.05))],
                )
            ]
        ),
        detected_result((0.029, 0.05, 0.05)),
        detected_result((0.05, 0.071, 0.05)),
    ],
)
def test_query_retries_only_the_color_without_an_acceptable_candidate(
    integration_module: ModuleType,
    rejected_result: QueryMessage.Result,
) -> None:
    """Skip accepted colors while retrying a rejected candidate."""
    action_client = RecordingActionClient(
        {
            "blue": [detected_result()],
            "red": [rejected_result, detected_result()],
            "yellow": [detected_result()],
        }
    )

    positions = integration_module.ColoredBlockPoseQuery(
        object(), action_client
    ).execute()

    assert action_client.sent_goals == [
        ("block", ("blue",)),
        ("block", ("red",)),
        ("block", ("yellow",)),
        ("block", ("red",)),
    ]
    assert set(positions) == {"blue", "red", "yellow"}


def test_query_returns_partial_mapping_after_five_attempts_per_missing_color(
    integration_module: ModuleType,
) -> None:
    """Stop requesting an undetected color after five fresh frames."""
    action_client = RecordingActionClient(
        {
            "blue": [empty_result() for _ in range(5)],
            "red": [detected_result()],
            "yellow": [detected_result()],
        }
    )

    positions = integration_module.ColoredBlockPoseQuery(
        object(), action_client
    ).execute()

    requested_colors = [goal[1][0] for goal in action_client.sent_goals]
    assert requested_colors == [
        "blue",
        "red",
        "yellow",
        "blue",
        "blue",
        "blue",
        "blue",
    ]
    assert set(positions) == {"red", "yellow"}


@pytest.mark.parametrize(
    "dimensions",
    [
        (0.05, 0.05, 0.05),
        (0.03, 0.03, 0.03),
        (0.07, 0.07, 0.07),
        (0.03, 0.05, 0.07),
    ],
)
def test_parser_accepts_inclusive_safe_cube_dimensions(
    integration_module: ModuleType,
    dimensions: tuple[float, float, float],
) -> None:
    """Accept candidates whose three sides are within the safety range."""
    positions_by_color = integration_module.ColoredBlockPoseParser().parse(
        detected_result(dimensions), integration_module.BlockColor.BLUE
    )

    assert positions_by_color == {"blue": (0.4, -0.2, 0.95)}


@pytest.mark.parametrize(
    "result",
    [
        QueryMessage.Result(
            res=[ResultObject(shape_size=[ShapeSize(Dimensions(0.05, 0.05, 0.05))])]
        ),
        QueryMessage.Result(
            res=[ResultObject(pose=[StampedPose(Pose(Position(0.4, -0.2)))])]
        ),
        detected_result((0.029, 0.05, 0.05)),
        detected_result((0.05, 0.071, 0.05)),
        detected_result((0.05, 0.05, 0.08)),
    ],
)
def test_parser_rejects_incomplete_or_unsafe_candidates(
    integration_module: ModuleType,
    result: QueryMessage.Result,
) -> None:
    """Reject candidates missing geometry or outside the safety range."""
    positions_by_color = integration_module.ColoredBlockPoseParser().parse(
        result, integration_module.BlockColor.RED
    )

    assert positions_by_color == {}


def test_parser_associates_candidate_with_the_requested_color(
    integration_module: ModuleType,
) -> None:
    """Use the goal color because color segmentation defines the candidate."""
    result = detected_result()
    result.res[0].color.append("red")

    positions_by_color = integration_module.ColoredBlockPoseParser().parse(
        result, integration_module.BlockColor.YELLOW
    )

    assert positions_by_color == {"yellow": (0.4, -0.2, 0.95)}


def test_query_preserves_server_unavailable_and_rejected_goal_errors(
    integration_module: ModuleType,
) -> None:
    """Raise clear errors for the existing action failure modes."""
    unavailable_client = RecordingActionClient({}, server_available=False)
    rejected_client = RecordingActionClient(
        {"blue": [empty_result()]}, goals_accepted=False
    )

    with pytest.raises(RuntimeError, match="server is not available"):
        integration_module.ColoredBlockPoseQuery(object(), unavailable_client).execute()

    with pytest.raises(RuntimeError, match="rejected the block query"):
        integration_module.ColoredBlockPoseQuery(object(), rejected_client).execute()


def test_stacking_pipeline_is_query_driven_color_segmentation() -> None:
    """Use one color contour and bounding-box poses in the stacking pipeline."""
    repository_root = Path(__file__).parents[2]
    pipeline_path = repository_root / (
        "robokudo/src/robokudo/descriptors/analysis_engines/stacking_robokudo.py"
    )
    source = pipeline_path.read_text(encoding="utf-8")
    syntax_tree = ast.parse(source)
    imported_classes = {
        imported.name
        for node in ast.walk(syntax_tree)
        if isinstance(node, ast.ImportFrom)
        for imported in node.names
    }

    assert "ImageClusterExtractor" in imported_classes
    assert "ClusterPoseBBAnnotator" in imported_classes
    assert "PointCloudClusterExtractor" not in imported_classes
    assert "ClusterPosePCAAnnotator" not in imported_classes
    assert "(22, 130, 85)" in source
    assert "(65, 255, 255)" in source
    assert "num_of_objects = 1" in source
