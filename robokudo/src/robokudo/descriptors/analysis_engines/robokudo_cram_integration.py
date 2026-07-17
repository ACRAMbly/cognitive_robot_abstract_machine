"""Query RoboKudo for safely sized colored-block positions."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import Enum
from threading import Event

import rclpy
from rclpy.action import ActionClient
from typing_extensions import TYPE_CHECKING

from robokudo_msgs.action import Query

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.task import Future
    from robokudo_msgs.msg import ShapeSize


class BlockColor(str, Enum):
    """Colors queried by the block-stacking integration."""

    BLUE = "blue"
    RED = "red"
    YELLOW = "yellow"


def _wait_for_future(future: Future) -> None:
    """Wait without attempting to spin the existing ROS executor."""
    completed = Event()
    future.add_done_callback(lambda _: completed.set())
    completed.wait()


@dataclass(frozen=True)
class FutureCompletion:
    """Complete action futures in standalone and executor-owned contexts."""

    node: Node
    """ROS node associated with the action futures."""

    spins_node: bool
    """Whether this completion strategy owns executor progress."""

    @classmethod
    def for_node(cls, node: Node) -> FutureCompletion:
        """Create a completion strategy from the node's initial ownership."""
        return cls(node=node, spins_node=node.executor is None)

    def wait(self, future: Future) -> None:
        """Wait for a future using the executor context that owns the node."""
        if self.spins_node:
            rclpy.spin_until_future_complete(self.node, future)
            return

        _wait_for_future(future)


@dataclass(frozen=True)
class ColoredBlockPoseParser:
    """Extract a safely sized block position from a color query result."""

    minimum_side_length: float = 0.03
    """Inclusive minimum accepted bounding-box side length in metres."""

    maximum_side_length: float = 0.07
    """Inclusive maximum accepted bounding-box side length in metres."""

    output_height: float = 0.95
    """Height used by the existing position transformation."""

    def parse(
        self,
        result: Query.Result,
        requested_color: BlockColor,
    ) -> dict[str, tuple[float, float, float]]:
        """Return a safely sized position keyed by the requested color."""
        for object_designator in result.res:
            if not object_designator.pose or not object_designator.shape_size:
                continue

            if not self._has_safe_dimensions(object_designator.shape_size):
                continue

            position = object_designator.pose[0].pose.position
            return {requested_color.value: (position.x, position.y, self.output_height)}

        return {}

    def _has_safe_dimensions(self, shape_sizes: Sequence[ShapeSize]) -> bool:
        """Return whether every reported side is within the safe range."""
        for shape_size in shape_sizes:
            dimensions = shape_size.dimensions
            side_lengths = (dimensions.x, dimensions.y, dimensions.z)
            if not all(
                self.minimum_side_length <= side_length <= self.maximum_side_length
                for side_length in side_lengths
            ):
                return False

        return True


@dataclass(frozen=True)
class ColoredBlockPoseQuery:
    """Collect target-colored block positions across fresh query attempts."""

    node: Node
    """ROS node used to complete action futures."""

    action_client: ActionClient
    """Client connected to the RoboKudo query action."""

    maximum_attempts: int = 5
    """Maximum fresh-frame queries issued for each missing color."""

    parser: ColoredBlockPoseParser = field(default_factory=ColoredBlockPoseParser)
    """Parser that validates and transforms target-color poses."""

    colors: tuple[BlockColor, ...] = (
        BlockColor.BLUE,
        BlockColor.RED,
        BlockColor.YELLOW,
    )
    """Deterministic color query order."""

    future_completion: FutureCompletion | None = None
    """Optional executor-aware action future completion strategy."""

    def execute(self) -> dict[str, tuple[float, float, float]]:
        """Query until all target colors are found or attempts run out."""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("RoboKudo query action server is not available.")

        positions_by_color: dict[str, tuple[float, float, float]] = {}
        attempts_remaining = self.maximum_attempts

        while attempts_remaining > 0:
            for color in self.colors:
                if color.value in positions_by_color:
                    continue

                result = self._request_fresh_frame(color)
                positions_by_color.update(self.parser.parse(result, color))

            if len(positions_by_color) == len(self.colors):
                return positions_by_color

            attempts_remaining -= 1

        return positions_by_color

    def _request_fresh_frame(self, color: BlockColor) -> Query.Result:
        """Send one single-color block query and return its result."""
        goal = Query.Goal()
        goal.obj.type = "block"
        goal.obj.color.append(color.value)

        send_future = self.action_client.send_goal_async(goal)
        self._wait_for_action_future(send_future)

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            raise RuntimeError("RoboKudo rejected the block query.")

        result_future = goal_handle.get_result_async()
        self._wait_for_action_future(result_future)

        return result_future.result().result

    def _wait_for_action_future(self, future: Future) -> None:
        """Complete an action future in the configured execution context."""
        if self.future_completion is None:
            _wait_for_future(future)
            return

        self.future_completion.wait(future)


def query_colored_block_poses_from_robokudo(
    node: Node,
) -> dict[str, tuple[float, float, float]]:
    """Query RoboKudo for detected block positions grouped by color.

    :raises RuntimeError: If the server is unavailable or rejects the query.
    :return: Transformed positions keyed by target block color.
    """
    action_client = ActionClient(node, Query, "/robokudo/query")
    future_completion = FutureCompletion.for_node(node)
    return ColoredBlockPoseQuery(
        node,
        action_client,
        future_completion=future_completion,
    ).execute()


def main() -> None:
    """Query RoboKudo and print the detected block poses."""
    rclpy.init()
    node = rclpy.create_node("robokudo_cram_integration")

    try:
        positions_by_color = query_colored_block_poses_from_robokudo(node)
        print(positions_by_color)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
