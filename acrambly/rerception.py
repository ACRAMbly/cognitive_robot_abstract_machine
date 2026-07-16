"""Query RoboKudo repeatedly for stable colored-block positions."""

from __future__ import annotations

from dataclasses import dataclass, field

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from robokudo_msgs.action import Query


@dataclass(frozen=True)
class ColoredBlockPoseParser:
    """Extract transformed target-color positions from a query result."""

    target_colors: frozenset[str] = frozenset({'red', 'yellow', 'blue'})
    """Block colors collected by the integration."""

    blue_color_labels: frozenset[str] = frozenset({'blue', 'cyan'})
    """RoboKudo labels accepted for the blue block."""

    def parse(
        self,
        result: Query.Result,
    ) -> dict[str, tuple[float, float, float]]:
        """Return transformed target-color positions with detected poses."""
        positions_by_color: dict[str, tuple[float, float, float]] = {}

        for object_designator in result.res:
            if not object_designator.pose:
                continue

            for color in object_designator.color:
                target_color = 'blue' if color in self.blue_color_labels else color
                if target_color not in self.target_colors:
                    continue

                position = object_designator.pose[0].pose.position
                positions_by_color[target_color] = (
                    position.x + 0.11,
                    position.y,
                    0.955,
                )

        return positions_by_color


@dataclass(frozen=True)
class ColoredBlockPoseQuery:
    """Collect target-colored block positions across fresh query attempts."""

    node: Node
    """ROS node used to complete action futures."""

    action_client: ActionClient
    """Client connected to the RoboKudo query action."""

    maximum_attempts: int = 5
    """Maximum fresh-frame queries issued for missing colors."""

    parser: ColoredBlockPoseParser = field(
        default_factory=ColoredBlockPoseParser
    )
    """Parser that selects and transforms target-color poses."""

    def execute(self) -> dict[str, tuple[float, float, float]]:
        """Query until all target colors are found or attempts run out."""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                'RoboKudo query action server is not available.'
            )

        positions_by_color: dict[str, tuple[float, float, float]] = {}
        for attempt_number in range(self.maximum_attempts):
            result = self._request_fresh_frame()
            positions_by_color.update(self.parser.parse(result))
            if self.parser.target_colors.issubset(positions_by_color):
                break

        return positions_by_color

    def _request_fresh_frame(self) -> Query.Result:
        """Send one block query and return its perception result."""
        goal = Query.Goal()
        goal.obj.type = 'block'

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError('RoboKudo rejected the block query.')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        return result_future.result().result


def query_colored_block_poses_from_robokudo(
    node: Node,
) -> dict[str, tuple[float, float, float]]:
    """Query RoboKudo for detected block positions grouped by color.

    :raises RuntimeError: If the server is unavailable or rejects the query.
    :return: Transformed positions keyed by target block color.
    """
    action_client = ActionClient(node, Query, '/robokudo/query')
    return ColoredBlockPoseQuery(node, action_client).execute()


def main() -> None:
    """Query RoboKudo and print the detected block poses."""
    rclpy.init()
    node = rclpy.create_node('robokudo_cram_integration')

    try:
        positions_by_color = query_colored_block_poses_from_robokudo(node)
        print(positions_by_color)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
