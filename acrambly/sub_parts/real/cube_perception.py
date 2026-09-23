"""Query RoboKudo for the cube-assembly poses produced by the live pipeline.

Replaces the colour-keyed version.  Three reasons it had to change:

  * Both assemblies are red-and-blue, so colour cannot separate them - the
    perception pipeline (demo_tracy_cubes_live) distinguishes them by metric
    size and returns designators typed 'child_cube_0' / 'child_cube_2'.
  * The old size filter accepted only boxes whose every side was 3-7 cm; the
    assemblies (0.05 x 0.203 x 0.05 and 0.152 x 0.101 x 0.101) fail it, so
    every detection was discarded.
  * The old parser replaced the measured z with a hardcoded 0.95 (and added
    2 cm to y).  The estimator's own height is returned here instead.

Poses come back in the frame RoboKudo reports - 'map' with
POSES_IN_MAP_FRAME = True and the query annotators in the pipeline.
"""

from __future__ import annotations

from dataclasses import dataclass
from threading import Event

import rclpy
from rclpy.action import ActionClient
from typing_extensions import TYPE_CHECKING

try:
    from robokudo_msgs.action import Query
except:
    Query = None

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.task import Future


# Classnames exactly as demo_tracy_cubes_live.OBJECTS reports them.
CUBE_CLASSNAMES: tuple[str, ...] = ("child_cube_0", "child_cube_2")


def _wait_for_future(future: Future) -> None:
    """Wait without attempting to spin the existing ROS executor."""
    completed = Event()
    future.add_done_callback(lambda _: completed.set())
    completed.wait()


@dataclass(frozen=True)
class FutureCompletion:
    """Complete action futures in standalone and executor-owned contexts."""

    node: Node
    spins_node: bool

    @classmethod
    def for_node(cls, node: Node) -> FutureCompletion:
        return cls(node=node, spins_node=node.executor is None)

    def wait(self, future: Future) -> None:
        if self.spins_node:
            rclpy.spin_until_future_complete(self.node, future)
            return
        _wait_for_future(future)


@dataclass(frozen=True)
class CubePoseQuery:
    """Collect cube positions by classname across fresh query attempts."""

    node: Node
    action_client: ActionClient
    classnames: tuple[str, ...] = CUBE_CLASSNAMES
    maximum_attempts: int = 10
    future_completion: FutureCompletion | None = None

    def execute(self) -> dict[str, tuple[float, float, float]]:
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("RoboKudo query action server is not available.")

        found: dict[str, tuple[float, float, float]] = {}
        attempts_remaining = self.maximum_attempts

        # One query returns every detected object (the AE does not filter on
        # goal contents), so we ask once per attempt and select by classname -
        # a query costs ~10 s, so one round trip beats one per object.
        while attempts_remaining > 0 and len(found) < len(self.classnames):
            result = self._request_fresh_frame()

            for designator in result.res:
                name = designator.type
                if name not in self.classnames or name in found:
                    continue
                if not designator.pose:
                    # detected, but FoundationPose produced no pose this frame
                    continue
                position = designator.pose[0].pose.position
                found[name] = (position.x, position.y, position.z)

            attempts_remaining -= 1

        return found

    def _request_fresh_frame(self) -> Query.Result:
        goal = Query.Goal()   # empty: the pipeline does not filter by goal

        send_future = self.action_client.send_goal_async(goal)
        self._wait_for_action_future(send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("RoboKudo rejected the query.")

        result_future = goal_handle.get_result_async()
        self._wait_for_action_future(result_future)

        return result_future.result().result

    def _wait_for_action_future(self, future: Future) -> None:
        if self.future_completion is None:
            _wait_for_future(future)
            return
        self.future_completion.wait(future)


def query_cube_poses_from_robokudo(
    node: Node,
    classnames: tuple[str, ...] = CUBE_CLASSNAMES,
) -> dict[str, tuple[float, float, float]]:
    """Positions keyed by classname, as reported by the estimator.

    :raises RuntimeError: If the server is unavailable or rejects the query.
    """
    action_client = ActionClient(node, Query, "/robokudo/query")
    return CubePoseQuery(
        node,
        action_client,
        classnames=tuple(classnames),
        future_completion=FutureCompletion.for_node(node),
    ).execute()


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("robokudo_cram_integration")
    try:
        print(query_cube_poses_from_robokudo(node))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
