"""
Cube perception (RoboKudo)
Helper functions that talk to the RoboKudo perception service.

Contents
--------
- ``query_colored_block_poses_from_robokudo()`` – repeatedly query until all
  target colors are found; returns a ``dict[color, PoseStamped]``
- ``pose_to_position()`` – extract ``(x, y, z)`` from a ``PoseStamped``
"""

import time

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from robokudo_msgs.action import Query


def query_colored_block_poses_from_robokudo(
        node: Node, target_colors=None, max_attempts: int = 10
) -> dict:
    """
    Query RoboKudo repeatedly until all TARGET_COLORS are found.

    Detection varies frame to frame, so if a color is missing, we re-query and
    accumulate found colors across attempts, keeping the first pose per color.
    """
    if target_colors is None:
        target_colors = {"red", "yellow", "blue"}

    poses_by_color = {}

    for attempt in range(1, max_attempts + 1):
        missing = target_colors - set(poses_by_color)
        if not missing:
            break

        print(f"\n[attempt {attempt}/{max_attempts}] querying; still missing: {sorted(missing)}")

        # Fresh action client each attempt (avoids reusing a client whose previous
        # goal handle may not be fully released).
        action_client = ActionClient(node, Query, "/robokudo/query")
        if not action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("RoboKudo query action server is not available.")

        goal = Query.Goal()
        goal.obj.type = "block"

        send_future = action_client.send_goal_async(goal)
        # bounded wait so we never hang forever if the server wedges
        waited = 0.0
        while not send_future.done():
            time.sleep(0.05)
            waited += 0.05
            if waited > 15.0:
                raise RuntimeError(
                    "Timed out waiting for RoboKudo to accept the goal "
                    "(server may be wedged; restart the perception script)."
                )

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("RoboKudo rejected the block query.")

        result_future = goal_handle.get_result_async()
        waited = 0.0
        while not result_future.done():
            time.sleep(0.05)
            waited += 0.05
            if waited > 30.0:
                raise RuntimeError(
                    "Timed out waiting for RoboKudo result "
                    "(server may be wedged; restart the perception script)."
                )

        result = result_future.result().result

        # ===== RAW QUERY RESULT LOG (test only) =====
        print(f"  raw: {len(result.res)} objects detected this attempt")
        for i, od in enumerate(result.res):
            colors = list(od.color)
            if od.pose:
                p = od.pose[0].pose.position
                print(f"    [{i}] colors={colors}  pos=(x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f})  frame={od.pose[0].header.frame_id}")
            else:
                print(f"    [{i}] colors={colors}  pose=<none>")
        # ============================================

        # Accumulate any target colors we don't already have.
        for object_designator in result.res:
            if not object_designator.pose:
                continue
            for color in object_designator.color:
                if color in target_colors and color not in poses_by_color:
                    poses_by_color[color] = object_designator.pose[0]
                    print(f"  -> found '{color}'")

        # Clean up this attempt's client before the next attempt.
        action_client.destroy()

        # Give the RoboKudo action server time to fully finish/close
        missing_after = target_colors - set(poses_by_color)
        if missing_after:
            print(f"  still missing {sorted(missing_after)}; pausing before next attempt...")
            time.sleep(3.0)

    missing = target_colors - set(poses_by_color)
    if missing:
        raise RuntimeError(
            f"RoboKudo did not detect blocks with colors {sorted(missing)} "
            f"after {max_attempts} attempts."
        )

    return poses_by_color

def pose_to_position(pose_stamped: PoseStamped) -> tuple[float, float, float]:
    p = pose_stamped.pose.position
    return p.x, p.y, p.z
