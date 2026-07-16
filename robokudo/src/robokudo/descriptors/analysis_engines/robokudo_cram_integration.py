import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robokudo_msgs.action import Query


def query_colored_block_poses_from_robokudo(
    node: Node,
) -> dict[str, tuple[float, float, float]]:
    """Query RoboKudo for detected block positions grouped by color."""
    action_client = ActionClient(node, Query, "/robokudo/query")

    if not action_client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError("RoboKudo query action server is not available.")

    goal = Query.Goal()
    goal.obj.type = "block"

    send_future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)

    goal_handle = send_future.result()
    if not goal_handle.accepted:
        raise RuntimeError("RoboKudo rejected the block query.")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result().result
    positions_by_color: dict[str, tuple[float, float, float]] = {}

    for object_designator in result.res:
        if not object_designator.pose:
            continue

        for color in object_designator.color:
            if color in {"red", "yellow", "blue"}:
                position = object_designator.pose[0].pose.position
                #positions_by_color[color] = (position.x + 0.22, position.y, position.z)
                positions_by_color[color] = (position.x + 0.22, position.y, 0.955)

    return positions_by_color


def main() -> None:
    """Query RoboKudo once and print the detected block poses."""
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
