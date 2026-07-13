import rclpy
from rclpy.action import ActionClient
from robokudo_msgs.action import Query

#def query_colored_block_poses_from_robokudo(node) -> dict[str, PoseStamped]:
action_client = ActionClient(node, Query, "/robokudo/query")

goal = Query.Goal()
goal.obj.type = "block"

send_future = action_client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send_future)

goal_handle = send_future.result()

result_future = goal_handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future)

result = result_future.result().result
poses_by_color: dict[str, PoseStamped] = {}

for object_designator in result.res:
    if not object_designator.pose:
        continue

    for color in object_designator.color:
        if color in {"red", "yellow", "blue"}:
            poses_by_color[color] = object_designator.pose[0]

print(poses_by_color)
#return poses_by_color