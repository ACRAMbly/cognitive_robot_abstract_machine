import sys
import threading

import rclpy
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import Arms
from coraplex.execution_environment import real_robot
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction

from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer

def main():
    rclpy.init()

    node = rclpy.create_node("coraplex_real_park_arms")

    # Giskard action/service clients need the node to spin.
    spinner = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spinner.start()

    print("Getting live world from Giskard...")
    world = fetch_world_from_service(node, timeout_seconds=300)
    world_synchronizer = WorldSynchronizer(_world=world, node=node, synchronous=True)
    if world is None:
        print("GiskardWrapper.world is None.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print(f"World received with {len(list(world.bodies))} bodies.")
    print("Building Tracy semantic robot from Giskard world...")
    try:
        tracy = world.get_semantic_annotations_by_type(AbstractRobot)[0]
    except Exception as e:
        print(f"Could not build Tracy from Giskard world: {e}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    context = Context(world=world, robot=tracy, ros_node=node)
    context.evaluate_conditions = False

    plan = sequential(
        [
            ParkArmsAction(Arms.BOTH),
        ],
        context=context,
    ).plan

    print("Executing ParkArmsAction on REAL robot through Giskard...")
    print("Keep E-stop reachable.")

    with real_robot:
        plan.perform()

    node.destroy_node()
    rclpy.shutdown()
    print("ParkArmsAction completed.")


if __name__ == "__main__":
    main()