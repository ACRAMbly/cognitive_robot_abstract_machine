"""
Tracy Real-Robot Task Runner
Single entry-point for all Tracy tasks on the real robot.

Architecture
------------
::

    tracy_stacking_real.py          ← orchestrator (this file)
    ├── sub_parts/
    │   ├── cube_perception.py      ← RoboKudo perception helpers
    │   ├── available_plans.py      ← plan factories (pure functions)
    │   ├── task_cubes.py           ← task: cube stacking (incl. spawn_box)
    │   ├── task_park_arms.py       ← task: park both arms
    │   └── task_<new>.py           ← add your own task here

Adding a new task
-----------------
1. Create ``sub_parts/task_yourname.py`` with a single public function::

       def setup_and_build_plan(world, tracy, context, node) -> Plan | None:
           # 1. perception – import from sub_parts (or inline)
           # 2. spawn objects – define your own helper, e.g. spawn_airplane_part()
           # 3. return a plan from available_plans (or build inline)

2. If you need a new plan shape, add it to ``available_plans.py``.

3. Import your task here and register it in ``TASKS``::

       from sub_parts.task_yourname import setup_and_build_plan as yourname_task
       TASKS["yourname"] = yourname_task

4. Update the ``Literal`` type hint of ``main(…)`` to include ``"yourname"``.

5. Run: ``python tracy_stacking_real.py --task yourname``

That's it – the orchestrator handles ROS init, world fetch, context creation,
and plan execution for you.
"""

import threading
from typing import Callable, Literal

import rclpy
import typer
from coraplex.alternative_motion_mappings.tracy_motion_mapping import (
    TracyRealMoveGripperMotion,
)
from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import real_robot
from coraplex.plans.plan import Plan
from rclpy.node import Node
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.world import World
from sub_parts.task_cubes import setup_and_build_plan as cubes_task
from sub_parts.task_park_arms import setup_and_build_plan as park_arms_task

#### IMPORTANT: RESTART THE GISKARD SCRIPT EACH TIME YOU RUN THIS SCRIPT
import subprocess
import time
# giskard_process = subprocess.Popen(
#     [
#             "gnome-terminal", "--", "bash", "-c",
#             "workon cram-env && ",
#             "ros2 launch giskardpy_ros giskardpy_tracy_velocity.launch.py; exec bash"
#     ],
# )
# print("Initializing GISKARD...")
# time.sleep(10)


# ---------------------------------------------------------------------------
# Task registry – add new tasks here
# Each entry maps a task name to a callable with signature:
#     (World, Tracy, Context, Node) -> Plan | None
# ---------------------------------------------------------------------------
TaskFactory = Callable[[object, object, Context, Node], Plan | None]

TASKS: dict[str, TaskFactory] = {
    "park_arms": park_arms_task,
    "cubes": cubes_task,
}

# --------------------------------------------------------------------------
# World fetch and synchronization
# --------------------------------------------------------------------------
def fetch_and_sync_world(node: Node, timeout_seconds: float = 300) -> World:
    """Fetch the live world from Giskard and set up synchronization."""
    print("Getting live world from Giskard...")
    tracy_world = fetch_world_from_service(node, timeout_seconds=timeout_seconds)
    print(f"World received with {len(list(tracy_world.bodies))} bodies.")
    WorldSynchronizer(_world=tracy_world, node=node, synchronous=True)
    print("Synchronized.")
    return tracy_world


# ---------------------------------------------------------------------------
# Main orchestrator
# ---------------------------------------------------------------------------
def main(task: Literal["park_arms", "cubes"]):
    rclpy.init()

    node: Node = rclpy.create_node("coraplex_real_stacking")

    # Giskard action clients need the node to spin.
    spinner = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spinner.start()

    # ---- always the same ----
    world = fetch_and_sync_world(node)

    print("Building Tracy semantic robot from giskard world...")
    tracy = world.get_semantic_annotations_by_type(AbstractRobot)[0]

    context = Context(
        world=world,
        robot=tracy,
        ros_node=node,
        alternative_motion_mappings=[TracyRealMoveGripperMotion],
        evaluate_conditions=False,
    )

    # ---- task-specific ----
    plan = TASKS[task](world, tracy, context, node)

    # ---- always the same ----
    print("Executing Plan on REAL robot through Giskard...")
    print("Keep E-stop reachable.")

    try:
        with real_robot:
            plan.perform()
            print("Plan completed.")
    except Exception as e:
        print(f"Error during plan execution: {e}")


if __name__ == "__main__":
    typer.run(main)