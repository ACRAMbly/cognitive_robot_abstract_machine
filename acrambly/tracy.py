"""
Tracy Unified Task Runner
==========================
Single entry-point for all Tracy tasks on both real and simulated robots.

::

    tracy.py                        ← unified orchestrator (this file)
    ├── sub_parts/
    │   ├── real/                   ← real-robot tasks (RoboKudo, Giskard)
    │   │   ├── task_cubes.py
    │   │   └── cube_perception.py
    │   ├── sim/                    ← simulation tasks (hardcoded, RViz2)
    │   │   └── task_cubes.py
    │   └── shared/                ← environment-agnostic tasks & plans
    │       ├── available_plans.py
    │       └── task_park_arms.py

Usage
-----
    python tracy.py --env sim  --task cubes       # RViz2 simulation
    python tracy.py --env real --task cubes       # Real robot through Giskard
    python tracy.py --env real --task park_arms
    python tracy.py --env sim --task hand_over
"""

import logging
import threading
from typing import Annotated, Callable, Literal

import rclpy
import typer

# --- real-robot imports ---
from coraplex.alternative_motion_mappings.tracy_motion_mapping import (
    TracyRealMoveGripperMotion,
)
from coraplex.datastructures.dataclasses import Context  # noqa: E402
from coraplex.execution_environment import real_robot, simulated_robot
from coraplex.plans.plan import Plan
from rclpy.node import Node

# --- simulation imports ---
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.world import World

# --- task imports ---
from sub_parts.real.task_cubes import setup_and_build_plan as cubes_real_task
from sub_parts.shared.task_park_arms import setup_and_build_plan as park_arms_task
from sub_parts.sim.task_cubes import setup_and_build_plan as cubes_sim_task
from sub_parts.sim.task_handover import setup_and_build_plan as hand_over_task

# Enable runtime action logging (fires only during actual robot execution)
logging.getLogger("coraplex.plans.executables").setLevel(logging.INFO)

# ---------------------------------------------------------------------------
# Task registry – maps (task_name, env) → factory
# ---------------------------------------------------------------------------
TaskFactory = Callable[[object, object, Context, Node], Plan | None]

TASKS: dict[tuple[str, str], TaskFactory] = {
    ("cubes", "real"): cubes_real_task,
    ("cubes", "sim"): cubes_sim_task,
    ("park_arms", "real"): park_arms_task,
    ("park_arms", "sim"): park_arms_task,
    ("hand_over", "sim"): hand_over_task,
    ("hand_over", "real"): hand_over_task,
}


# ---------------------------------------------------------------------------
# Environment setup functions
# ---------------------------------------------------------------------------
def setup_real(node: Node) -> tuple[World, object, Context]:
    """Fetch the live world from Giskard and create a real-robot context."""
    print("Getting live world from Giskard...")
    world = fetch_world_from_service(node, timeout_seconds=300)
    print(f"World received with {len(list(world.bodies))} bodies.")
    WorldSynchronizer(_world=world, node=node, synchronous=True)
    print("Synchronized.")

    print("Building Tracy semantic robot from giskard world...")
    tracy = world.get_semantic_annotations_by_type(AbstractRobot)[0]

    context = Context(
        world=world,
        robot=tracy,
        ros_node=node,
        alternative_motion_mappings=[TracyRealMoveGripperMotion],
        evaluate_conditions=False,
    )
    return world, tracy, context


def setup_sim(node: Node) -> tuple[World, object, Context]:
    """Build a fresh Tracy world from URDF and create a simulation context."""
    print("Building simulation world from Tracy URDF...")
    world: World = URDFParser.from_file(Tracy.get_ros_file_path()).parse()

    VizMarkerPublisher(_world=world, node=node).with_tf_publisher()

    print("Building Tracy semantic robot from simulation world...")
    tracy = Tracy.from_world(world)

    context = Context(
        world=world,
        robot=tracy,
        evaluate_conditions=False,
    )
    return world, tracy, context


# ---------------------------------------------------------------------------
# Main orchestrator
# ---------------------------------------------------------------------------
def main(
    env: Annotated[Literal["real", "sim"], typer.Option("--env", "-e")],
    task: Annotated[Literal["park_arms", "cubes", "hand_over"], typer.Option("--task", "-t")],
):
    rclpy.init()

    node: Node = rclpy.create_node("coraplex_task_runner")

    spinner = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spinner.start()

    # ---- env-specific setup ----
    world, tracy, context = setup_real(node) if env == "real" else setup_sim(node)

    # ---- task-specific ----
    task_key = (task, env)
    print(f"Executing task '{task}' on environment '{env}'.")

    if task_key not in TASKS:
        print(f"Unknown task '{task}' for environment '{env}'.")
        print(f"Available: {list(TASKS.keys())}")
        return

    plan = TASKS[task_key](world, tracy, context, node)

    if plan is None:
        print("No valid plan could be generated. Exiting.")
        return

    # ---- execution ----
    execution_env = real_robot if env == "real" else simulated_robot
    env_label = "REAL" if env == "real" else "SIMULATED"

    print(f"Executing Plan on {env_label} robot...")

    if env == "real":
        print("Keep E-stop reachable.")

    try:
        with execution_env:
            plan.perform()
            print("Plan completed.")
    except Exception as e:
        print(f"Error during plan execution: {e}")


if __name__ == "__main__":
    typer.run(main)