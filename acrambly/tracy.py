"""
Tracy Unified Task Runner.

Tasks are imported lazily so an unused legacy task cannot prevent
another task from running.
"""

import logging
import threading

from typing import Annotated, Callable, Literal

import rclpy
import typer

from rclpy.node import Node

from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import (
    real_robot,
    kinematically_simulated_robot,
)
from coraplex.plans.plan import Plan

from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.ros.world_fetcher import (
    fetch_world_from_service,
)
from semantic_digital_twin.adapters.ros.world_synchronizer import (
    WorldSynchronizer,
)
from semantic_digital_twin.adapters.urdf import URDFParser

from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.world import World


logging.getLogger(
    "coraplex.plans.executables"
).setLevel(logging.INFO)


TaskFactory = Callable[
    [object, object, Context, Node],
    Plan | None,
]


# ---------------------------------------------------------------------
# Environment setup
# ---------------------------------------------------------------------

def setup_real(node: Node):
    """Fetch the live robot world and create its execution context."""

    # Import real-robot mappings only when the real environment is used.
    from coraplex.alternative_motion_mappings.tracy_motion_mapping import (
        TracyRealMoveGripperMotion,
    )

    print("Getting live world from Giskard...")

    world = fetch_world_from_service(
        node,
        timeout_seconds=300,
    )

    print(
        f"World received with {len(list(world.bodies))} bodies."
    )

    WorldSynchronizer(
        _world=world,
        node=node,
    )

    print("Synchronized.")
    print("Building Tracy semantic robot...")

    tracy = world.get_semantic_annotations_by_type(
        AbstractRobot
    )[0]

    context = Context(
        world=world,
        robot=tracy,
        ros_node=node,
        alternative_motion_mappings=[
            TracyRealMoveGripperMotion
        ],
        evaluate_conditions=False,
    )

    return world, tracy, context


def setup_sim(node: Node):
    """Build a fresh Tracy simulation world from its URDF."""

    print("Building simulation world from Tracy URDF...")

    world: World = URDFParser.from_file(
        Tracy.get_ros_file_path()
    ).parse()

    VizMarkerPublisher(
        _world=world,
        node=node,
    )

    print("Building Tracy semantic robot...")

    tracy = Tracy.from_world(world)

    context = Context(
        world=world,
        robot=tracy,
        evaluate_conditions=False,
    )

    return world, tracy, context


# ---------------------------------------------------------------------
# Lazy task factory loader
# ---------------------------------------------------------------------

def get_task_factory(
    task: str,
    env: str,
) -> TaskFactory | None:
    """
    Import only the requested task.

    This prevents unrelated legacy task modules from breaking startup.
    """

    if task == "hand_over2" and env == "sim":
        from sub_parts.sim.task_hand_over2 import (
            setup_and_build_plan,
        )

        return setup_and_build_plan

    if task == "hand_over2" and env == "real":
        from sub_parts.real.task_hand_over2 import (
            setup_and_build_plan,
        )

        return setup_and_build_plan

    if task == "park_arms":
        from sub_parts.shared.task_park_arms import (
            setup_and_build_plan,
        )

        return setup_and_build_plan

    if task == "cubes" and env == "sim":
        from sub_parts.sim.task_cubes import (
            setup_and_build_plan,
        )

        return setup_and_build_plan

    if task == "cubes" and env == "real":
        from sub_parts.real.task_cubes import (
            setup_and_build_plan,
        )

        return setup_and_build_plan

    if task == "task_tetris" and env == "sim":
        from sub_parts.sim.task_tetris import (
            setup_and_build_plan,
        )

        return setup_and_build_plan

    return None


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------

def main(
    env: Annotated[
        Literal["real", "sim"],
        typer.Option("--env", "-e"),
    ],
    task: Annotated[
        Literal[
            "park_arms",
            "cubes",
            "hand_over2",
            "task_tetris",
        ],
        typer.Option("--task", "-t"),
    ],
):

    rclpy.init()

    node: Node = rclpy.create_node(
        "coraplex_task_runner"
    )

    spinner = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )

    spinner.start()

    try:
        # ---------------------------------------------------------
        # Environment setup
        # ---------------------------------------------------------

        if env == "real":
            world, tracy, context = setup_real(node)
            execution_env = real_robot
            env_label = "REAL"

        else:
            world, tracy, context = setup_sim(node)
            execution_env = kinematically_simulated_robot
            env_label = "KINEMATICALLY SIMULATED"

        # ---------------------------------------------------------
        # Load only the requested task
        # ---------------------------------------------------------

        print(
            f"Loading task '{task}' "
            f"for environment '{env}'..."
        )

        task_factory = get_task_factory(
            task,
            env,
        )

        if task_factory is None:
            print(
                f"Task '{task}' is not implemented "
                f"for environment '{env}'."
            )
            return

        # ---------------------------------------------------------
        # Build plan
        # ---------------------------------------------------------

        print(
            f"Building plan for '{task}'..."
        )

        plan = task_factory(
            world,
            tracy,
            context,
            node,
        )

        if plan is None:
            print(
                "No valid plan could be generated."
            )
            return

        # ---------------------------------------------------------
        # Execute
        # ---------------------------------------------------------

        print(
            f"Executing plan on {env_label} robot..."
        )

        if env == "real":
            print(
                "Keep the emergency stop reachable."
            )

        with execution_env:
            plan.perform()

        print("Plan completed successfully.")

        # Remove unnamed temporary bodies.
        for body in list(world.bodies):
            if "None" in str(body.name):
                body.remove_from_world()

    except KeyboardInterrupt:
        print("\nExecution interrupted.")

    except Exception:
        logging.exception(
            "Task execution failed."
        )
        raise

    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    typer.run(main)