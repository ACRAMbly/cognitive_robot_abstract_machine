"""
Task: Right-to-left Handover with STL Meshes (Simulation)
==========================================================
1. Spawns three STL mesh cubes at hardcoded positions (matching task2.py).
2. Returns a per-cube right-pick -> two-arm handover -> left-place plan.

Callable signature
------------------
``setup_and_build_plan(world, tracy, context, node) -> Plan | None``
"""

from rclpy.node import Node
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.plan import Plan

from sub_parts.shared.available_plans import build_hand_over2_plan
from sub_parts.shared.utils import spawn_body


def setup_and_build_plan(
    world: World, tracy: Tracy, context: Context, node: Node
) -> Plan | None:
    """Spawn three mesh cubes and build the right-to-left handover plan."""
    print("[Setup] Spawning STL mesh cubes in simulation world...")

    cube0: Body = spawn_body(
        world, (0.5, -0.5, 0.93), (0.5, 0, 0), "mesh",
        mesh_filename="child_cube_0_scaled.stl",
    )
    cube1: Body = spawn_body(
        world, (0.8, -0.6, 0.93), (0.5, 0, 0), "mesh",
        mesh_filename="child_cube_1_scaled.stl",
    )
    cube2: Body = spawn_body(
        world, (1.0, -0.3, 0.93), (0.5, 0, 0), "mesh",
        mesh_filename="child_cube_2_scaled.stl",
    )

    return build_hand_over2_plan(world, tracy, context, cube0, cube1, cube2)
