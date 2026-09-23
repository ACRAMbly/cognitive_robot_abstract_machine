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

CUBES = {
    "child_cube_0": ("cube0", "child_cube_0_scaled.stl"),
    "child_cube_1": ("cube1", "child_cube_1_scaled.stl"),
    "child_cube_2": ("cube2", "child_cube_2_scaled.stl"),
}

def setup_and_build_plan(
    world: World, tracy: Tracy, context: Context, node: Node
) -> Plan | None:
    """Spawn three mesh cubes and build the right-to-left handover plan."""
    print("[Setup] Spawning STL mesh cubes in simulation world...")
    poses = {
        "child_cube_0": (0.5, -0.5, 0.93),
        "child_cube_1": (0.8, -0.6, 0.93),
        "child_cube_2": (1.0, -0.3, 0.93),
    }

    bodies = {}
    for classname, (grasp_key, mesh) in CUBES.items():
        position = poses[classname]
        print(
            f"  {classname:<14} -> {grasp_key}  at "
            f"({position[0]:+.3f}, {position[1]:+.3f}, {position[2]:+.3f})"
        )
        bodies[grasp_key] = spawn_body(
            world, position, (0.0, 0.0, 0.0), "mesh", mesh_filename=mesh
        )

    return build_hand_over2_plan(world, tracy, context, bodies)
