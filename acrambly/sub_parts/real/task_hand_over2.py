"""
Task: Right-to-left Handover with STL Meshes (Real)
====================================================
1. Queries RoboKudo for red / yellow / blue blocks.
2. Spawns corresponding STL mesh cube bodies in the Giskard world.
3. Returns a per-cube right-pick -> two-arm handover -> left-place plan.

Callable signature
------------------
``setup_and_build_plan(world, tracy, context, node) -> Plan | None``
"""

from rclpy.node import Node
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.world import World

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.plan import Plan

from sub_parts.real.cube_perception import query_colored_block_poses_from_robokudo
from sub_parts.shared.available_plans import build_hand_over2_plan
from sub_parts.shared.utils import spawn_body


def setup_and_build_plan(
    world: World, tracy: Tracy, context: Context, node: Node
) -> Plan | None:
    """Perceive three blocks and build the right-to-left handover plan."""
    print("[Perception] querying perceived positions...")
    block_poses = query_colored_block_poses_from_robokudo(node)

    red_pos = block_poses["red"]
    yellow_pos = block_poses["yellow"]
    blue_pos = block_poses["blue"]

    print("[Setup] Spawning STL mesh cubes in world...")

    cube0 = spawn_body(
        world, red_pos, (0.0, 0.0, 0.0), "mesh",
        mesh_filename="child_cube_0_scaled.stl",
    )
    cube1 = spawn_body(
        world, yellow_pos, (0.0, 0.0, 0.0), "mesh",
        mesh_filename="child_cube_1_scaled.stl",
    )
    cube2 = spawn_body(
        world, blue_pos, (0.0, 0.0, 0.0), "mesh",
        mesh_filename="child_cube_2_scaled.stl",
    )

    return build_hand_over2_plan(world, tracy, context, cube0, cube1, cube2)
