"""
Task: Cube Stacking with STL Meshes (Real)
=================================================
1. Queries RoboKudo for red / yellow / blue blocks.
2. Spawns corresponding STL mesh cube bodies in the Giskard world.
3. Returns a 3-step pick-and-place stacking plan.

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
from sub_parts.shared.available_plans import build_plan_cubes
from sub_parts.shared.utils import spawn_body


def setup_and_build_plan(
    world: World, tracy: Tracy, context: Context, node: Node
) -> Plan | None:
    """
    Task-specific setup for the STL-mesh cube stacking scenario:
    1. Perceives colored blocks via RoboKudo
    2. Spawns red/yellow/blue STL mesh cubes at perceived positions
    3. Builds the stacking plan
    """

    print("[Perception] querying perceived positions...")
    block_poses = query_colored_block_poses_from_robokudo(node)

    red_pos = block_poses["red"]
    yellow_pos = block_poses["yellow"]
    blue_pos = block_poses["blue"]

    # ===== FILTERED POSITIONS LOG =====
    print("\n===== Positions used for spawning =====")
    print(f"  red    : x={red_pos[0]:.3f}  y={red_pos[1]:.3f}  z={red_pos[2]:.3f}")
    print(f"  yellow : x={yellow_pos[0]:.3f}  y={yellow_pos[1]:.3f}  z={yellow_pos[2]:.3f}")
    print(f"  blue   : x={blue_pos[0]:.3f}  y={blue_pos[1]:.3f}  z={blue_pos[2]:.3f}")
    print("=======================================\n")

    print("[Setup] Spawning STL mesh cubes in world...")

    red_box = spawn_body(
        world, red_pos, (0.0, 0.0, 0.0), "mesh",
        mesh_filename="child_cube_0_scaled.stl",
    )
    yellow_box = spawn_body(
        world, yellow_pos, (0.0, 0.0, 0.0), "mesh",
        mesh_filename="child_cube_1_scaled.stl",
    )
    blue_box = spawn_body(
        world, blue_pos, (0.0, 0.0, 0.0), "mesh",
        mesh_filename="child_cube_2_scaled.stl",
    )

    return build_plan_cubes(world, tracy, context, red_box, yellow_box, blue_box)
