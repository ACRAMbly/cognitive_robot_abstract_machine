"""
Task: Cube Stacking
===================
1. Queries RoboKudo for red / yellow / blue blocks.
2. Spawns corresponding box bodies in the Giskard world.
3. Returns a 3-step pick-and-place stacking plan.

This is the reference implementation for a "full" task module that combines
perception, world-setup, and plan-building.  It also contains ``spawn_box()``
because the URDF for a simple box is specific to this task – other tasks
(e.g. airplane) will define their own spawning helpers.

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
from sub_parts.shared.utils import spawn_cube

def setup_and_build_plan(world: World, tracy: Tracy, context: Context, node: Node) -> Plan | None:
    """
    Task-specific setup for the cube stacking scenario:
    1. Perceives colored blocks via RoboKudo
    2. Spawns red/green/blue boxes at perceived positions
    3. Builds the stacking plan
    """

    print("[Perception] querying perceived positions...")
    block_poses = query_colored_block_poses_from_robokudo(node)

    red_box_pos = block_poses["red"]
    green_box_pos = block_poses["yellow"]
    blue_box_pos = block_poses["blue"]
    scale = 0.1

    # ===== FILTERED POSITIONS LOG =====
    print("\n===== Positions used for spawning =====")
    print(f"  red    (from 'red')   : x={red_box_pos[0]:.3f}  y={red_box_pos[1]:.3f}  z={red_box_pos[2]:.3f}")
    print(f"  green  (from 'yellow'): x={green_box_pos[0]:.3f}  y={green_box_pos[1]:.3f}  z={green_box_pos[2]:.3f}")
    print(f"  blue   (from 'blue')  : x={blue_box_pos[0]:.3f}  y={blue_box_pos[1]:.3f}  z={blue_box_pos[2]:.3f}")
    print("=======================================\n")

    print("[Perception] Adding cubes to world")
    red = spawn_cube(world, "red", red_box_pos, 0, scale, 1.0, 0.0, 0.0)
    green = spawn_cube(world, "green", green_box_pos, 0, scale, 0.0, 1.0, 0.0)
    blue = spawn_cube(world, "blue", blue_box_pos, 0, scale, 0.0, 0.0, 1.0)

    return build_plan_cubes(world, tracy, context, red, green, blue)