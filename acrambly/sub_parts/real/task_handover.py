from rclpy.node import Node

from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Color
from sub_parts.real.cube_perception import query_colored_block_poses_from_robokudo
from coraplex.datastructures.dataclasses import Context
from coraplex.plans.plan import Plan
from sub_parts.shared.available_plans import build_handover_object_plan
from sub_parts.shared.utils import spawn_cube

def setup_and_build_plan(
    world: World, tracy: Tracy, context: Context, node: Node
) -> Plan | None:
    """
    Task-specific setup for the cube stacking simulation:
    1. Spawns red/green/blue boxes at hardcoded positions (matching demo.py)
    2. Builds the stacking plan from available_plans
    """
    print("[Perception] querying perceived positions...")
    block_poses = query_colored_block_poses_from_robokudo(node)

    red_box_pos = block_poses["red"]
    print(f"  red    (from 'red')   : x={red_box_pos[0]:.3f}  y={red_box_pos[1]:.3f}  z={red_box_pos[2]:.3f}")

    print("[Setup] Spawning boxes in simulation world...")

    obj = spawn_cube(
        world, "box3", red_box_pos, 1.0, color=Color(0.0, 0.0, 1.0, 1.0)
    )

    return build_handover_object_plan(world, tracy, context, obj)