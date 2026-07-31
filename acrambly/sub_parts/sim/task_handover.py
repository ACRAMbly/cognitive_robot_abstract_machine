from rclpy.node import Node

from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Color

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.plan import Plan
from sub_parts.shared.available_plans import build_handover_object_plan
from sub_parts.shared.utils import spawn_body

def setup_and_build_plan(
    world: World, tracy: Tracy, context: Context, node: Node
) -> Plan | None:
    """
    Task-specific setup for the cube stacking simulation:
    1. Spawns red/green/blue boxes at hardcoded positions (matching demo.py)
    2. Builds the stacking plan from available_plans
    """

    print("[Setup] Spawning boxes in simulation world...")

    obj = spawn_body(
        world, (0.8, 0.5, 0.93), (1.0, 0.0, 0.0), "box",
        name="box3", color=Color(0.0, 0.0, 1.0, 1.0),
    )

    return build_handover_object_plan(world, tracy, context, obj)