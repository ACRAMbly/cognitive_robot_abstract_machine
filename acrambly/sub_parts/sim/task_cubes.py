"""
Task: Cube Stacking (Simulation)
=================================
1. Uses hardcoded block positions (no RoboKudo).
2. Spawns boxes via ``spawn_free_box`` (like the original demo.py).
3. Returns a 3-step pick-and-place stacking plan from ``available_plans``.

Callable signature
------------------
``setup_and_build_plan(world, tracy, context, node) -> Plan | None``
"""

from rclpy.node import Node
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.geometry import Box, Color, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.plan import Plan

from sub_parts.shared.available_plans import build_plan_cubes
from sub_parts.shared.utils import spawn_cube

def setup_and_build_plan(
    world: World, tracy: Tracy, context: Context, node: Node
) -> Plan | None:
    """
    Task-specific setup for the cube stacking simulation:
    1. Spawns red/green/blue boxes at hardcoded positions (matching demo.py)
    2. Builds the stacking plan from available_plans
    """

    print("[Setup] Spawning boxes in simulation world...")

    red = spawn_cube(
        world, "box1", (0.8, 0.5, 0.93), 0, color=Color(1.0, 0.0, 0.0, 1.0)
    )
    green = spawn_cube(
        world, "box2", (0.8, -0.5, 0.93), 0, color=Color(0.0, 1.0, 0.0, 1.0)
    )
    blue = spawn_cube(
        world, "box3", (0.8, 0, 0.93), 0, color=Color(0.0, 0.0, 1.0, 1.0)
    )

    return build_plan_cubes(world, tracy, context, red, green, blue)