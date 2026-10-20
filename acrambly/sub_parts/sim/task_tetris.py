"""
Task: Cube Stacking with STL Meshes (Simulation)
=================================================
1. Spawns red / yellow / blue boxes from STL mesh files at hardcoded positions.
2. Returns a 3-step pick-and-place stacking plan using ``ApproachDirection.RIGHT``.

Callable signature
------------------
``setup_and_build_plan(world, tracy, context, node) -> Plan | None``
"""

from rclpy.node import Node
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction

from sub_parts.shared.utils import spawn_body, select_arm
from sub_parts.shared.available_plans import build_plan_cubes



def setup_and_build_plan(
    world: World, tracy: Tracy, context: Context, node: Node
) -> Plan | None:
    """
    Task-specific setup for the STL-mesh cube stacking simulation:
    1. Spawns red/yellow/blue STL mesh cubes at hardcoded positions
    2. Builds the stacking plan with ``ApproachDirection.RIGHT``
    """

    print("[Setup] Spawning STL mesh cubes in simulation world...")

    red_box: Body = spawn_body(
        world, (0.8, -0.5, 0.93), (0.5, 0, 0), "mesh",
        mesh_filename="child_cube_0_scaled.stl",
    )
    yellow_box: Body = spawn_body(
        world, (0.8, 0, 0.93), (-0.8, 0, 0), "mesh",
        mesh_filename="child_cube_1_scaled.stl",
    )
    blue_box: Body = spawn_body(
        world, (0.8, 0.5, 0.93), (1.5, 0, 0), "mesh",
        mesh_filename="child_cube_2_scaled.stl",
    )

    return build_plan_cubes(world, tracy, context, red_box, yellow_box, blue_box)
