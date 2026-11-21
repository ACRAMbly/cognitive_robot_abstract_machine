"""
Pose-adaptive autonomous handover task for Tracy simulation.

The three STL bodies are spawned into the simulation. Their current world
poses are then used by build_autonomous_handover_plan to calculate grasp,
handover, and drop poses at runtime.
"""

from rclpy.node import Node

from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.world import World

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.plan import Plan

from sub_parts.shared.available_plans import build_autonomous_handover_plan
from sub_parts.shared.utils import spawn_body


CUBES = {
    "child_cube_0": ("cube0", "cubes/child_cube_0_scaled.stl"),
    "child_cube_1": ("cube1", "cubes/child_cube_1_scaled.stl"),
    "child_cube_2": ("cube2", "cubes/child_cube_2_scaled.stl"),
}

# Simulation spawn locations only. The plan reads each body's live global pose.
SPAWN_POSES = {
    "child_cube_0": (0.5, -0.5, 0.93),
    "child_cube_1": (0.8, -0.6, 0.93),
    "child_cube_2": (1.0, -0.3, 0.93),
}


def setup_and_build_plan(
    world: World, tracy: Tracy, context: Context, node: Node
) -> Plan | None:
    print("[Setup] Spawning STL objects for pose-adaptive handover...")

    bodies = {}
    for classname, (grasp_key, mesh) in CUBES.items():
        position = SPAWN_POSES[classname]
        body = spawn_body(
            world,
            position,
            (0.0, 0.0, 0.0),
            "mesh",
            mesh_filename=mesh,
        )
        bodies[grasp_key] = body
        print(
            f"[Setup] {grasp_key} spawned at "
            f"({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
        )

    return build_autonomous_handover_plan(
        world=world,
        tracy=tracy,
        context=context,
        bodies=bodies,
    )
