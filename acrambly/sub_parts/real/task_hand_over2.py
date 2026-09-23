"""
Task: Right-to-left Handover with STL Meshes (Real)
====================================================
1. Queries RoboKudo for the cube assemblies BY CLASSNAME.
2. Spawns the corresponding STL mesh bodies in the Giskard world.
3. Returns a per-cube right-pick -> two-arm handover -> left-place plan.

Only the two assemblies the perception pipeline knows about are used:
'child_cube_0' and 'child_cube_2' (see demo_tracy_cubes_live.OBJECTS).  Their
names line up with the 'cube0' / 'cube2' grasp descriptions in
available_plans.build_hand_over2_plan, so the tuned offsets apply unchanged.

Callable signature
------------------
``setup_and_build_plan(world, tracy, context, node) -> Plan | None``
"""

from rclpy.node import Node
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.world import World

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.plan import Plan

from sub_parts.real.cube_perception import query_cube_poses_from_robokudo
from sub_parts.shared.available_plans import build_hand_over2_plan
from sub_parts.shared.utils import spawn_body


# perception classname -> (grasp key in available_plans, mesh file)
CUBES = {
    "child_cube_0": ("cube0", "child_cube_0_scaled.stl"),
    "child_cube_2": ("cube2", "child_cube_2_scaled.stl"),
}


def setup_and_build_plan(
    world: World, tracy: Tracy, context: Context, node: Node
) -> Plan | None:
    """Perceive the cube assemblies and build the right-to-left handover plan."""
    print("[Perception] querying perceived positions...")
    poses = query_cube_poses_from_robokudo(node, classnames=tuple(CUBES))

    missing = [name for name in CUBES if name not in poses]
    if missing:
        raise RuntimeError(
            f"perception did not report: {missing} - is the pipeline running, "
            "and are both assemblies visible and within MAX_OBJECT_DISTANCE?")

    print("[Setup] Spawning STL mesh cubes in world...")
    bodies = {}
    for classname, (grasp_key, mesh) in CUBES.items():
        position = poses[classname]
        print(f"  {classname:<14} -> {grasp_key}  at "
              f"({position[0]:+.3f}, {position[1]:+.3f}, {position[2]:+.3f})")
        # Orientation is deliberately left at zero: the grasp offsets in
        # available_plans were tuned against axis-aligned spawns, and the
        # assemblies are symmetric enough that FoundationPose's orientation may
        # be correct only up to a symmetry.  Pass the estimated quaternion here
        # only once the planners have been re-tuned for it.
        bodies[grasp_key] = spawn_body(
            world, position, (0.0, 0.0, 0.0), "mesh", mesh_filename=mesh)

    return build_hand_over2_plan(world, tracy, context, bodies)
