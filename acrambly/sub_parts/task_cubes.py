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

import time

from rclpy.node import Node
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.world_entity import Body

from coraplex.datastructures.dataclasses import Context
from coraplex.plans.plan import Plan

from sub_parts.cube_perception import query_colored_block_poses_from_robokudo
from sub_parts.available_plans import build_plan_cubes


def spawn_box(
    spawn_world: World,
    name: str = "box",
    position: tuple = (0.0, 0.0, 1.5),
    scale: float = 0.1,
    r: float = 0.0,
    g: float = 0.0,
    b: float = 0.0,
) -> Body:
    """Spawn a simple URDF box body at the given position (cube-stacking helper)."""
    spawn_body = URDFParser(
        f"""<?xml version="1.0"?>
        <robot name="{name}_box">
          <link name="{name}_link">
            <inertial>
              <mass value="0.1"/>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
            </inertial>
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="{scale} {scale} {scale}"/>
              </geometry>
              <material name="{name}_mat">
                <color rgba="{r} {g} {b} 1.0"/>
              </material>
            </visual>
            <collision>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="{scale} {scale} {scale}"/>
              </geometry>
            </collision>
          </link>
        </robot>
        """
    ).parse()

    with spawn_world.modify_world():
        connection = Connection6DoF.create_with_dofs(
            parent=spawn_world.root,
            child=spawn_body.root,
            world=spawn_world,
        )
        spawn_world.merge_world(spawn_body, connection)

    time.sleep(0.5)
    connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=position[0],
        y=position[1],
        z=position[2],
        reference_frame=spawn_body,
    )

    box = spawn_world.get_kinematic_structure_entity_by_name(f"{name}_link")
    return box


def setup_and_build_plan(world: World, tracy: Tracy, context: Context, node: Node) -> Plan | None:
    """
    Task-specific setup for the cube stacking scenario:
    1. Perceives colored blocks via RoboKudo
    2. Spawns red/green/blue boxes at perceived positions
    3. Builds the stacking plan
    """
    print("Adding boxes to world.")

    block_poses = query_colored_block_poses_from_robokudo(node)
    print(block_poses)

    red_box_pos = block_poses["red"]
    green_box_pos = block_poses["yellow"]
    blue_box_pos = block_poses["blue"]
    SCALE = 0.1

    # ===== FILTERED POSITIONS LOG =====
    print("\n===== Positions used for spawning =====")
    print(
        f"  red    (from 'red')   : x={red_box_pos[0]:.3f}  y={red_box_pos[1]:.3f}  z={red_box_pos[2]:.3f}"
    )
    print(
        f"  green  (from 'yellow'): x={green_box_pos[0]:.3f}  y={green_box_pos[1]:.3f}  z={green_box_pos[2]:.3f}"
    )
    print(
        f"  blue   (from 'blue')  : x={blue_box_pos[0]:.3f}  y={blue_box_pos[1]:.3f}  z={blue_box_pos[2]:.3f}"
    )
    print("=======================================\n")

    red = spawn_box(world, "red", red_box_pos, SCALE, 1.0, 0.0, 0.0)
    green = spawn_box(world, "green", green_box_pos, SCALE, 0.0, 1.0, 0.0)
    blue = spawn_box(world, "blue", blue_box_pos, SCALE, 0.0, 0.0, 1.0)

    return build_plan_cubes(world, tracy, context, red, green, blue)