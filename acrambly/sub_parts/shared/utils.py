import os
from pathlib import Path
from typing import Literal

from coraplex.datastructures.enums import Arms
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.geometry import Box, Color, Mesh, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

_ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../assets")


def select_arm(cube: Body, tracy):
    cube_y = float(cube.global_pose.position.to_np().reshape(-1)[1])
    end_effectors = Tracy.get_end_effectors(tracy)
    if cube_y > 0:
        return Arms.LEFT, end_effectors[0]
    else:
        return Arms.RIGHT, end_effectors[1]


def spawn_body(
    spawn_world: World,
    position: tuple[float, float, float] = (0.0, 0.0, 1.5),
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    shape_type: Literal["mesh", "box"] = "box",
    *,
    mesh_filename: str | None = None,
    name: str = "box",
    scale: Scale = Scale(0.05, 0.05, 0.05),
    color: Color = Color(1.0, 1.0, 0.0, 1.0),
) -> Body:
    """Spawn a free-floating body via the Semantic Digital Twin API.

    Args:
        spawn_world: The world to spawn the body into.
        position: (x, y, z) position of the body.
        rotation: (yaw, pitch, roll) rotation of the body.
        shape_type: ``"box"`` for a coloured box shape, ``"mesh"`` for an STL mesh file.
        mesh_filename: Required when *shape_type* is ``"mesh"``. Name of the STL file
            inside the assets directory.
        name: Required when *shape_type* is ``"box"``. Base name for the body.
        scale: Box dimensions (``"box"`` only).
        color: Box colour (``"box"`` only).

    Returns:
        The spawned :class:`Body`.
    """
    with spawn_world.modify_world():
        if shape_type == "mesh":
            if mesh_filename is None:
                raise ValueError("mesh_filename is required for shape_type='mesh'")
            mesh_path = Path(_ASSETS_DIR, mesh_filename)
            if not mesh_path.exists():
                raise FileNotFoundError(f"Mesh file not found: {mesh_path}")
            mesh_shape = Mesh(origin=HomogeneousTransformationMatrix(), filename=str(mesh_path))
            collision_shapes = ShapeCollection([mesh_shape])
            visual_shapes = collision_shapes
            body_name = PrefixedName(prefix="objects", name=mesh_path.stem)
        else:
            collision_shapes = ShapeCollection(shapes=[Box(scale=scale)])
            visual_shapes = ShapeCollection(shapes=[Box(scale=scale, color=color)])
            body_name = PrefixedName(name)

        body = Body(
            name=body_name,
            collision=collision_shapes,
            visual=visual_shapes,
        )
        spawn_world.add_kinematic_structure_entity(body)
        spawn_world.add_connection(
            Connection6DoF.create_with_dofs(
                parent=spawn_world.root,
                child=body,
                world=spawn_world,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=position[0], y=position[1], z=position[2],
                    yaw=rotation[0], pitch=rotation[1], roll=rotation[2],
                ),
            )
        )
    return body
