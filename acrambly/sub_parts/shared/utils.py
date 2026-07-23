from semantic_digital_twin.world import World
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.geometry import Box, Color, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

def spawn_cube(
    spawn_world: World,
    name: str = "box",
    position: tuple = (0.0, 0.0, 1.5),
    yaw: float | int = 0.0,
    scale: Scale = Scale(0.05, 0.05, 0.05),
    color: Color = Color(1.0, 1.0, 0.0, 1.0),
) -> Body:
    """Spawn a free-floating box body via the Semantic Digital Twin API."""
    with spawn_world.modify_world():
        spawn_body = Body(
            name=PrefixedName(name),
            collision=ShapeCollection(shapes=[Box(scale=scale)]),
            visual=ShapeCollection(shapes=[Box(scale=scale, color=color)]),
        )

        spawn_world.add_kinematic_structure_entity(spawn_body)

        spawn_world.add_connection(
            Connection6DoF.create_with_dofs(
                parent=spawn_world.root,
                child=spawn_body,
                world=spawn_world,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=position[0], y=position[1], z=position[2], yaw=yaw
                ),
            )
        )

    return spawn_body
