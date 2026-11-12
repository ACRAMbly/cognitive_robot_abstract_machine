"""
Screwdriver tool representation and spawner.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Point3,
    Pose,
    RotationMatrix,
    Vector3,
)
from semantic_digital_twin.datastructures.alignment import AlignmentPair
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.semantic_annotations import Tool
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    Connection6DoF,
    FixedConnection,
)
from semantic_digital_twin.world_description.geometry import Color, Cylinder
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% Screwdriver tool


@dataclass(eq=False)
class Screwdriver(Tool):
    """
    A handheld screwdriver tool for driving threaded fasteners.
    """

    grasp_height: float = 0.025
    """
    Height along the tool's local Z axis where the gripper grasps the handle.
    """

    def tool_alignment(self, target: Body | Pose) -> list[AlignmentPair]:
        tip_body = self.get_tool_frame()
        return [
            AlignmentPair(
                tip_normal=Vector3(0, 0, 1, reference_frame=tip_body),
                goal_normal=Vector3(
                    0,
                    0,
                    1,
                    reference_frame=(
                        target if isinstance(target, Body) else target.reference_frame
                    ),
                ),
            )
        ]

    def default_grasp_candidates(self) -> list[GraspCandidate]:
        """
        The grasp from above on the handle along local -Z.
        """
        return [
            GraspCandidate(
                self,
                Pose(
                    position=Point3(
                        0.0, 0.0, self.grasp_height, reference_frame=self.root
                    ),
                    orientation=RotationMatrix.from_rpy(pitch=np.pi / 2).quaternion,
                    reference_frame=self.root,
                ),
            )
        ]


# %% Spawning


def spawn_screwdriver(
    world: World,
    position: tuple[float, float, float] = (0.55, -0.30, 1.0),
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    *,
    parent: Body | None = None,
    name: str = "screwdriver",
    length: float = 0.10,
    diameter: float = 0.03,
    color: Color | None = None,
) -> Screwdriver:
    """
    Spawn a hypothetical screwdriver tool with a handle cylinder and a tip frame.

    :param world: The world to spawn the screwdriver into.
    :param position: (x, y, z) position in world coordinates.
    :param rotation: (yaw, pitch, roll) orientation in radians.
    :param parent: Optional parent body to attach the tool to; defaults to world root.
    :param name: Base name of the screwdriver body.
    :param length: Total length of the screwdriver along local Z, in metres.
    :param diameter: Diameter of the handle cylinder, in metres.
    :param color: Visual colour of the screwdriver handle.
    :return: The screwdriver tool annotation.
    """
    parent_body = world.root if parent is None else parent
    with world.modify_world():
        cylinder = Cylinder(
            origin=HomogeneousTransformationMatrix(),
            width=diameter,
            height=length,
            color=color or Color.from_rgba(red=0.2, green=0.5, blue=0.8, opacity=1.0),
        )
        handle_body = Body(
            name=PrefixedName(prefix="objects", name=name),
            collision=ShapeCollection(shapes=[cylinder]),
            visual=ShapeCollection(shapes=[cylinder]),
        )
        world.add_kinematic_structure_entity(handle_body)

        world.add_connection(
            Connection6DoF.create_with_dofs(
                world=world,
                parent=parent_body,
                child=handle_body,
                name=PrefixedName(prefix="connections", name=f"{name}_connection"),
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=position[0],
                    y=position[1],
                    z=position[2],
                    yaw=rotation[0],
                    pitch=rotation[1],
                    roll=rotation[2],
                ),
            )
        )

        tip_body = Body(
            name=PrefixedName(prefix="objects", name=f"{name}_end_effector")
        )
        world.add_kinematic_structure_entity(tip_body)
        world.add_connection(
            FixedConnection(
                parent=handle_body,
                child=tip_body,
                name=PrefixedName(prefix="connections", name=f"{name}_tip_connection"),
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=0.0,
                    y=0.0,
                    z=-length / 2.0,
                    pitch=math.pi,
                    reference_frame=handle_body,
                ),
            )
        )

        screwdriver = Screwdriver(root=handle_body, grasp_height=length / 4.0)
        world.add_semantic_annotations([screwdriver])

    return screwdriver
