"""
Stretch fetches a cereal box from a shelf and places it on a bedside table.

Running with :attr:`~coraplex.datastructures.enums.ExecutionType.REAL` drives the actual
robot and fetches the world from the running world server. The default runs the whole plan
in simulation against a world built from the Stretch URDF, so nothing on the network is
needed.
"""

from dataclasses import dataclass

import numpy as np
from typing_extensions import ClassVar

from coraplex.alternative_motion_mappings.stretch_motion_mapping import (
    StretchClose,
    StretchMoveReal,
    StretchMoveSim,
    StretchMoveToolCenterPoint,
)
from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import (
    ApproachDirection,
    Arms,
    ExecutionType,
    VerticalAlignment,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.plans.factories import sequential
from coraplex.plans.plan_node import PlanNode
from coraplex.robot_plans.actions.core.navigation import LookAtAction, NavigateAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.view_manager import ViewManager
from experiments.demonstration import RobotDemonstration
from semantic_digital_twin.adapters.package_resolver import CompositePathResolver
from semantic_digital_twin.api import (
    BodySpecification,
    RobotSpecification,
    WorldSpecification,
)
from semantic_digital_twin.robots.stretch import Stretch
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    CheezeIt,
    Door,
    Handle,
    Hinge,
    Shelf,
    ShelfLayer,
    SideTable,
    Sofa,
    Wall,
    Wardrobe,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Vector3
from semantic_digital_twin.spatial_types.derivatives import DerivativeMap
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.degree_of_freedom import (
    DegreeOfFreedomLimits,
)
from semantic_digital_twin.world_description.geometry import Scale

CEREAL_NAME = "cheeze_it.obj"
"""
Name of the transported body, which also marks whether the scene was already spawned.
"""


def apartment_mesh_path(mesh_file_name: str) -> str:
    """
    Resolve one of the apartment's visual meshes to a local file path.
    """
    return CompositePathResolver().resolve(
        f"package://iai_apartment/meshes/visual/{mesh_file_name}"
    )


@dataclass
class StretchApartmentDemonstration(RobotDemonstration):
    """
    Stretch transports a cereal box from a shelf to a bedside table in the apartment.
    """

    ros_node_name: ClassVar[str] = "stretch_demo_node"

    def build_simulated_world(self) -> World:
        """
        Put the Stretch on its drive in a world holding nothing but a map body.
        """
        return WorldSpecification(
            world=World.create_with_root_body(),
            robots=[
                RobotSpecification(
                    semantic_annotation_type=Stretch,
                    odom_T_robot_start=HomogeneousTransformationMatrix.from_xyz_rpy(
                        1, 1
                    ),
                )
            ],
        ).to_domain_object()

    def is_scene_populated(self, world: World) -> bool:
        return world.is_kinematic_structure_entity_in_world_by_name(CEREAL_NAME)

    def populate_scene(self, world: World) -> None:
        """
        Spawn the shelf, the furniture meshes and the cereal box.
        """
        # %% shelf

        with world.modify_world():
            # The hollow-case geometry parameters live on the root specification, so the
            # shelf is spawned from a specification rather than the plain body factory.
            shelf = Shelf.get_specification(
                "shelf",
                Shelf.get_default_root_specification(
                    scale=Scale(0.305, 0.85, 1.9), wall_thickness=0.035
                ),
            ).spawn(
                world,
                parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    0.455 + (0.85 / 2),
                    -0.17,
                    1.9 / 2,
                    yaw=-np.pi / 2,
                    reference_frame=world.root,
                ),
            )
            for layer_name, layer_height in [
                ("shelf_layer1", 0.283),
                ("shelf_layer2", 0.63),
                ("shelf_layer3", 1.265),
                ("shelf_layer4", 1.613),
            ]:
                shelf.add(
                    ShelfLayer.create_with_new_body_in_world(
                        world=world,
                        name=layer_name,
                        world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                            0.455 + (0.85 / 2),
                            -0.17,
                            layer_height,
                            yaw=-np.pi / 2,
                            reference_frame=world.root,
                        ),
                        scale=Scale(0.305, 0.85, 0.018),
                    )
                )

            Wall.create_with_new_body_in_world(
                world=world,
                name="wall",
                world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    0, (2.81 / 2), 0, reference_frame=world.root
                ),
                scale=Scale(0.03, 2.81, 0.265),
            )

        # %% bedside table

        SideTable.get_specification(
            "bedside_table.dae",
            BodySpecification.mesh(
                "bedside_table.dae",
                apartment_mesh_path("bedside_table.dae"),
                parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=1.92, y=2.68, yaw=17.8 * (-np.pi / 32)
                ),
            ),
        ).spawn(world)

        # %% sofa

        # The sofa rests on the floor, so its placement needs the height of its own
        # geometry, which only the specification can measure.
        sofa_body = BodySpecification.mesh(
            "sofa_bed.obj", apartment_mesh_path("sofa_bed.obj")
        )
        Sofa.get_specification("sofa_bed.obj", sofa_body).spawn(
            world,
            parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=1.0, y=3.15, z=sofa_body.scale.z / 2, yaw=17.75 * (-np.pi / 32)
            ),
        )

        # %% walls

        Wall.get_specification(
            "walls.dae",
            BodySpecification.mesh(
                "walls.dae",
                apartment_mesh_path("walls.dae"),
                parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=-7.34, y=1.43, z=-0.2, yaw=0
                ),
            ),
        ).spawn(world)

        # %% wardrobe

        door_definitions = [
            ("left", "wardrobe_door_left.dae", -0.460513, 0.5, -np.pi / 2),
            ("right", "wardrobe_door_right.dae", 0.460513, -0.5, np.pi / 2),
        ]

        doors = [
            Door.get_specification(
                door_mesh,
                BodySpecification.mesh(
                    door_mesh,
                    apartment_mesh_path(door_mesh),
                    parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                        -0.3246, door_y
                    ),
                ),
                part_specifications={
                    "handle": Handle.get_specification(
                        f"wardrobe_door_handle_{side}",
                        BodySpecification.mesh(
                            f"wardrobe_door_handle_{side}",
                            apartment_mesh_path("wardrobe_door_handle.dae"),
                            parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                                -0.032089, handle_y, 0.973703
                            ),
                        ),
                    )
                },
            )
            for side, door_mesh, handle_y, door_y, _ in door_definitions
        ]

        wardrobe = Wardrobe.get_specification(
            "wardrobe.dae",
            BodySpecification.mesh(
                "wardrobe.dae",
                apartment_mesh_path("wardrobe.dae"),
                parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=2, y=-0.17, yaw=-np.pi / 2
                ),
            ),
            part_specifications={"doors": doors},
        ).spawn(world)

        # The hinges are mounted after the doors are on the wardrobe: mounting a part
        # moves it under the whole, which would undo a joint inserted beforehand.
        #
        # Each door mesh has its origin on the edge it hangs from, so a hinge placed at
        # the door's own frame already sits on the rotation axis. Both leaves swing about
        # the shared vertical axis, in opposite directions, to open towards -x.
        doors_by_name = {door.root.name.name: door for door in wardrobe.doors}
        hinge_poses = {
            door_mesh: doors_by_name[door_mesh].root.global_transform
            for _, door_mesh, _, _, _ in door_definitions
        }
        with world.modify_world():
            for side, door_mesh, _, _, opening_angle in door_definitions:
                doors_by_name[door_mesh].add(
                    Hinge.create_with_new_body_in_world(
                        world=world,
                        name=f"wardrobe_hinge_{side}",
                        world_root_T_self=hinge_poses[door_mesh],
                        parent_connection_specification=Hinge.parent_connection_specification(
                            axis=Vector3.Z(),
                            dof_limits=DegreeOfFreedomLimits(
                                lower=DerivativeMap[float](
                                    position=min(0.0, opening_angle)
                                ),
                                upper=DerivativeMap[float](
                                    position=max(0.0, opening_angle)
                                ),
                            ),
                        ),
                    )
                )

        # %% cereal

        CheezeIt.get_specification(
            CEREAL_NAME,
            BodySpecification.mesh(
                CEREAL_NAME,
                apartment_mesh_path(CEREAL_NAME),
                parent_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=-0.05, y=0.0, z=0.115
                ),
            ),
        ).spawn(world, parent=world.get_body_by_name("shelf_layer2"))

    def build_context(self, world: World) -> Context:
        """
        Build the plan context around the Stretch in ``world``.

        ..note:: The ROS node has to be in the context for a real robot.
        """
        return Context(
            world=world,
            robot=world.get_semantic_annotations_by_type(Stretch)[0],
            ros_node=self.ros_node,
            evaluate_conditions=False,
            alternative_motion_mappings=[
                StretchMoveToolCenterPoint,
                StretchMoveSim,
                StretchMoveReal,
                StretchClose,
            ],
        )

    def build_plan(self, context: Context) -> PlanNode:
        """
        Pick the cereal box off the shelf and place it on the bedside table.
        """
        world = context.world

        # Stretch has a single arm, so ViewManager resolves Arms.LEFT to it. Going
        # through the ViewManager guarantees this is the same end effector the pick and
        # place actions will drive.
        grasp_description = GraspDescription(
            ApproachDirection.FRONT,
            VerticalAlignment.NoAlignment,
            ViewManager.get_arm_view(Arms.LEFT, context.robot).end_effector,
        )

        cereal_body = world.get_body_by_name(CEREAL_NAME)
        shelf_layer_body = world.get_body_by_name("shelf_layer2")
        bedside_table_body = world.get_body_by_name("bedside_table.dae")

        return sequential(
            [
                ParkArmsAction(Arms.BOTH),
                NavigateAction(
                    Pose.from_xyz_rpy(
                        0.8, 0.6, 0, yaw=-np.pi / 2, reference_frame=world.root
                    )
                ),
                LookAtAction(Pose.from_xyz_rpy(reference_frame=shelf_layer_body)),
                PickUpAction(cereal_body, Arms.LEFT, grasp_description),
                ParkArmsAction(Arms.BOTH),
                NavigateAction(
                    Pose.from_xyz_rpy(
                        2, 2, 0, yaw=np.pi / 2, reference_frame=world.root
                    )
                ),
                PlaceAction(
                    object_designator=cereal_body,
                    target_location=Pose.from_xyz_rpy(
                        x=0.1, z=0.49, yaw=np.pi, reference_frame=bedside_table_body
                    ),
                    arm=Arms.LEFT,
                ),
                ParkArmsAction(Arms.BOTH),
            ],
            context,
        )


def main(execution_type: ExecutionType = ExecutionType.SIMULATED) -> None:
    """
    Run the demonstration.

    :param execution_type: Whether to drive the real robot or simulate it.
    """
    StretchApartmentDemonstration(execution_type=execution_type).run()


if __name__ == "__main__":
    main()
