import threading

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor

from coraplex.alternative_motion_mappings.stretch_motion_mapping import (
    StretchClose,
    StretchMoveReal,
    StretchMoveSim,
    StretchMoveToolCenterPoint,
)
from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import (
    Arms,
    ApproachDirection,
    VerticalAlignment,
    ExecutionType,
)
from coraplex.datastructures.grasp import GraspDescription
from coraplex.execution_environment import ExecutionEnvironment
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.navigation import NavigateAction, LookAtAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.view_manager import ViewManager
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.adapters.package_resolver import CompositePathResolver
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.stretch import Stretch
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Shelf,
    ShelfLayer,
    Wall,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.connections import (
    DifferentialDrive,
    FixedConnection,
)
from semantic_digital_twin.world_description.geometry import Scale
from semantic_digital_twin.world_description.world_entity import Body

rclpy.init()
node = rclpy.create_node("stretch_demo_node")

executor = SingleThreadedExecutor()
executor.add_node(node)

thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
thread.start()

execution_type = ExecutionType.REAL

if execution_type == ExecutionType.SIMULATED:
    stretch_parse = URDFParser.from_file(
        "package://stretch_description/urdf/stretch_description_RE2V0_tool_stretch_dex_wrist.xacro"
    )
    world = stretch_parse.parse()
    Stretch.from_world(world)
    with world.modify_world():
        world.add_kinematic_structure_entity(map_body := Body(name=PrefixedName("map")))
        world.add_connection(
            drive := DifferentialDrive.create_with_dofs(world, map_body, world.root)
        )
    VizMarkerPublisher(_world=world, node=node).with_tf_publisher()
    drive.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        1, 1, reference_frame=world.root
    )
else:
    # 300s matches giskardpy's own client (giskardpy/middleware/ros2/python_interface.py),
    # which waits this long for the same race: the world-fetcher server is still parsing
    # the URDF and starting up when the default 10s budget would otherwise expire.
    world = fetch_world_from_service(node=node, timeout_seconds=300)
    WorldSynchronizer(_world=world, node=node)

if not world.is_kinematic_structure_entity_in_world_by_name("cheeze_it.obj"):
    # ------------------------------------------------------------------ shelf
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

    # ---------------------------------------------------------- bedside table
    bedside_table_world = STLParser(
        file_path=CompositePathResolver().resolve(
            "package://iai_apartment/meshes/visual/bedside_table.dae"
        )
    ).parse()
    world_T_bedside_table = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=1.92, y=2.68, yaw=17.8 * (-np.pi / 32), reference_frame=world.root
    )
    world.merge_world(
        bedside_table_world,
        FixedConnection.create_with_dofs(
            world=world,
            parent=world.root,
            child=bedside_table_world.root,
            parent_T_connection_expression=world_T_bedside_table,
        ),
    )

    # -------------------------------------------------------------------- sofa
    sofa_world = STLParser(
        file_path=CompositePathResolver().resolve(
            "package://iai_apartment/meshes/visual/sofa_bed.obj"
        )
    ).parse()
    sofa_height = (
        sofa_world.root.collision.max_point[2] - sofa_world.root.collision.min_point[2]
    )
    world_T_sofa = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=1.0,
        y=3.15,
        z=sofa_height / 2,
        yaw=17.75 * (-np.pi / 32),
        reference_frame=world.root,
    )
    world.merge_world(
        sofa_world,
        FixedConnection.create_with_dofs(
            world=world,
            parent=world.root,
            child=sofa_world.root,
            parent_T_connection_expression=world_T_sofa,
        ),
    )

    # ------------------------------------------------------------------- walls
    wall_world = STLParser(
        file_path=CompositePathResolver().resolve(
            "package://iai_apartment/meshes/visual/walls.dae"
        )
    ).parse()
    world_T_wall = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=-7.34, y=1.43, z=-0.2, yaw=0, reference_frame=world.root
    )
    world.merge_world(
        wall_world,
        FixedConnection.create_with_dofs(
            world=world,
            parent=world.root,
            child=wall_world.root,
            parent_T_connection_expression=world_T_wall,
        ),
    )

    # ---------------------------------------------------------------- wardrobe
    wardrobe_world = STLParser(
        file_path=CompositePathResolver().resolve(
            "package://iai_apartment/meshes/visual/wardrobe.dae"
        )
    ).parse()

    for side, door_mesh, handle_y, door_y in [
        ("left", "wardrobe_door_left.dae", -0.460513, 0.5),
        ("right", "wardrobe_door_right.dae", 0.460513, -0.5),
    ]:
        door_world = STLParser(
            file_path=CompositePathResolver().resolve(
                f"package://iai_apartment/meshes/visual/{door_mesh}"
            )
        ).parse()

        handle_world = STLParser(
            file_path=CompositePathResolver().resolve(
                "package://iai_apartment/meshes/visual/wardrobe_door_handle.dae"
            )
        ).parse()
        handle_world.root.name.name = f"wardrobe_door_handle_{side}"
        door_world.merge_world(
            handle_world,
            FixedConnection.create_with_dofs(
                world=door_world,
                parent=door_world.root,
                child=handle_world.root,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    -0.032089,
                    handle_y,
                    0.973703,
                    reference_frame=door_world.root,
                ),
            ),
        )

        wardrobe_world.merge_world(
            door_world,
            FixedConnection.create_with_dofs(
                world=wardrobe_world,
                parent=wardrobe_world.root,
                child=door_world.root,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    -0.3246, door_y, reference_frame=wardrobe_world.root
                ),
            ),
        )

    world.merge_world(
        wardrobe_world,
        FixedConnection.create_with_dofs(
            world=world,
            parent=world.root,
            child=wardrobe_world.root,
            parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=2, y=-0.17, yaw=-np.pi / 2, reference_frame=world.root
            ),
        ),
    )

    # ------------------------------------------------------------------ cereal
    cereal = STLParser(
        file_path=CompositePathResolver().resolve(
            "package://iai_apartment/meshes/visual/cheeze_it.obj"
        )
    ).parse()

    with world.modify_world():
        parent = world.get_body_by_name("shelf_layer2")
        world.merge_world(
            cereal,
            FixedConnection.create_with_dofs(
                world=world,
                parent=parent,
                child=cereal.root,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=-0.05,
                    y=0.0,
                    z=0.115,
                    reference_frame=parent,
                ),
            ),
        )

robot = world.get_semantic_annotations_by_type(Stretch)[0]

# It is important to have the ros_node in the context for a real robot.
context = Context(
    world=world,
    robot=robot,
    ros_node=node,
    evaluate_conditions=False,
    alternative_motion_mappings=[
        StretchMoveToolCenterPoint,
        StretchMoveSim,
        StretchMoveReal,
        StretchClose,
    ],
)

# Stretch has a single arm, so ViewManager resolves Arms.LEFT to it. Going through the
# ViewManager guarantees this is the same end effector the pick/place actions will drive.
grasp_description = GraspDescription(
    ApproachDirection.FRONT,
    VerticalAlignment.NoAlignment,
    ViewManager.get_arm_view(Arms.LEFT, robot).end_effector,
)

cereal_body = world.get_body_by_name("cheeze_it.obj")
shelf_layer_body = world.get_body_by_name("shelf_layer2")
bedside_table_body = world.get_body_by_name("bedside_table.dae")

plan = sequential(
    [
        ParkArmsAction(Arms.BOTH),
        NavigateAction(
            Pose.from_xyz_rpy(0.8, 0.6, 0, yaw=-np.pi / 2, reference_frame=world.root)
        ),
        LookAtAction(Pose.from_xyz_rpy(reference_frame=shelf_layer_body)),
        PickUpAction(cereal_body, Arms.LEFT, grasp_description),
        ParkArmsAction(Arms.BOTH),
        NavigateAction(
            Pose.from_xyz_rpy(2, 2, 0, yaw=np.pi / 2, reference_frame=world.root)
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

with ExecutionEnvironment(execution_type=execution_type, collision_avoidance=False):
    plan.perform()

rclpy.shutdown()
