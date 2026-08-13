import os
import threading

import rclpy
from math import pi

from coraplex.alternative_motion_mappings.tracy_motion_mapping import TracyRealMoveGripperMotion
from semantic_digital_twin.adapters.ros.world_fetcher import fetch_world_from_service
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.robots.robot_parts import AbstractRobot

from coraplex.plans.attachment_nodes import AttachNode
from coraplex.robot_plans import MoveGripperMotion
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.world_entity import Body

from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import simulated_robot
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.pick_up import PickUpAction, ReachAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from krrood.entity_query_language.factories import entity, variable
from sub_parts.shared.utils import spawn_body

rclpy.init()

sim = True
assets_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")

node = rclpy.create_node("coraplex_task_runner")

spinner = threading.Thread(
    target=rclpy.spin,
    args=(node,),
    daemon=True,
)
spinner.start()

if sim:
    world = URDFParser.from_file(Tracy.get_ros_file_path()).parse()
    v = VizMarkerPublisher(_world=world, node=node).with_tf_publisher()

    tracy = Tracy.from_world(world)
    context = Context(world=world, robot=tracy, evaluate_conditions=False)

else:
    world = fetch_world_from_service(node, timeout_seconds=300)
    WorldSynchronizer(_world=world, node=node, synchronous=True)

    tracy = world.get_semantic_annotations_by_type(AbstractRobot)[0]

    context = Context(
        world=world,
        robot=tracy,
        ros_node=node,
        alternative_motion_mappings=[TracyRealMoveGripperMotion],
        evaluate_conditions=False,
    )

spawn_body(world, (0.5, -0.5, 0.93), (0.5, 0, 0), "mesh", mesh_filename="child_cube_0_scaled.stl")
spawn_body(world, (0.8, -0.6, 0.93), (0.5, 0, 0), "mesh", mesh_filename="child_cube_1_scaled.stl")
spawn_body(world, (1.0, -0.3, 0.93), (0.5, 0, 0), "mesh", mesh_filename="child_cube_2_scaled.stl")

objects = world.bodies

obj = variable(Body, domain=objects)
query = entity(obj).where(
    obj.name.prefix == "objects"
)

results = list(query.evaluate())

for result in results:
    print(result.name)

cube0 = results[0]
cube1 = results[1]
cube2 = results[2]

end_effectors = Tracy.get_end_effectors(tracy)

grasp_descriptions = {
    "cube0" : {
        "blue_grasp" : GraspDescription(
            approach_direction=ApproachDirection.LEFT,
            vertical_alignment=VerticalAlignment.TOP,
            end_effector=end_effectors[0],
            grasp_offset=Point3(0.0, 0.08, 0.0),
        ),
        "red_grasp" : GraspDescription(
            approach_direction=ApproachDirection.LEFT,
            vertical_alignment=VerticalAlignment.TOP,
            end_effector=end_effectors[1],
            grasp_offset=Point3(0.0, -0.08, 0.0),
        )
    },
    "cube1" : {
        "blue_grasp" : GraspDescription(
            approach_direction=ApproachDirection.FRONT,
            vertical_alignment=VerticalAlignment.TOP,
            end_effector=end_effectors[0],
            grasp_offset=Point3(-0.06, 0.033, -0.01),
        ),
        "red_grasp" : GraspDescription(
            approach_direction=ApproachDirection.FRONT,
            vertical_alignment=VerticalAlignment.TOP,
            end_effector=end_effectors[1],
            grasp_offset=Point3(0.06, -0.017, -0.01),
        )
    },
    "cube2" : {
        "blue_grasp" : GraspDescription(
            approach_direction=ApproachDirection.BACK,
            vertical_alignment=VerticalAlignment.TOP,
            end_effector=end_effectors[0],
            grasp_offset=Point3(-0.09, -0.015, -0.01),
        ),
        "red_grasp" : GraspDescription(
            approach_direction=ApproachDirection.FRONT,
            vertical_alignment=VerticalAlignment.TOP,
            end_effector=end_effectors[1],
            grasp_offset=Point3(0.016, -0.015, 0.03),
        )
    }
}

handover_pose = {
    "cube0" : Pose.from_xyz_rpy(0.8, -0.08, 1.0, reference_frame=world.root),
    "cube1" : Pose.from_xyz_rpy(0.8, -0.08, 1.0, yaw=-pi/2, reference_frame=world.root),
    "cube2" : Pose.from_xyz_rpy(0.8, -0.08, 1.0, yaw=-pi/2, reference_frame=world.root)
}

place_pose = {
    "cube0": Pose.from_xyz_rpy(0.5, 0.4, 0.95, reference_frame=world.root),
    "cube1" : Pose.from_xyz_rpy(0.6, 0.4, 0.95, reference_frame=world.root),
    "cube2": Pose.from_xyz_rpy(1.0, 0.4, 0.95, reference_frame=world.root),
}

def get_plan(cube, j):
    cube_name = "cube" + str(j)
    return sequential(
        [
            ParkArmsAction(Arms.BOTH),
            PickUpAction(
                cube,
                Arms.RIGHT,
                grasp_descriptions[cube_name]["red_grasp"],
            ),
            ReachAction(
                target_pose=handover_pose[cube_name],
                object_designator=cube,
                arm=Arms.RIGHT,
                grasp_description=grasp_descriptions[cube_name]["red_grasp"],
            ),
            ReachAction(
                target_pose=grasp_descriptions[cube_name][
                    "blue_grasp"
                ].grasp_target_pose(cube),
                object_designator=cube,
                arm=Arms.LEFT,
                grasp_description=grasp_descriptions[cube_name]["blue_grasp"],
            ),
            MoveGripperMotion(motion=GripperState.CLOSE, gripper=Arms.LEFT),
            AttachNode(body=cube, new_parent=end_effectors[0].tool_frame),
            MoveGripperMotion(motion=GripperState.OPEN, gripper=Arms.RIGHT),
            ParkArmsAction(Arms.RIGHT),
            PlaceAction(
                object_designator=cube,
                target_location=place_pose[cube_name],
                arm=Arms.LEFT,
            ),
            ParkArmsAction(Arms.BOTH),
        ],
        context=context,
    ).plan


with simulated_robot:
    for i,j in enumerate([cube0, cube1, cube2]):
        get_plan(j, i).perform()

cube0.remove_from_world()
cube1.remove_from_world()
cube2.remove_from_world()