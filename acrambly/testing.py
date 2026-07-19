import os
import rclpy

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.geometry import Box, Scale, Color
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world import World

from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import real_robot, simulated_robot
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.composite.transporting import PickAndPlaceAction
from coraplex.robot_plans.actions.core.pick_up import PickUpAction, ReachAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.robot_plans.motions.gripper import MoveGripperMotion
from semantic_digital_twin.datastructures.definitions import GripperState
from coraplex.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
import time
from coraplex.view_manager import ViewManager
from coraplex.plans.attachment_nodes import DetachNode
from coraplex.plans.attachment_nodes import AttachNode

def spawn_free_box(
    spawn_world: World,
    name: str = "box",
    position: tuple = (0.0, 0.0, 1.5),
    scale: Scale = Scale(0.05, 0.05, 0.05),
    color: Color = Color(1.0, 1.0, 0.0, 1.0),
) -> Body:
    """Spawn a free-floating box body via the Semantic Digital Twin API."""
    spawn_body = Body(name=PrefixedName(name))

    box = Box(
        origin=HomogeneousTransformationMatrix.from_xyz_rpy(
            reference_frame=spawn_body,
        ),
        scale=scale,
        color=color,
    )
    spawn_body.collision = ShapeCollection([box], reference_frame=spawn_body)

    with spawn_world.modify_world():
        connection = Connection6DoF.create_with_dofs(
            parent=spawn_world.root,
            child=spawn_body,
            world=spawn_world,
        )
        spawn_world.add_connection(connection)

        connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
            x=position[0],
            y=position[1],
            z=position[2],
            reference_frame=spawn_body,
        )

    return spawn_body

def setup_world():
    tracy_world = URDFParser.from_file(Tracy.get_ros_file_path()).parse()

    box = spawn_free_box(
        tracy_world, "box", (0.8, 0.5, 0.93), color=Color(0.0, 0.0, 1.0, 1.0)
    )
    return tracy_world, box

world, obj = setup_world()

rclpy.init()
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher

node = rclpy.create_node("viz_marker")
v = VizMarkerPublisher(_world=world, node=node).with_tf_publisher()

tracy = Tracy.from_world(world)
context = Context(world=world, robot=tracy)

context.evaluate_conditions = False

meeting_pose = Pose.from_xyz_rpy(
        0.8, 0, 1.2,
        -1.57, 0, 0,
        reference_frame=world.root,
    )

with simulated_robot:
    sequential(
        [
            ParkArmsAction(Arms.BOTH),
            PickUpAction(
                obj,
                Arms.LEFT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[0],
                ),
            ),
            ReachAction(
                target_pose=meeting_pose,
                arm=Arms.LEFT,
                grasp_description=GraspDescription(
                    ApproachDirection.RIGHT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[0],
                ),
            ),
            ReachAction(
                target_pose=meeting_pose,
                arm=Arms.RIGHT,
                grasp_description=GraspDescription(
                    ApproachDirection.BACK,
                    VerticalAlignment.BOTTOM,
                    Tracy.get_end_effectors(tracy)[1],
                ),
            ),
            MoveGripperMotion(GripperState.CLOSE, Arms.RIGHT),
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT),
            AttachNode(body=obj, new_parent=ViewManager.get_end_effector_view(Arms.RIGHT, tracy).tool_frame),
            PlaceAction(
                target_location=Pose.from_xyz_rpy(
                    1,
                    -0.5,
                    0.93,
                    reference_frame=world.root,
                ),
                arm=Arms.RIGHT,
                object_designator=obj,
            ),
            ParkArmsAction(Arms.BOTH),
        ],
        context=context,
    ).plan.perform()
