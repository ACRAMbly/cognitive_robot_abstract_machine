import os
import rclpy

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.world_entity import Body

from coraplex.datastructures.dataclasses import Context
from coraplex.execution_environment import simulated_robot
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.core.pick_up import PickUpAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.robot_plans.actions.core.placing import PlaceAction
from coraplex.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from krrood.entity_query_language.factories import entity, variable

world = URDFParser.from_file(Tracy.get_ros_file_path()).parse()
assets_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")

def spawn_body(world, obj, x, y, z):
    with obj.modify_world():
        obj.bodies[0].name.prefix = "objects"
    with world.modify_world():
        connection = Connection6DoF.create_with_dofs(
            parent=world.root, child=obj.root, world=world
        )
        world.merge_world(obj, connection)
        connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
            x=x, y=y, z=z, reference_frame=obj
        )

spawn_body(world, STLParser(os.path.join(assets_dir, "child_cube_0_scaled.stl")).parse(), 0.8, -0.5, 0.93)
spawn_body(world, STLParser(os.path.join(assets_dir, "child_cube_1_scaled.stl")).parse(), 0.8, 0, 0.93)
spawn_body(world, STLParser(os.path.join(assets_dir, "child_cube_2_scaled.stl")).parse(), 0.8, 0.5, 0.93)

rclpy.init()

node = rclpy.create_node("viz_marker")
v = VizMarkerPublisher(_world=world, node=node).with_tf_publisher()

tracy = Tracy.from_world(world)
context = Context(world=world, robot=tracy)

context.evaluate_conditions = False

objects = world.bodies

obj = variable(Body, domain=objects)
query = entity(obj).where(
    obj.name.prefix == "objects"
)

results = list(query.evaluate())

for result in results:
    print(result.name)

red_box = results[0]
yellow_box = results[1]
blue_box = results[2]

stack_pos_x = 1
stack_pos_y = 0


def select_arm(cube: Body):
    cube_y = float(
        cube.global_pose.position.to_np().reshape(-1)[1]
    )

    end_effectors = Tracy.get_end_effectors(tracy)

    if cube_y > 0:
        return Arms.LEFT, end_effectors[0]
    else:
        return Arms.RIGHT, end_effectors[1]


red_arm, red_end_effector = select_arm(red_box)
yellow_arm, yellow_end_effector = select_arm(yellow_box)
blue_arm, blue_end_effector = select_arm(blue_box)

with simulated_robot:
    sequential(
        [
            ParkArmsAction(Arms.BOTH),
            PickUpAction(
                red_box,
                red_arm,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    red_end_effector,
                ),
            ),
            PlaceAction(
                red_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    0.953,
                    reference_frame=world.root,
                ),
                red_arm,
            ),
            ParkArmsAction(Arms.BOTH),
            PickUpAction(
                yellow_box,
                yellow_arm,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    yellow_end_effector,
                ),
            ),
            PlaceAction(
                yellow_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    1.003,
                    reference_frame=world.root,
                ),
                yellow_arm,
            ),
            ParkArmsAction(Arms.BOTH),
            PickUpAction(
                blue_box,
                blue_arm,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    blue_end_effector,
                ),
            ),
            PlaceAction(
                blue_box,
                Pose.from_xyz_rpy(
                    stack_pos_x,
                    stack_pos_y,
                    1.053,
                    reference_frame=world.root,
                ),
                blue_arm,
            ),
            ParkArmsAction(Arms.BOTH),
        ],
        context=context,
    ).plan.perform()
