import rclpy

from acrambly.pipeline import PerceptionClientSingle
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.datastructures.grasp import GraspDescription
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.robot_plans import (
    ParkArmsActionDescription,
    StackingActionDescription,
)
from semantic_digital_twin.adapters.ros.visualization.viz_marker import VizMarkerPublisher
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_computations.raytracer import RayTracer
from semantic_digital_twin.spatial_types.spatial_types import HomogeneousTransformationMatrix as tm
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.geometry import Box, Scale, Color
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

perception = True

sw = World()
body_box1 = Body(
    name=PrefixedName("red_box", "PhysicalObject"),
)
body_box2 = Body(
    name=PrefixedName("yellow_box", "PhysicalObject"),
)
body_box3 = Body(
    name=PrefixedName("blue_box", "PhysicalObject"),
)
box1_start = [0.75, 0.00, 0.9, 0, 0, 0]
box2_start = [0.75, 0.50, 0.9, 0, 0, 0]
box3_start = [0.75, 0.25, 0.9, 0, 0, 0]

box1_target = [0.75, 0.50, 0.07, 0, 0, 0]
box2_target = [0.75, 0.50, 0.12, 0, 0, 0]
box3_target = [0.75, 0.50, 0.19, 0, 0, 0]

box1 = Box(tm(reference_frame=body_box1), scale=Scale(0.05, 0.05, 0.05), color=Color(1, 0, 0, 1))
box2 = Box(tm(reference_frame=body_box2), scale=Scale(0.05, 0.05, 0.05), color=Color(1, 1, 0, 1))
box3 = Box(tm(reference_frame=body_box3), scale=Scale(0.05, 0.05, 0.05), color=Color(0, 0, 1, 1))

body_box1.collision = body_box1.visual = ShapeCollection([box1], body_box1)
body_box2.collision = body_box2.visual = ShapeCollection([box2], body_box2)
body_box3.collision = body_box3.visual = ShapeCollection([box3], body_box3)

tracy_world = URDFParser.from_file("../semantic_digital_twin/resources/urdf/tracy.urdf").parse()
robot_view = Tracy.from_world(tracy_world)
root = tracy_world.root

# add boxes to world
with tracy_world.modify_world():
    c_root_box1 = Connection6DoF.create_with_dofs(world=tracy_world, parent=root, child=body_box1)
    tracy_world.add_connection(c_root_box1)

    c_root_box2 = Connection6DoF.create_with_dofs(world=tracy_world, parent=root, child=body_box2)
    tracy_world.add_connection(c_root_box2)

    c_root_box3 = Connection6DoF.create_with_dofs(world=tracy_world, parent=root, child=body_box3)
    tracy_world.add_connection(c_root_box3)
    c_root_box1.origin = tm.from_xyz_rpy(*box1_start, reference_frame=root, child_frame=body_box1)
    c_root_box2.origin = tm.from_xyz_rpy(*box2_start, reference_frame=root)
    c_root_box3.origin = tm.from_xyz_rpy(*box3_start, reference_frame=root)

if not rclpy.ok():
    rclpy.init()

node = rclpy.create_node("semantic_world")

viz = VizMarkerPublisher(world=tracy_world, node=node)
viz.with_tf_publisher()

client = None
if perception:
    client = PerceptionClientSingle(tracy_world, node)

    client.request(body_box1.name.name)
    client.request(body_box2.name.name)
    client.request(body_box3.name.name)

with simulated_robot:
    ctx = Context(world=tracy_world, robot=robot_view)
    SequentialPlan(
        ctx,
        StackingActionDescription(
            object_designator=body_box1,
            target=body_box2,
            arm=Arms.LEFT,
            grasp_description=GraspDescription(ApproachDirection.FRONT, VerticalAlignment.TOP,
                                               robot_view.left_arm.manipulator)),
        StackingActionDescription(
            object_designator=body_box3,
            target=body_box1,
            arm=Arms.LEFT,
            grasp_description=GraspDescription(ApproachDirection.FRONT, VerticalAlignment.TOP,
                                               robot_view.left_arm.manipulator))
    ).perform()

print("done")
node.destroy_node()

try:
    rclpy.shutdown()
except Exception:
    pass
