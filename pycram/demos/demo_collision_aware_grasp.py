import time
import os

from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.grasp import GraspDescription
from pycram.datastructures.dataclasses import Context
from pycram.language import SequentialPlan
from pycram.motion_executor import simulated_robot
from pycram.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from pycram.robot_plans import (
    ParkArmsActionDescription,
    PickUpActionDescription,
    PlaceActionDescription,
)

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix as tm,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.adapters.package_resolver import PackageUriResolver
from semantic_digital_twin.world_description.geometry import Box, Scale, Color
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.robots.tracy import Tracy

import rclpy
import threading
from rclpy.executors import SingleThreadedExecutor

from pycram.robot_plans.actions.composite.collision_aware_transport import (
    CollisionAwareTransportActionDescription,
    GraspClassifier,
    load_grasp_data,
)

from pycram.error_recovery.plan_guardian import PlanGuardian, with_error_recovery


# WORLD SETUP

body_box1 = Body(name=PrefixedName("box1", "PhysicalObject"))
body_box2 = Body(name=PrefixedName("box2", "PhysicalObject"))
body_box3 = Body(name=PrefixedName("box3", "PhysicalObject"))
body_box4 = Body(name=PrefixedName("box4", "PhysicalObject"))
body_box5 = Body(name=PrefixedName("box5", "PhysicalObject"))
body_box6 = Body(name=PrefixedName("box6", "PhysicalObject"))

box1_start = (0.75, 0.10, 0.9, 0, 0, 0)
box2_start = (0.75, 0.50, 0.9, 0, 0, 0)
box3_start = (0.75, 0.25, 0.9, 0, 0, 0)
box4_start = (0.75, -0.10, 0.9, 0, 0, 0)
box5_start = (0.75, -0.50, 0.9, 0, 0, 0)
box6_start = (0.75, -0.25, 0.9, 0, 0, 0)

# All boxes go to the SAME target — collision detection will offset them
shared_target = PoseStamped.from_list(position=[0.75, 0.00, 0.9])

box1 = Box(
    tm(reference_frame=body_box1),
    scale=Scale(0.05, 0.05, 0.05),
    color=Color(1, 0, 0, 1),
)
box2 = Box(
    tm(reference_frame=body_box2),
    scale=Scale(0.05, 0.05, 0.05),
    color=Color(0, 1, 0, 1),
)
box3 = Box(
    tm(reference_frame=body_box3),
    scale=Scale(0.05, 0.05, 0.05),
    color=Color(0, 0, 1, 1),
)
box4 = Box(
    tm(reference_frame=body_box4),
    scale=Scale(0.05, 0.05, 0.05),
    color=Color(1, 0, 0, 1),
)
box5 = Box(
    tm(reference_frame=body_box5),
    scale=Scale(0.05, 0.05, 0.05),
    color=Color(0, 1, 0, 1),
)
box6 = Box(
    tm(reference_frame=body_box6),
    scale=Scale(0.05, 0.05, 0.05),
    color=Color(0, 0, 1, 1),
)

body_box1.collision = body_box1.visual = ShapeCollection([box1], body_box1)
body_box2.collision = body_box2.visual = ShapeCollection([box2], body_box2)
body_box3.collision = body_box3.visual = ShapeCollection([box3], body_box3)
body_box4.collision = body_box4.visual = ShapeCollection([box4], body_box4)
body_box5.collision = body_box5.visual = ShapeCollection([box5], body_box5)
body_box6.collision = body_box6.visual = ShapeCollection([box6], body_box6)

# Load Tracy URDF with correct resolver
tracy_urdf_path = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "..",
    "semantic_digital_twin",
    "resources",
    "urdf",
    "tracy.urdf",
)

with open(tracy_urdf_path, "r") as f:
    urdf_str = f.read()

tracy_world = URDFParser(urdf=urdf_str, path_resolver=PackageUriResolver()).parse()
robot_view = Tracy.from_world(tracy_world)
root = tracy_world.root

# Add boxes to world
with tracy_world.modify_world():
    c_root_box1 = Connection6DoF.create_with_dofs(
        world=tracy_world, parent=root, child=body_box1
    )
    tracy_world.add_connection(c_root_box1)

    c_root_box2 = Connection6DoF.create_with_dofs(
        world=tracy_world, parent=root, child=body_box2
    )
    tracy_world.add_connection(c_root_box2)

    c_root_box3 = Connection6DoF.create_with_dofs(
        world=tracy_world, parent=root, child=body_box3
    )
    tracy_world.add_connection(c_root_box3)

    c_root_box4 = Connection6DoF.create_with_dofs(
        world=tracy_world, parent=root, child=body_box4
    )
    tracy_world.add_connection(c_root_box4)

    c_root_box5 = Connection6DoF.create_with_dofs(
        world=tracy_world, parent=root, child=body_box5
    )
    tracy_world.add_connection(c_root_box5)

    c_root_box6 = Connection6DoF.create_with_dofs(
        world=tracy_world, parent=root, child=body_box6
    )
    tracy_world.add_connection(c_root_box6)

c_root_box1.origin = tm.from_xyz_rpy(*box1_start, reference_frame=root)
c_root_box2.origin = tm.from_xyz_rpy(*box2_start, reference_frame=root)
c_root_box3.origin = tm.from_xyz_rpy(*box3_start, reference_frame=root)
c_root_box4.origin = tm.from_xyz_rpy(*box4_start, reference_frame=root)
c_root_box5.origin = tm.from_xyz_rpy(*box5_start, reference_frame=root)
c_root_box6.origin = tm.from_xyz_rpy(*box6_start, reference_frame=root)

# Initialize ROS and visualization
if not rclpy.ok():
    rclpy.init()
node = rclpy.create_node("collision_aware_demo")
viz = VizMarkerPublisher(world=tracy_world, node=node)
tf_pub = TFPublisher(world=tracy_world, node=node)

executor = SingleThreadedExecutor()
executor.add_node(node)
spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()


# GRASP CLASSIFIER SETUP

grasp_classifier = None
grasp_yaml_path = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "Cube_Pad_grasps.yaml"
)

if os.path.exists(grasp_yaml_path):
    grasp_data = load_grasp_data(grasp_yaml_path)
    grasp_classifier = GraspClassifier(grasp_data)
    print("Grasp Statistics:")
    print(grasp_classifier.get_grasp_count_by_direction())
else:
    print(f"[INFO] No grasp YAML at {grasp_yaml_path}, using auto-grasp")

guardian = PlanGuardian(world=tracy_world, robot_view=robot_view, grasp_classifier=grasp_classifier)


# EXECUTE

print("COLLISION-AWARE TRANSPORT DEMO")

time.sleep(5)

@with_error_recovery(guardian)
def transport_all_boxes():
    with simulated_robot:
        ctx = Context(world=tracy_world, robot=robot_view)

        SequentialPlan(
            ctx,
            CollisionAwareTransportActionDescription(
                object_designator=body_box1,
                target_location=shared_target,
                grasp_classifier=grasp_classifier,
            ),
            CollisionAwareTransportActionDescription(
                object_designator=body_box2,
                target_location=shared_target,
                grasp_classifier=grasp_classifier,
            ),
            CollisionAwareTransportActionDescription(
                object_designator=body_box3,
                target_location=shared_target,
                grasp_classifier=grasp_classifier,
            ),
            CollisionAwareTransportActionDescription(
                object_designator=body_box4,
                target_location=shared_target,
                grasp_classifier=grasp_classifier,
            ),
            CollisionAwareTransportActionDescription(
                object_designator=body_box5,
                target_location=shared_target,
                grasp_classifier=grasp_classifier,
            ),
            CollisionAwareTransportActionDescription(
                object_designator=body_box6,
                target_location=shared_target,
                grasp_classifier=grasp_classifier,
            ),
        ).perform()
    pass

transport_all_boxes()

print("DEMO COMPLETE")

time.sleep(60)
executor.shutdown()
node.destroy_node()
try:
    rclpy.shutdown()
except Exception:
    pass