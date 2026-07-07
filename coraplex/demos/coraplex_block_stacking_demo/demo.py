import rclpy

from coraplex.datastructures.dataclasses import Context
from coraplex.datastructures.enums import ApproachDirection, Arms, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription
from coraplex.execution_environment import simulated_robot
from coraplex.plans.factories import sequential
from coraplex.plans.plan import Plan
from coraplex.robot_plans.actions.composite.transporting import PickAndPlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from semantic_digital_twin.adapters.ros.visualization.viz_marker import (
    VizMarkerPublisher,
)
from semantic_digital_twin.adapters.ros.world_synchronizer import WorldSynchronizer
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.geometry import Box, Color, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

def spawn_free_box(
        spawn_world: World,
        name: str = "box",
        position: tuple = (0.0, 0.0, 1.5),
        scale: Scale = Scale(0.1, 0.1, 0.1),
        color: Color = Color(1.0, 1.0, 0.0, 1.0)
) -> Body:
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

        # Set the initial world pose of the box via the 6-DoF DoF state.
        connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
            x=position[0],
            y=position[1],
            z=position[2],
            reference_frame=spawn_body,
        )

    return spawn_body

def setup_world() -> World:
    tracy_world: World = URDFParser.from_file(Tracy.get_ros_file_path()).parse()

    spawn_free_box(tracy_world, "box1", (0.8, 0.5, 0.93), color=Color(1.0, 0.0, 0.0, 1.0))
    spawn_free_box(tracy_world, "box2", (0.8, -0.5, 0.93), color=Color(0.0, 1.0, 0.0, 1.0))
    spawn_free_box(tracy_world, "box3", (0.8, 0, 0.93), color=Color(0.0, 0.0, 1.0, 1.0))
    return tracy_world

def build_plan(world: World, tracy: Tracy, context: Context) -> Plan | None:
    return sequential(
        [
            ParkArmsAction(Arms.BOTH),
            PickAndPlaceAction(
                world.get_body_by_name("box3"),
                Pose.from_xyz_rpy(0.6, 0.0, 0.93, reference_frame=world.root),
                Arms.RIGHT,
                GraspDescription(ApproachDirection.FRONT, VerticalAlignment.TOP, tracy.right_arm.end_effector),
            ),
            PickAndPlaceAction(
                world.get_body_by_name("box1"),
                Pose.from_xyz_rpy(0.6, 0.0, 1.03, reference_frame=world.root),
                Arms.LEFT,
                GraspDescription(ApproachDirection.FRONT, VerticalAlignment.TOP, tracy.left_arm.end_effector),
            ),
            PickAndPlaceAction(
                world.get_body_by_name("box2"),
                Pose.from_xyz_rpy(0.6, 0.0, 1.13, reference_frame=world.root),
                Arms.RIGHT,
                GraspDescription(ApproachDirection.FRONT, VerticalAlignment.TOP, tracy.right_arm.end_effector),
            ),
        ],
        context=context,
    ).plan

def run_simulation():
    world = setup_world()
    tracy = Tracy.from_world(world)

    node = rclpy.create_node("viz_marker")
    VizMarkerPublisher(_world=world, node=node).with_tf_publisher()

    context = Context(world=world, robot=tracy)
    context.evaluate_conditions = False
    plan: Plan | None = build_plan(world, tracy, context)

    if plan is None:
        print("No valid plan could be generated. Exiting.")
        return

    with simulated_robot:
        plan.perform()

def main():
    rclpy.init()
    try:
        run_simulation()
    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()