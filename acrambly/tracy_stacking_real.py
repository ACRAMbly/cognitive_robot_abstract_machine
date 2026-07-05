import threading
import time
import sys
import rclpy

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.tracy import Tracy
from giskardpy.middleware.ros2.python_interface import GiskardWrapper

from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world import World

from coraplex.datastructures.dataclasses import Context
from coraplex.motion_executor import real_robot
from coraplex.plans.factories import sequential
from coraplex.robot_plans.actions.composite.transporting import PickAndPlaceAction
from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.datastructures.enums import Arms, ApproachDirection, VerticalAlignment
from coraplex.datastructures.grasp import GraspDescription

def spawn_box(spawn_world: World, name: str = "box", position: tuple = (0.0, 0.0, 1.5), scale: float = 0.1, r: float = 0.0, g: float = 0.0, b: float = 0.0) -> Body:
    spawn_body = URDFParser(f"""<?xml version="1.0"?>
        <robot name="{name}_box">
          <link name="{name}_link">
            <inertial>
              <mass value="0.1"/>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
            </inertial>
            <visual>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="{scale} {scale} {scale}"/>
              </geometry>
              <material name="{name}_mat">
                <color rgba="{r} {g} {b} 1.0"/>
              </material>
            </visual>
            <collision>
              <origin xyz="0 0 0" rpy="0 0 0"/>
              <geometry>
                <box size="{scale} {scale} {scale}"/>
              </geometry>
            </collision>
          </link>
        </robot>
        """).parse()

    with spawn_world.modify_world():
        connection = Connection6DoF.create_with_dofs(
            parent=spawn_world.root,
            child=spawn_body.root,
            world=spawn_world,
        )
        spawn_world.merge_world(spawn_body, connection)

    time.sleep(0.5)
    # Set the initial world pose of the box via the 6-DoF DoF state.
    connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=position[0],
        y=position[1],
        z=position[2],
        reference_frame=spawn_body,
    )

    box = spawn_world.get_kinematic_structure_entity_by_name(f"{name}_link")
    return box

def setup_world(giskard: GiskardWrapper):
    tracy_world = giskard.world

    print("Adding boxes to world.")
    red = spawn_box(tracy_world, "red", (0.8, 0.5, 0.93), 0.1, 1.0, 0.0, 0.0)
    green = spawn_box(tracy_world, "green", (0.8, -0.5, 0.93), 0.1, 0.0, 1.0, 0.0)
    blue = spawn_box(tracy_world, "blue", (0.8, 0, 0.93), 0.1, 0.0, 0.0, 1.0)
    return tracy_world, red, green, blue

def main():
    rclpy.init()

    node = rclpy.create_node("coraplex_real_stacking")

    # Giskard action/service clients need the node to spin.
    spinner = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    spinner.start()

    print("Connecting to Giskard...")
    giskard = GiskardWrapper(node_handle=node)
    print("Getting live world from Giskard...")

    world, red_box, green_box, blue_box = setup_world(giskard)
    if world is None:
        print("GiskardWrapper.world is None.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print(f"World received with {len(list(world.bodies))} bodies.")

    print("Building Tracy semantic robot from giskard world...")
    try:
        tracy = giskard.robot
    except Exception as e:
        print(f"Could not build Tracy from Giskard world: {e}")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    context = Context(world=world, robot=tracy, ros_node=node)
    context.evaluate_conditions = False

    plan = sequential(
        [
            ParkArmsAction(Arms.BOTH),
            PickAndPlaceAction(
                red_box,
                Pose.from_xyz_rpy(
                    0.6,
                    0.0,
                    0.93,
                    reference_frame=world.root
                ),
                Arms.RIGHT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[1],
                ),
            ),
            PickAndPlaceAction(
                world.get_body_by_name("box1"),
                Pose.from_xyz_rpy(
                    0.6,
                    0.0,
                    1.03,
                    reference_frame=world.root
                ),
                Arms.LEFT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[0]
                ),
            ),
            PickAndPlaceAction(
                world.get_body_by_name("box2"),
                Pose.from_xyz_rpy(
                    0.6,
                    0.0,
                    1.13,
                    reference_frame=world.root
                ),
                Arms.RIGHT,
                GraspDescription(
                    ApproachDirection.FRONT,
                    VerticalAlignment.TOP,
                    Tracy.get_end_effectors(tracy)[1]
                ),
            ),
        ],
        context=context,
    ).plan

    print("Executing ParkArmsAction on REAL robot through Giskard...")
    print("Keep E-stop reachable.")

    with real_robot:
        plan.perform()

    node.destroy_node()
    rclpy.shutdown()
    print("ParkArmsAction completed.")


if __name__ == "__main__":
    main()